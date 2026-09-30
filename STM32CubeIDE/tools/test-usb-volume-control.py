"""Compile the actual USB volume functions with host-only RTOS/SPI mocks.

Usage: python test-usb-volume-control.py --cc clang
Requires a host C compiler (not arm-none-eabi-gcc). No target is accessed.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "JUMBLEQ/Appli/Core/Src"


def function(source, name):
    match = re.search(r"^static [^;{\n]+\b" + name + r"\([^;{]*\)\s*\{", source, re.M)
    if not match:
        raise ValueError(f"Function not found: {name}")
    end = source.index("\n}", match.end()) + 2
    return source[match.start():end]


PREAMBLE = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "audio_usb_control_internal.h"
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX 4
#define TU_LOG1(...) ((void)0)
#define TU_ASSERT(x) do { if (!(x)) return false; } while (0)
#define TU_VERIFY(x) TU_ASSERT(x)
#define TU_U16_HIGH(x) ((uint8_t)((x) >> 8))
#define TU_U16_LOW(x) ((uint8_t)(x))
#define tu_htole16(x) ((uint16_t)(x))
enum { UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT=2, AUDIO20_CS_REQ_CUR=1,
       AUDIO20_CS_REQ_RANGE=2, AUDIO20_FU_CTRL_MUTE=1, AUDIO20_FU_CTRL_VOLUME=2 };
typedef struct { uint8_t bmRequestType, bRequest; uint16_t wValue,wIndex,wLength; } tusb_control_request_t;
typedef struct { int8_t bCur; } audio20_control_cur_1_t;
typedef struct { int16_t bCur; } audio20_control_cur_2_t;
#define audio20_control_range_2_n_t(n) struct { uint16_t wNumSubRanges; struct { int16_t bMin,bMax; uint16_t bRes; } subrange[n]; }
typedef enum { SIGMA_SPI_RESULT_OK, SIGMA_SPI_RESULT_FAILED } sigma_spi_result_t;
static int8_t s_mute[5];
static int16_t s_volume[5];
static volatile uint8_t s_feature_dirty_mask;
static bool s_feature_backoff_active;
static uint32_t s_feature_retry_started_tick;
volatile audio_usb_feature_diagnostics_t g_audio_usb_feature_diagnostics;
static uint32_t tick, irq;
static uint32_t __get_PRIMASK(void) { return irq; }
static void __disable_irq(void) { irq=1; }
static void __set_PRIMASK(uint32_t v) { irq=v; }
static uint32_t HAL_GetTick(void) { return tick; }
static uint8_t response[16];
static size_t response_size;
static bool tud_audio_buffer_and_schedule_control_xfer(uint8_t port, const tusb_control_request_t *req, const void *buf, uint16_t len) {
    (void)port; (void)req; assert(len<=sizeof response); memcpy(response,buf,len); response_size=len; return true;
}
static struct { char op; int value; } calls[16];
static int call_count, fail_call;
static bool dsp_mute[5];
static int dsp_gain[5];
static sigma_spi_result_t adau1466_control_input_from_usb_gain(uint8_t ch, int16_t db) {
    assert(db>=-100 && db<=0); assert(call_count<16);
    calls[call_count].op='G'; calls[call_count++].value=db;
    if(call_count==fail_call) return SIGMA_SPI_RESULT_FAILED;
    dsp_gain[ch]=db; return SIGMA_SPI_RESULT_OK;
}
static sigma_spi_result_t adau1466_control_input_from_usb_mute(uint8_t ch, bool value) {
    assert(call_count<16); calls[call_count].op='M'; calls[call_count++].value=value;
    if(call_count==fail_call) return SIGMA_SPI_RESULT_FAILED;
    dsp_mute[ch]=value; return SIGMA_SPI_RESULT_OK;
}
'''

TESTS = r'''
static void reset_state(void) {
    memset(s_mute,0,sizeof s_mute); memset(s_volume,0,sizeof s_volume);
    memset((void*)&g_audio_usb_feature_diagnostics,0,sizeof g_audio_usb_feature_diagnostics);
    memset(dsp_mute,0,sizeof dsp_mute); memset(dsp_gain,0,sizeof dsp_gain);
    s_feature_dirty_mask=0; s_feature_backoff_active=false; tick=0; irq=0;
    call_count=fail_call=0;
}
static tusb_control_request_t request(uint8_t ch, uint8_t sel) {
    tusb_control_request_t r={0x21,AUDIO20_CS_REQ_CUR,(uint16_t)((sel<<8)|ch),
        UAC2_ENTITY_STEREO_OUT_FEATURE_UNIT<<8,2}; return r;
}
static bool set_volume(uint8_t ch,int16_t value) {
    tusb_control_request_t r=request(ch,AUDIO20_FU_CTRL_VOLUME);
    uint8_t bytes[2]={(uint8_t)value,(uint8_t)((uint16_t)value>>8)};
    return audio20_feature_unit_set_request(0,&r,bytes);
}
static int get_volume(uint8_t ch) {
    tusb_control_request_t r=request(ch,AUDIO20_FU_CTRL_VOLUME);
    assert(audio20_feature_unit_get_request(0,&r)); assert(response_size==2);
    return (int16_t)(response[0]|((uint16_t)response[1]<<8));
}
int main(void) {
    /* Independent nearest-neighbour oracle, including attenuating ties. */
    for(int v=INT16_MIN;v<=INT16_MAX;v++) {
        int expected=INT16_MIN;
        if(v!=INT16_MIN) {
            int distance=INT32_MAX;
            for(int candidate=-12800;candidate<=0;candidate+=256) {
                int d=abs(candidate-v);
                if(d<distance) { distance=d; expected=candidate; }
            }
        }
        assert(audio20_feature_unit_normalize_volume((int16_t)v)==expected);
        assert(audio20_feature_unit_normalize_volume((int16_t)expected)==expected);
        for(uint8_t ch=0;ch<=4;ch++) {
            reset_state(); assert(set_volume(ch,(int16_t)v));
            assert(get_volume(ch)==expected);
            assert(s_feature_dirty_mask==(ch==0?15:(1u<<(ch-1))));
            assert(g_audio_usb_feature_diagnostics.request_count==1);
            assert(g_audio_usb_feature_diagnostics.master_request_count==(ch==0));
            assert(g_audio_usb_feature_diagnostics.normalized_request_count==(v!=expected));
            if(v!=expected) {
                assert(g_audio_usb_feature_diagnostics.last_normalized_requested_volume==v);
                assert(g_audio_usb_feature_diagnostics.last_normalized_channel==ch);
            }
            assert(irq==0);
        }
    }
    /* All finite gains, silence and independent explicit mute combinations. */
    for(int m=0;m<52;m++) for(int c=0;c<52;c++) for(int mask=0;mask<4;mask++) {
        reset_state();
        s_volume[0]=m==51?INT16_MIN:(int16_t)(-m*256);
        s_volume[1]=c==51?INT16_MIN:(int16_t)(-c*256);
        s_mute[0]=mask&1; s_mute[1]=(mask>>1)&1;
        bool silent=mask!=0 || m==51 || c==51;
        audio_usb_feature_apply_channel(1,s_mute,s_volume);
        assert(call_count==2 && calls[0].op==(silent?'M':'G'));
        assert(dsp_mute[1]==silent);
        assert(dsp_gain[1]==-(m==51?0:m)-(c==51?0:c));
    }
    /* GET_RANGE remains finite, not silence. */
    reset_state(); tusb_control_request_t r=request(0,AUDIO20_FU_CTRL_VOLUME);
    r.bRequest=AUDIO20_CS_REQ_RANGE;
    assert(audio20_feature_unit_get_request(0,&r));
    const uint8_t range[]={1,0,0,0xce,0,0,0,1};
    assert(response_size==sizeof range && memcmp(response,range,sizeof range)==0);
    /* Malformed requests leave an existing pending request intact. */
    assert(set_volume(1,-256)); r=request(5,AUDIO20_FU_CTRL_VOLUME);
    uint8_t bytes[2]={0,0}; assert(!audio20_feature_unit_set_request(0,&r,bytes));
    r=request(1,AUDIO20_FU_CTRL_VOLUME); r.wLength=1;
    assert(!audio20_feature_unit_set_request(0,&r,bytes));
    r=request(1,99); assert(!audio20_feature_unit_set_request(0,&r,bytes));
    r=request(1,AUDIO20_FU_CTRL_VOLUME); r.bRequest=99;
    assert(!audio20_feature_unit_set_request(0,&r,bytes));
    assert(s_feature_dirty_mask==1 && get_volume(1)==-256);
    assert(g_audio_usb_feature_diagnostics.request_count==1);
    /* New corrected value wins, pending for other channels is retained. */
    assert(set_volume(2,-512)); assert(set_volume(1,-384));
    assert(s_feature_dirty_mask==3 && get_volume(1)==-512);
    assert(g_audio_usb_feature_diagnostics.coalesced_request_count==1);
    audio_usb_control_feature_service(); assert(s_feature_dirty_mask==0);
    assert(dsp_gain[1]==-2 && dsp_gain[2]==-2);
    /* Gain failure after silence Mute must not unmute; retry uses latest state. */
    reset_state(); assert(set_volume(1,INT16_MIN)); fail_call=2;
    audio_usb_control_feature_service();
    assert(dsp_mute[1] && s_feature_dirty_mask==1 && s_feature_backoff_active);
    assert(g_audio_usb_feature_diagnostics.last_failed_operation==AUDIO_USB_FEATURE_OP_GAIN);
    assert(set_volume(1,-768)); call_count=0; fail_call=1; tick=50;
    audio_usb_control_feature_service(); assert(call_count==1 && dsp_mute[1]);
    assert(s_feature_dirty_mask==1); call_count=0; fail_call=0; tick=100;
    audio_usb_control_feature_service(); assert(!dsp_mute[1] && dsp_gain[1]==-3);
    assert(s_feature_dirty_mask==0 && g_audio_usb_feature_diagnostics.retry_count==2);
    /* Failed Mute writes stop the sequence and retain pending. */
    reset_state(); assert(set_volume(1,INT16_MIN)); fail_call=1;
    audio_usb_control_feature_service(); assert(call_count==1 && s_feature_dirty_mask==1);
    assert(g_audio_usb_feature_diagnostics.last_failed_operation==AUDIO_USB_FEATURE_OP_MUTE);
    /* Explicit Mute survives a volume silence/finite transition. */
    reset_state(); r=request(1,AUDIO20_FU_CTRL_MUTE); r.wLength=1; bytes[0]=1;
    assert(audio20_feature_unit_set_request(0,&r,bytes));
    assert(set_volume(1,INT16_MIN)); audio_usb_control_feature_service(); call_count=0;
    assert(set_volume(1,-256)); audio_usb_control_feature_service(); assert(dsp_mute[1]);
    /* Retry deadline also works across tick rollover. */
    reset_state(); tick=UINT32_MAX-20; assert(set_volume(1,INT16_MIN)); fail_call=1;
    audio_usb_control_feature_service(); call_count=0; fail_call=0; tick=28;
    audio_usb_control_feature_service(); assert(call_count==0); tick=29;
    audio_usb_control_feature_service(); assert(dsp_mute[1] && s_feature_dirty_mask==0);
    puts("PASS: 65536 inputs x 5 channels, GET_CUR/RANGE, 10816 gain/mute combinations, pending/retry/error paths");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="clang")
    args = parser.parse_args()
    source = (SRC / "audio_usb_control.c").read_text(encoding="utf-8")
    enums = re.findall(r"enum\s*\{[^}]+\};", source)
    constants = "\n".join(e for e in enums if "VOLUME_CTRL_0_DB" in e or "AUDIO_USB_FEATURE_SERVICE_PERIOD_MS" in e)
    names = ["audio_usb_feature_channel_bit", "audio20_feature_unit_normalize_volume",
             "audio_usb_feature_store_mute", "audio_usb_feature_store_volume",
             "audio_usb_feature_note_failure", "audio_usb_feature_apply_channel",
             "audio_usb_control_feature_service", "audio20_feature_unit_channel_is_valid",
             "audio20_feature_unit_get_request", "audio20_feature_unit_set_request"]
    # Generate a host translation unit from real function bodies; do not duplicate their logic.
    harness = PREAMBLE + constants + "\n".join(function(source,n) for n in names) + TESTS
    with tempfile.TemporaryDirectory(prefix="jumbleq-volume-") as tmp:
        cfile = Path(tmp) / "test.c"
        exe = Path(tmp) / "test.exe"
        cfile.write_text(harness, encoding="utf-8")
        subprocess.run([args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(SRC),
                        str(cfile), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
