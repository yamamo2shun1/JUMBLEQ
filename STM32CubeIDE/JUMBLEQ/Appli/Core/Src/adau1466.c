/*
 * adau1466.c
 *
 *  Created on: 2026/01/24
 *      Author: Shunichi Yamamoto
 */

#include "adau1466.h"

#include "SigmaStudioFW.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1_Defines.h"
#include "JUMBLEQ_DSP_ADAU146xSchematic_1_PARAM.h"

#include "cmsis_os2.h"
#include <string.h>

_Static_assert((PROGRAM_DATA_SIZE_ADAU146XSCHEMATIC_1 % 4U) == 0U,
               "SigmaStudio program data must contain complete 32-bit words");
_Static_assert((DM0_DATA_SIZE_ADAU146XSCHEMATIC_1 % 4U) == 0U,
               "SigmaStudio DM0 data must contain complete 32-bit words");
_Static_assert((DM1_DATA_SIZE_ADAU146XSCHEMATIC_1 % 4U) == 0U,
               "SigmaStudio DM1 data must contain complete 32-bit words");

#define ADAU1466_REG_PLL_LOCK     0xF004U
#define ADAU1466_REG_CLK_GEN2_M   0xF022U
#define ADAU1466_REG_SOUT_SOURCE0 0xF180U

#define ADAU1466_PLL_LOCK_TIMEOUT_MS 200U
#define ADAU1466_CLOCK_SETTLE_MS     2U
#define ADAU1466_CORE_SAMPLE_RATE_HZ 96000U

#define ADAU1466_USB_MUX_DIRECT 0U
#define ADAU1466_USB_MUX_ASRC   4U

#define ADAU1466_SOUT_FROM_DSP   0x02U
#define ADAU1466_SOUT0_FROM_ASRC2 0x13U
#define ADAU1466_SOUT1_FROM_ASRC3 0x1BU

typedef struct
{
    uint8_t clk_gen2_m;
    uint8_t sout_source0;
    uint8_t sout_source1;
    uint32_t usb_mux_index;
} adau1466_sample_rate_cfg_t;

static float normalize_pot10_ratio(uint16_t adc_val)
{
    if (adc_val <= POT_10BIT_MIN_DEADZONE)
    {
        return 0.0f;
    }
    if (adc_val >= POT_10BIT_DB_MAX_SNAP_START)
    {
        return 1.0f;
    }

    return (float) (adc_val - POT_10BIT_MIN_DEADZONE) /
           (float) (POT_10BIT_DB_MAX_SNAP_START - POT_10BIT_MIN_DEADZONE);
}

static float normalize_pot10_snap_ratio(uint16_t adc_val)
{
    if (adc_val <= POT_10BIT_MIN_DEADZONE)
    {
        return 0.0f;
    }
    if (adc_val >= POT_10BIT_DW_MAX_SNAP_START)
    {
        return 1.0f;
    }

    return (float) (adc_val - POT_10BIT_MIN_DEADZONE) /
           (float) (POT_10BIT_DW_MAX_SNAP_START - POT_10BIT_MIN_DEADZONE);
}

static bool adau1466_get_sample_rate_cfg(uint32_t hz, adau1466_sample_rate_cfg_t* cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    if (hz == 48000U)
    {
        cfg->clk_gen2_m   = 0x06U;
        cfg->sout_source0 = ADAU1466_SOUT0_FROM_ASRC2;
        cfg->sout_source1 = ADAU1466_SOUT1_FROM_ASRC3;
        cfg->usb_mux_index = ADAU1466_USB_MUX_ASRC;
        return true;
    }

    if (hz == 96000U)
    {
        cfg->clk_gen2_m   = 0x03U;
        cfg->sout_source0 = ADAU1466_SOUT_FROM_DSP;
        cfg->sout_source1 = ADAU1466_SOUT_FROM_DSP;
        cfg->usb_mux_index = ADAU1466_USB_MUX_DIRECT;
        return true;
    }

    return false;
}

static void adau1466_write_reg_u16(uint16_t addr, uint8_t value)
{
    uint8_t data[2] = {0x00, value};
    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, addr, 2, data);
}

static bool adau1466_wait_pll_lock(uint32_t timeout_ms)
{
    uint8_t pll_lock[2] = {0};
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        SIGMA_READ_REGISTER(DEVICE_ADDR_ADAU146XSCHEMATIC_1, ADAU1466_REG_PLL_LOCK, 2, pll_lock);
        if ((pll_lock[1] & 0x01U) != 0U)
        {
            return true;
        }
        osDelay(1);
    }

    SEGGER_RTT_printf(0, "[ADAU1466] PLL lock timeout\n");
    return false;
}

static uint32_t adau1466_q8_24_to_raw(double val)
{
    int64_t fixed_q8_24 = (int64_t) llround(val * 16777216.0);  // 2^24
    if (fixed_q8_24 > INT32_MAX)
    {
        fixed_q8_24 = INT32_MAX;
    }
    else if (fixed_q8_24 < INT32_MIN)
    {
        fixed_q8_24 = INT32_MIN;
    }

    return (uint32_t) ((int32_t) fixed_q8_24);
}

static void adau1466_store_be32(uint32_t raw, uint8_t out[4])
{
    out[0] = (uint8_t) ((raw >> 24) & 0xFFU);
    out[1] = (uint8_t) ((raw >> 16) & 0xFFU);
    out[2] = (uint8_t) ((raw >> 8) & 0xFFU);
    out[3] = (uint8_t) (raw & 0xFFU);
}

static void adau1466_delay_us(uint32_t delay_us)
{
    if (delay_us == 0U)
    {
        return;
    }

    if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) ||
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U))
    {
        HAL_Delay((delay_us + 999U) / 1000U);
        return;
    }

    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    if (cycles_per_us == 0U)
    {
        HAL_Delay(1);
        return;
    }

    uint32_t start_cycle = DWT->CYCCNT;
    uint32_t wait_cycles = cycles_per_us * delay_us;

    while ((DWT->CYCCNT - start_cycle) < wait_cycles)
    {
    }
}

static void adau1466_wait_safeload_frame(void)
{
    adau1466_delay_us((1000000U + ADAU1466_CORE_SAMPLE_RATE_HZ - 1U) /
                      ADAU1466_CORE_SAMPLE_RATE_HZ);
}

static bool adau1466_safeload_write_words(uint16_t addr, uint8_t mem_page, const uint8_t* data, uint8_t word_count)
{
    uint8_t safeload_ctrl[12] = {0x00};

    if ((data == NULL) || (word_count == 0U) || (word_count > 5U))
    {
        SEGGER_RTT_printf(0, "[ADAU1466] invalid safeload word_count: %u\n", word_count);
        return false;
    }

    if (mem_page > 1U)
    {
        SEGGER_RTT_printf(0, "[ADAU1466] invalid safeload mem_page: %u\n", mem_page);
        return false;
    }

    adau1466_store_be32(addr, &safeload_ctrl[0]);
    if (mem_page == 0U)
    {
        safeload_ctrl[7] = word_count;
    }
    else
    {
        safeload_ctrl[11] = word_count;
    }

    SIGMA_SAFELOAD_WRITE_DATA(
        DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SAFELOAD_DATA_SAFELOAD0_ADDR, (uint16_t) (word_count * 4U), (uint8_t*) data);
    SIGMA_SAFELOAD_WRITE_DATA(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SAFELOAD_ADDR_SAFELOAD_ADDR, sizeof(safeload_ctrl), safeload_ctrl);
    adau1466_wait_safeload_frame();

    return true;
}

double convert_pot2dB(uint16_t adc_val)
{
    double x  = (double) normalize_pot10_ratio(adc_val);
    double db = 0.0;
    if (x < 0.7)
    {
        db = -80.0 + (x / 0.7) * 80.0;
    }
    else
    {
        db = (x - 0.7) / 0.3 * 10.0;
    }
    return db;
}

int16_t convert_pot2dB_int(uint16_t adc_val)
{
    // Pot end-stop付近のADCノイズで表示/制御値が揺れないように端点デッドゾーンを設ける
    if (adc_val <= POT_10BIT_MIN_DEADZONE)
    {
        return -80;
    }
    if (adc_val >= POT_10BIT_DB_MAX_SNAP_START)
    {
        return 10;
    }

    double db    = convert_pot2dB(adc_val);
    int16_t db_i = (int16_t) ((db >= 0.0) ? (db + 0.5) : (db - 0.5));
    if (db_i < -80)
    {
        db_i = -80;
    }
    if (db_i > 10)
    {
        db_i = 10;
    }
    return db_i;
}

double convert_dB2gain(double db)
{
    return pow(10.0, db / 20.0);
}

static void write_q8_24(const uint16_t addr, const double val)
{
    uint8_t gain_array[4] = {0x00};
    uint32_t raw          = adau1466_q8_24_to_raw(val);

    adau1466_store_be32(raw, gain_array);

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, addr, 4, gain_array);
}

void safeload_write_q8_24(uint16_t addr, uint8_t mem_page, double val)
{
    uint8_t safeload_data[4] = {0x00};
    adau1466_store_be32(adau1466_q8_24_to_raw(val), safeload_data);
    (void) adau1466_safeload_write_words(addr, mem_page, safeload_data, 1U);
}

void AUDIO_Init_ADAU1466(uint32_t hz)
{
    // ADAU1466 HW Reset
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 0);
    osDelay(10);
    HAL_GPIO_WritePin(DSP_RESET_GPIO_Port, DSP_RESET_Pin, 1);
    osDelay(500);

#if RESET_FROM_FW
    // Program/configuration download is required only after reset. Runtime
    // sample-rate changes keep the 96 kHz DSP program and parameters intact.
    default_download_ADAU146XSCHEMATIC_1();
    osDelay(5);
#endif

    if (!AUDIO_Update_ADAU1466_SampleRate(hz))
    {
        SEGGER_RTT_printf(0, "[ADAU1466] initialization failed for %lu Hz\n", (unsigned long) hz);
    }
}

bool AUDIO_Update_ADAU1466_SampleRate(uint32_t hz)
{
    adau1466_sample_rate_cfg_t cfg;

    if (!adau1466_get_sample_rate_cfg(hz, &cfg))
    {
        SEGGER_RTT_printf(0, "[ADAU1466] unsupported sample rate: %lu\n", (unsigned long) hz);
        return false;
    }

    uint8_t mux_data[4] = {0U};
    uint8_t sout_data[4] = {0x00U, cfg.sout_source0, 0x00U, cfg.sout_source1};
    adau1466_store_be32(cfg.usb_mux_index, mux_data);

    if (hz == 48000U)
    {
        // Move both DSP paths to the ASRCs while the serial port still runs at
        // 96 kHz, then lower only the USB-side clock generator to 48 kHz.
        if (!adau1466_safeload_write_words(MOD_USB_RATE_SELECT_INDEX4_ADDR,
                                           MOD_USB_RATE_SELECT_INDEX4_MEM_PAGE,
                                           mux_data,
                                           1U))
        {
            return false;
        }
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1,
                                   ADAU1466_REG_SOUT_SOURCE0,
                                   sizeof(sout_data),
                                   sout_data);
        adau1466_write_reg_u16(ADAU1466_REG_CLK_GEN2_M, cfg.clk_gen2_m);
    }
    else
    {
        // Bring the USB-side serial port to 96 kHz before selecting the direct
        // paths. The DSP core and AK4619 clocks remain at 96 kHz throughout.
        adau1466_write_reg_u16(ADAU1466_REG_CLK_GEN2_M, cfg.clk_gen2_m);
        osDelay(ADAU1466_CLOCK_SETTLE_MS);
        if (!adau1466_safeload_write_words(MOD_USB_RATE_SELECT_INDEX4_ADDR,
                                           MOD_USB_RATE_SELECT_INDEX4_MEM_PAGE,
                                           mux_data,
                                           1U))
        {
            return false;
        }
        SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1,
                                   ADAU1466_REG_SOUT_SOURCE0,
                                   sizeof(sout_data),
                                   sout_data);
    }

    osDelay(ADAU1466_CLOCK_SETTLE_MS);

    if (!adau1466_wait_pll_lock(ADAU1466_PLL_LOCK_TIMEOUT_MS))
    {
        return false;
    }

    SEGGER_RTT_printf(0,
                      "[ADAU1466] USB path: %lu Hz, CLK_GEN2 M=%u, %s\n",
                      (unsigned long) hz,
                      cfg.clk_gen2_m,
                      (cfg.usb_mux_index == ADAU1466_USB_MUX_ASRC) ? "ASRC" : "direct");
    return true;
}

void set_dc_inputA(float ch_fader_position)
{
    write_q8_24(MOD_DCINPUT_A_DCVALUE_ADDR, ch_fader_position);
}

void set_dc_inputB(float ch_fader_position)
{
    write_q8_24(MOD_DCINPUT_B_DCVALUE_ADDR, ch_fader_position);
}

void control_input_from_usb_gain(uint8_t ch, int16_t db)
{
    SEGGER_RTT_printf(0, "USB CH%d Gain: %.2f dB\n", ch, db);

    const double gain = convert_dB2gain(db);

    switch (ch)
    {
    case 1:
        write_q8_24(MOD_INPUT_FROM_USB1_GAIN_ADDR, gain);
        break;
    case 2:
        write_q8_24(MOD_INPUT_FROM_USB2_GAIN_ADDR, gain);
        break;
    case 3:
        write_q8_24(MOD_INPUT_FROM_USB3_GAIN_ADDR, gain);
        break;
    case 4:
        write_q8_24(MOD_INPUT_FROM_USB4_GAIN_ADDR, gain);
        break;
    default:
        break;
    }
}

void control_input_from_ch1_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_CH1_GAIN_ADDR, gain);
}

void control_input_from_ch2_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_CH2_GAIN_ADDR, gain);
}

void control_input_from_return_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_INPUT_FROM_RETURN_GAIN_ADDR, gain);
}

void mute_input_from_return(void)
{
    write_q8_24(MOD_INPUT_FROM_RETURN_GAIN_ADDR, 0.0);
}

void control_send1_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_SEND1_OUTPUT_GAIN_ADDR, gain);
}

void control_send2_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_SEND2_OUTPUT_GAIN_ADDR, gain);
}

void control_dryA_out_gain(const uint16_t adc_val)
{
    const float rate = cos(pow(normalize_pot10_snap_ratio(adc_val), 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_DRYA_DCVALUE_ADDR, rate);
}

void control_dryB_out_gain(const uint16_t adc_val)
{
    const float rate = cos(pow(normalize_pot10_snap_ratio(adc_val), 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_DRYB_DCVALUE_ADDR, rate);
}

void control_wet_out_gain(const uint16_t adc_val)
{
    const float rate = sin(pow(normalize_pot10_snap_ratio(adc_val), 2.0f) * M_PI_2);
    write_q8_24(MOD_DCINPUT_WET_DCVALUE_ADDR, rate);
}

void control_ch1_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_CH1_OUTPUT_GAIN_ADDR, gain);
}

void control_ch2_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_CH2_OUTPUT_GAIN_ADDR, gain);
}

void control_hp_out_gain(const uint16_t adc_val)
{
    const double db   = (double) convert_pot2dB_int(adc_val);
    const double gain = convert_dB2gain(db);
    write_q8_24(MOD_HP_OUTPUT_GAIN_ADDR, gain);
}

void set_ch1_line()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_1_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void set_ch1_phono()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_1_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void set_ch2_line()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_2_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void set_ch2_phono()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(
        MOD_LN_PN_SW_2_INDEX_CHANNEL0_ADDR, MOD_LN_PN_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void select_input_type(uint8_t ch, uint8_t type)
{
    if (ch == INPUT_CH1)
    {
        switch (type)
        {
        case INPUT_TYPE_LINE:
            set_ch1_line();
            break;
        case INPUT_TYPE_PHONO:
            set_ch1_phono();
            break;
        default:
            break;
        }
    }
    else if (ch == INPUT_CH2)
    {
        switch (type)
        {
        case INPUT_TYPE_LINE:
            set_ch2_line();
            break;
        case INPUT_TYPE_PHONO:
            set_ch2_phono();
            break;
        default:
            break;
        }
    }
}

void disable_ch1_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_1_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void enable_ch1_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_1_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_1_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

static void select_send_ch1_src(bool select_dvs)
{
    ADI_REG_TYPE Mode0_0[4] = {0x00, 0x00, 0x00, select_dvs ? 0x01 : 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SEND_SW_1_INDEX_ADDR, 4, Mode0_0);
}

void disable_ch2_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x01, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x00, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_2_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

void enable_ch2_dvs()
{
    ADI_REG_TYPE Mode0_0[4]  = {0x00, 0x00, 0x00, 0x00};
    ADI_REG_TYPE Mode0_1[4]  = {0x01, 0x00, 0x00, 0x00};
    uint8_t safeload_data[8] = {0x00};

    memcpy(&safeload_data[0], Mode0_0, sizeof(Mode0_0));
    memcpy(&safeload_data[4], Mode0_1, sizeof(Mode0_1));
    (void) adau1466_safeload_write_words(MOD_DVS_SW_2_INDEX_CHANNEL0_ADDR, MOD_DVS_SW_2_INDEX_CHANNEL0_MEM_PAGE, safeload_data, 2U);
}

static void select_send_ch2_src(bool select_dvs)
{
    ADI_REG_TYPE Mode0_0[4] = {0x00, 0x00, 0x00, select_dvs ? 0x01 : 0x00};

    SIGMA_WRITE_REGISTER_BLOCK_IT(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_SEND_SW_2_INDEX_ADDR, 4, Mode0_0);
}

void select_send_source(uint8_t ch, bool select_insert)
{
    if (ch == INPUT_CH1)
    {
        select_send_ch1_src(select_insert);
    }
    else if (ch == INPUT_CH2)
    {
        select_send_ch2_src(select_insert);
    }
}

void set_input_insert_enabled(uint8_t ch, bool enabled)
{
    if (ch == INPUT_CH1)
    {
        if (enabled)
        {
            enable_ch1_dvs();
        }
        else
        {
            disable_ch1_dvs();
        }
    }
    else if (ch == INPUT_CH2)
    {
        if (enabled)
        {
            enable_ch2_dvs();
        }
        else
        {
            disable_ch2_dvs();
        }
    }
}

void select_ch_fader_assign_a_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB12:
        Mode0[3] = 0x02;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x03;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_CH_FADER_ASSIGN_SW_A_INDEX_ADDR, 4, Mode0);
}

void select_ch_fader_assign_b_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB12:
        Mode0[3] = 0x02;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x03;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_CH_FADER_ASSIGN_SW_B_INDEX_ADDR, 4, Mode0);
}

void select_ch_fader_assign_post_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_CH1:
        Mode0[3] = 0x00;
        break;
    case INPUT_CH2:
        Mode0[3] = 0x01;
        break;
    case INPUT_USB12:
        Mode0[3] = 0x02;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x03;
        break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_CH_FADER_ASSIGN_SW_POST_INDEX_ADDR, 4, Mode0);
}

void select_return_ch_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case INPUT_USB12:
        Mode0[3] = 0x00;
        break;
    case INPUT_USB34:
        Mode0[3] = 0x01;
        break;
    default:
        return;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_RETURN_CH_SW_INDEX_ADDR, 4, Mode0);
}

void select_hp_out_source(uint8_t ch)
{
    ADI_REG_TYPE Mode0[4] = {0x00, 0x00, 0x00, 0x00};

    switch (ch)
    {
    case CUE_SEL_CH_FADER_A:
        Mode0[3] = 0x00;
        break;
    case CUE_SEL_CH_FADER_B:
        Mode0[3] = 0x01;
        break;
    case CUE_SEL_THRU:
        Mode0[3] = 0x02;
        break;
    case CUE_SEL_MST:
        Mode0[3] = 0x03;
        break;
    default:
    	break;
    }

    SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_ADAU146XSCHEMATIC_1, MOD_HP_OUT_SW_INDEX_ADDR, 4, Mode0);
}
