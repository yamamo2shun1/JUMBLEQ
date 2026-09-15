/*
 * ui_control.c
 *
 *  Created on: Feb 18, 2026
 */

#include "ui_control.h"
#include "ui_control_internal.h"
#include "eeprom_config_internal.h"
#include "ui_adc_control_internal.h"
#include "ui_ch_fader_internal.h"
#include "ui_midi_control_internal.h"
#include "ui_pot_control_internal.h"
#include "ui_routing_control_internal.h"
#include "ui_uf2_control_internal.h"

#include "audio_control.h"
#include "timecode_synth.h"

#include "eeprom.h"
#include "i2c.h"
#include "led_control.h"

#include "adau1466.h"

#include "FreeRTOS.h"
#include "task.h"

// Aggregated UI runtime state (ADC-derived controls + persisted selections).
typedef struct
{
    bool mag_out_as_note;
    bool curve_edit_mode;
    bool is_start_audio_control;
} ui_control_state_t;

static ui_control_state_t s_ui = {
    .mag_out_as_note        = false,
    .curve_edit_mode        = false,
    .is_start_audio_control = false,
};

static const uint8_t MIDI_CH_15                  = 14U;  // zero-based MIDI channel index.
static const uint8_t MIDI_CC_CH_FADER_CURVE_A        = 20U;
static const uint8_t MIDI_CC_CH_FADER_CURVE_B        = 21U;
static const uint8_t MIDI_CC_CH_FADER_DVS_DELAY      = 22U;
static const uint8_t MIDI_PC_CURVE_EDIT_MODE_OFF = 120U;
static const uint8_t MIDI_PC_CURVE_EDIT_MODE_ON  = 121U;
static const uint8_t MIDI_PC_MUX_OUTPUT_CC       = 122U;
static const uint8_t MIDI_PC_MUX_OUTPUT_NOTE     = 123U;
static const uint8_t MIDI_PC_ARM_UF2_BOOTLOADER  = 124U;
static const uint8_t MIDI_PC_CANCEL_UF2_BOOTLOADER = 125U;
static const uint8_t MIDI_PC_REQUEST_EEPROM_DUMP = 126U;
static const uint8_t MIDI_PC_SAVE_EEPROM         = 127U;
bool ui_control_is_curve_edit_mode_enabled(void)
{
    return s_ui.curve_edit_mode;
}

static uint8_t midi_program_for_input_type(uint8_t input_ch, uint8_t input_type)
{
    if (input_ch == INPUT_CH1)
    {
        return (input_type == INPUT_TYPE_PHONO) ? CH1_PHONO : CH1_LINE;
    }

    if (input_ch == INPUT_CH2)
    {
        return (input_type == INPUT_TYPE_PHONO) ? CH2_PHONO : CH2_LINE;
    }

    return CH1_LINE;
}

static uint8_t midi_program_for_ch_fader_assign_a(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return CH_FADER_ASSIGN_A_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return CH_FADER_ASSIGN_A_CH2;
    case INPUT_SRC_USB12:
        return CH_FADER_ASSIGN_A_USB12;
    case INPUT_SRC_USB34:
        return CH_FADER_ASSIGN_A_USB34;
    default:
        return CH_FADER_ASSIGN_A_CH1;
    }
}

static uint8_t midi_program_for_ch_fader_assign_b(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return CH_FADER_ASSIGN_B_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return CH_FADER_ASSIGN_B_CH2;
    case INPUT_SRC_USB12:
        return CH_FADER_ASSIGN_B_USB12;
    case INPUT_SRC_USB34:
        return CH_FADER_ASSIGN_B_USB34;
    default:
        return CH_FADER_ASSIGN_B_CH1;
    }
}

static uint8_t midi_program_for_ch_fader_assign_post(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return CH_FADER_ASSIGN_POST_CH1;
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return CH_FADER_ASSIGN_POST_CH2;
    case INPUT_SRC_USB12:
        return CH_FADER_ASSIGN_POST_USB12;
    case INPUT_SRC_USB34:
        return CH_FADER_ASSIGN_POST_USB34;
    default:
        return CH_FADER_ASSIGN_POST_CH1;
    }
}

static uint8_t midi_program_for_input_mode(uint8_t input_ch, uint8_t mode)
{
    if (input_ch == INPUT_CH1)
    {
        if (mode == UI_INPUT_MODE_DVS)
        {
            return CH1_MODE_DVS;
        }
        if (mode == UI_INPUT_MODE_SYNTH)
        {
            return CH1_MODE_SYNTH;
        }
        return CH1_MODE_DISABLE;
    }

    if (input_ch == INPUT_CH2)
    {
        if (mode == UI_INPUT_MODE_DVS)
        {
            return CH2_MODE_DVS;
        }
        if (mode == UI_INPUT_MODE_SYNTH)
        {
            return CH2_MODE_SYNTH;
        }
        return CH2_MODE_DISABLE;
    }

    return CH1_MODE_DISABLE;
}

static uint8_t midi_program_for_return_assign(uint8_t assign)
{
    switch (assign)
    {
    case INPUT_SRC_USB12:
        return RETURN_CH_USB12;
    case INPUT_SRC_USB34:
        return RETURN_CH_USB34;
    case INPUT_SRC_NONE:
        return RETURN_CH_NONE;
    default:
        return RETURN_CH_USB34;
    }
}

static uint8_t midi_program_for_hp_out_source(uint8_t source)
{
    switch (source)
    {
    case CUE_SEL_CH_FADER_A:
        return HP_OUT_CH_FADER_A;
    case CUE_SEL_CH_FADER_B:
        return HP_OUT_CH_FADER_B;
    case CUE_SEL_THRU:
        return HP_OUT_THRU;
    case CUE_SEL_MST:
        return HP_OUT_MASTER;
    default:
        return HP_OUT_MASTER;
    }
}

static uint8_t midi_program_for_ch_fader_aux_assignment(uint8_t sensor_idx, uint8_t assign)
{
    if (sensor_idx == 2U)
    {
        return (assign == UI_CH_FADER_AUX_ASSIGN_A) ? CH_FADER_AUX_SENSOR2_TO_A : CH_FADER_AUX_SENSOR2_TO_B;
    }

    return (assign == UI_CH_FADER_AUX_ASSIGN_A) ? CH_FADER_AUX_SENSOR3_TO_A : CH_FADER_AUX_SENSOR3_TO_B;
}

static uint8_t midi_program_for_timecode_synth_ratio_set(uint8_t ratio_set)
{
    if (ratio_set == TIMECODE_RATIO_HARMONIC)
    {
        return SYNTH_RATIO_HARMONIC;
    }
    if (ratio_set == TIMECODE_RATIO_CHORD)
    {
        return SYNTH_RATIO_CHORD;
    }
    return SYNTH_RATIO_OCTAVE;
}

static uint8_t midi_program_for_timecode_synth_warp_algorithm(uint8_t warp_algorithm)
{
    switch (warp_algorithm)
    {
    case TIMECODE_WARP_CLEAN:
        return SYNTH_WARP_CLEAN;
    case TIMECODE_WARP_RING_MOD:
        return SYNTH_WARP_RING_MOD;
    case TIMECODE_WARP_COMPARATOR:
        return SYNTH_WARP_COMPARATOR;
    case TIMECODE_WARP_CROSSFOLD:
    default:
        return SYNTH_WARP_CROSSFOLD;
    }
}

static void send_midi_config_dump(const EEPROM_DeviceConfig_t* cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    ui_midi_control_send_program(midi_program_for_input_type(INPUT_CH1, cfg->current_ch1_input_type), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_input_type(INPUT_CH2, cfg->current_ch2_input_type), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_ch_fader_assign_a(cfg->current_ch_fader_a_assign), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_ch_fader_assign_b(cfg->current_ch_fader_b_assign), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_ch_fader_assign_post(cfg->current_ch_fader_post_assign), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_return_assign(cfg->current_return_assign), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_hp_out_source(cfg->current_hp_out_source), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_input_mode(INPUT_CH1, cfg->current_ch1_input_mode), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_input_mode(INPUT_CH2, cfg->current_ch2_input_mode), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_ch_fader_aux_assignment(2U, cfg->sensor2_aux_fade_down_assign), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_ch_fader_aux_assignment(3U, cfg->sensor3_aux_fade_down_assign), MIDI_CH_15);
    ui_midi_control_send_program(eeprom_config_is_ch_fader_reverse_a_enabled(cfg) ? CH_FADER_REVERSE_A_ON : CH_FADER_REVERSE_A_OFF, MIDI_CH_15);
    ui_midi_control_send_program(eeprom_config_is_ch_fader_reverse_b_enabled(cfg) ? CH_FADER_REVERSE_B_ON : CH_FADER_REVERSE_B_OFF, MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_timecode_synth_ratio_set(cfg->timecode_synth_ratio_set), MIDI_CH_15);
    ui_midi_control_send_program(midi_program_for_timecode_synth_warp_algorithm(cfg->timecode_synth_warp_algorithm), MIDI_CH_15);
    ui_midi_control_send_cc(MIDI_CC_CH_FADER_CURVE_A, ui_ch_fader_curve_width_to_midi_cc(cfg->current_ch_fader_curve_width_a), MIDI_CH_15);
    ui_midi_control_send_cc(MIDI_CC_CH_FADER_CURVE_B, ui_ch_fader_curve_width_to_midi_cc(cfg->current_ch_fader_curve_width_b), MIDI_CH_15);
    ui_midi_control_send_cc(MIDI_CC_CH_FADER_DVS_DELAY, cfg->ch_fader_dvs_delay_ms, MIDI_CH_15);
    if (eeprom_config_is_mag_out_as_note(cfg))
    {
        ui_midi_control_send_program(MIDI_PC_MUX_OUTPUT_NOTE, MIDI_CH_15);
    }
    else
    {
        ui_midi_control_send_program(MIDI_PC_MUX_OUTPUT_CC, MIDI_CH_15);
    }
}

void ui_control_reapply_pot_outputs(void)
{
    ui_pot_control_reapply_outputs(ui_routing_is_synth_mode_active(),
                                   s_ui.mag_out_as_note,
                                   ui_routing_get_return_assign());
}

void ui_control_reapply_ch_fader_outputs(void)
{
    ui_ch_fader_reapply_outputs();
}

typedef void (*midi_program_handler_t)(uint8_t arg);

typedef struct
{
    uint8_t command;
    midi_program_handler_t handler;
    uint8_t arg;
} midi_program_cmd_t;

static void midi_program_set_input_type(uint8_t arg)
{
    uint8_t input_ch   = (arg >> 4) & 0x0F;
    uint8_t input_type = arg & 0x0F;
    ui_routing_apply_input_type(input_ch, input_type);
}

static void midi_program_apply_ch_fader_a(uint8_t input_ch)
{
    ui_routing_apply_ch_fader_assign_a(input_ch);
}

static void midi_program_apply_ch_fader_b(uint8_t input_ch)
{
    ui_routing_apply_ch_fader_assign_b(input_ch);
}

static void midi_program_apply_ch_fader_post(uint8_t input_ch)
{
    ui_routing_apply_ch_fader_assign_post(input_ch);
}

static void midi_program_apply_return(uint8_t input_ch)
{
    const uint8_t return_assign = ui_routing_apply_return_source(input_ch);

    ui_pot_control_apply_return_outputs(return_assign);
}

static void midi_program_apply_hp_out(uint8_t source)
{
    ui_routing_apply_hp_out_source(source);
}

static void midi_program_apply_input_mode(uint8_t arg)
{
    const uint8_t input_ch = (arg >> 4) & 0x0FU;
    const uint8_t mode     = arg & 0x0FU;

    if (mode <= UI_INPUT_MODE_SYNTH)
    {
        ui_routing_apply_input_mode(input_ch, (UI_InputMode_t) mode);
        ui_control_reapply_ch_fader_outputs();
    }
}

static void midi_program_apply_ch_fader_aux_assignment(uint8_t arg)
{
    const uint8_t sensor_idx = (arg >> 4) & 0x0FU;
    const uint8_t assign     = arg & 0x0FU;

    if (sensor_idx == 2U)
    {
        (void) ui_ch_fader_apply_aux_assignments(assign, ui_ch_fader_get_aux_assign(3U));
    }
    else if (sensor_idx == 3U)
    {
        (void) ui_ch_fader_apply_aux_assignments(ui_ch_fader_get_aux_assign(2U), assign);
    }

    SEGGER_RTT_printf(0,
                      "Channel fader aux assignment: sensor2=%c sensor3=%c\r\n",
                      (ui_ch_fader_get_aux_assign(2U) == UI_CH_FADER_AUX_ASSIGN_A) ? 'A' : 'B',
                      (ui_ch_fader_get_aux_assign(3U) == UI_CH_FADER_AUX_ASSIGN_A) ? 'A' : 'B');
}

static void midi_program_apply_timecode_synth_ratio_set(uint8_t arg)
{
    timecode_synth_set_ratio_set((TimecodeOscillatorRatioSet_t) arg);
    SEGGER_RTT_printf(0, "SYNTH ratio set: %u\r\n", (unsigned) arg);
}

static void midi_program_apply_timecode_synth_warp_algorithm(uint8_t arg)
{
    timecode_synth_set_warp_algorithm((TimecodeOscillatorWarpAlgorithm_t) arg);
    SEGGER_RTT_printf(0, "SYNTH warp algorithm: %u\r\n", (unsigned) arg);
}

static bool dispatch_midi_program_change(uint8_t channel, uint8_t program)
{
    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (program == MIDI_PC_CURVE_EDIT_MODE_OFF)
    {
        s_ui.curve_edit_mode = false;
        SEGGER_RTT_printf(0, "Curve edit mode OFF (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_CURVE_EDIT_MODE_ON)
    {
        s_ui.curve_edit_mode = true;
        SEGGER_RTT_printf(0, "Curve edit mode ON (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_MUX_OUTPUT_CC)
    {
        s_ui.mag_out_as_note = false;
        SEGGER_RTT_printf(0, "Mag/pot_mag output mode: CC (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_MUX_OUTPUT_NOTE)
    {
        s_ui.mag_out_as_note = true;
        SEGGER_RTT_printf(0, "Mag/pot_mag output mode: Note (PC%u)\r\n", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_ARM_UF2_BOOTLOADER)
    {
        ui_uf2_control_arm(MIDI_PC_ARM_UF2_BOOTLOADER);
        return true;
    }

    if (program == MIDI_PC_CANCEL_UF2_BOOTLOADER)
    {
        ui_uf2_control_cancel("cancelled by PC125 Ch15");
        return true;
    }

    if ((program == CH_FADER_REVERSE_A_OFF) || (program == CH_FADER_REVERSE_A_ON))
    {
        ui_ch_fader_set_reverse(0U, (program == CH_FADER_REVERSE_A_ON));
        ui_control_reapply_ch_fader_outputs();
        SEGGER_RTT_printf(0, "Channel fader A Reverse: %s (PC%u)\r\n", ui_control_is_ch_fader_reverse_a_enabled() ? "ON" : "OFF", (unsigned) program);
        return true;
    }

    if ((program == CH_FADER_REVERSE_B_OFF) || (program == CH_FADER_REVERSE_B_ON))
    {
        ui_ch_fader_set_reverse(1U, (program == CH_FADER_REVERSE_B_ON));
        ui_control_reapply_ch_fader_outputs();
        SEGGER_RTT_printf(0, "Channel fader B Reverse: %s (PC%u)\r\n", ui_control_is_ch_fader_reverse_b_enabled() ? "ON" : "OFF", (unsigned) program);
        return true;
    }

    if (program == MIDI_PC_REQUEST_EEPROM_DUMP)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        send_midi_config_dump(&cfg);
        SEGGER_RTT_printf(0, "Current config dumped by MIDI PC126: CH1=%u CH2=%u CH_FADER_A=%u CH_FADER_B=%u CH_FADER_POST=%u RTN=%u HP=%u MODE1=%u MODE2=%u MAG_AS_NOTE=%u AUX2=%u AUX3=%u REVERSE_A=%u REVERSE_B=%u CURVE_WIDTH_A=%.4f CURVE_WIDTH_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_ch_fader_a_assign, (unsigned) cfg.current_ch_fader_b_assign, (unsigned) cfg.current_ch_fader_post_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_input_mode, (unsigned) cfg.current_ch2_input_mode, (unsigned) eeprom_config_is_mag_out_as_note(&cfg), (unsigned) cfg.sensor2_aux_fade_down_assign, (unsigned) cfg.sensor3_aux_fade_down_assign, (unsigned) eeprom_config_is_ch_fader_reverse_a_enabled(&cfg), (unsigned) eeprom_config_is_ch_fader_reverse_b_enabled(&cfg), (double) cfg.current_ch_fader_curve_width_a, (double) cfg.current_ch_fader_curve_width_b);

        return true;
    }

    if (program == MIDI_PC_SAVE_EEPROM)
    {
        EEPROM_DeviceConfig_t cfg;

        EEPROM_ConfigCaptureCurrent(&cfg);
        if (EEPROM_SaveConfig(&hi2c2, &cfg) == HAL_OK)
        {
            led_notify_save_success();
            SEGGER_RTT_printf(0, "EEPROM config saved by MIDI PC127: CH1=%u CH2=%u CH_FADER_A=%u CH_FADER_B=%u CH_FADER_POST=%u RTN=%u HP=%u MODE1=%u MODE2=%u AUX2=%u AUX3=%u REVERSE_A=%u REVERSE_B=%u CURVE_WIDTH_A=%.4f CURVE_WIDTH_B=%.4f\r\n", (unsigned) cfg.current_ch1_input_type, (unsigned) cfg.current_ch2_input_type, (unsigned) cfg.current_ch_fader_a_assign, (unsigned) cfg.current_ch_fader_b_assign, (unsigned) cfg.current_ch_fader_post_assign, (unsigned) cfg.current_return_assign, (unsigned) cfg.current_hp_out_source, (unsigned) cfg.current_ch1_input_mode, (unsigned) cfg.current_ch2_input_mode, (unsigned) cfg.sensor2_aux_fade_down_assign, (unsigned) cfg.sensor3_aux_fade_down_assign, (unsigned) eeprom_config_is_ch_fader_reverse_a_enabled(&cfg), (unsigned) eeprom_config_is_ch_fader_reverse_b_enabled(&cfg), (double) cfg.current_ch_fader_curve_width_a, (double) cfg.current_ch_fader_curve_width_b);
        }
        else
        {
            SEGGER_RTT_printf(0, "EEPROM config save failed by MIDI PC127\r\n");
        }
        return true;
    }

    static const midi_program_cmd_t commands[] = {
        {CH1_LINE,             midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_LINE) },
        {CH1_PHONO,            midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_PHONO)},
        {CH2_LINE,             midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_LINE) },
        {CH2_PHONO,            midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_PHONO)},
        {CH_FADER_ASSIGN_A_CH1,      midi_program_apply_ch_fader_a,     INPUT_CH1                                      },
        {CH_FADER_ASSIGN_A_CH2,      midi_program_apply_ch_fader_a,     INPUT_CH2                                      },
        {CH_FADER_ASSIGN_A_USB12,    midi_program_apply_ch_fader_a,     INPUT_USB12                                    },
        {CH_FADER_ASSIGN_A_USB34,    midi_program_apply_ch_fader_a,     INPUT_USB34                                    },
        {CH_FADER_ASSIGN_B_CH1,      midi_program_apply_ch_fader_b,     INPUT_CH1                                      },
        {CH_FADER_ASSIGN_B_CH2,      midi_program_apply_ch_fader_b,     INPUT_CH2                                      },
        {CH_FADER_ASSIGN_B_USB12,    midi_program_apply_ch_fader_b,     INPUT_USB12                                    },
        {CH_FADER_ASSIGN_B_USB34,    midi_program_apply_ch_fader_b,     INPUT_USB34                                    },
        {CH_FADER_ASSIGN_POST_CH1,   midi_program_apply_ch_fader_post,  INPUT_CH1                                      },
        {CH_FADER_ASSIGN_POST_CH2,   midi_program_apply_ch_fader_post,  INPUT_CH2                                      },
        {CH_FADER_ASSIGN_POST_USB12, midi_program_apply_ch_fader_post,  INPUT_USB12                                    },
        {CH_FADER_ASSIGN_POST_USB34, midi_program_apply_ch_fader_post,  INPUT_USB34                                    },
        {CH1_MODE_DISABLE,     midi_program_apply_input_mode, (uint8_t) ((INPUT_CH1 << 4) | UI_INPUT_MODE_DISABLED)},
        {CH1_MODE_DVS,         midi_program_apply_input_mode, (uint8_t) ((INPUT_CH1 << 4) | UI_INPUT_MODE_DVS)     },
        {CH1_MODE_SYNTH,       midi_program_apply_input_mode, (uint8_t) ((INPUT_CH1 << 4) | UI_INPUT_MODE_SYNTH)   },
        {CH2_MODE_DISABLE,     midi_program_apply_input_mode, (uint8_t) ((INPUT_CH2 << 4) | UI_INPUT_MODE_DISABLED)},
        {CH2_MODE_DVS,         midi_program_apply_input_mode, (uint8_t) ((INPUT_CH2 << 4) | UI_INPUT_MODE_DVS)     },
        {CH2_MODE_SYNTH,       midi_program_apply_input_mode, (uint8_t) ((INPUT_CH2 << 4) | UI_INPUT_MODE_SYNTH)   },
        {RETURN_CH_USB12,      midi_program_apply_return,   INPUT_USB12                                    },
        {RETURN_CH_USB34,      midi_program_apply_return,   INPUT_USB34                                    },
        {RETURN_CH_NONE,       midi_program_apply_return,   INPUT_SRC_NONE                                 },
        {HP_OUT_CH_FADER_A,          midi_program_apply_hp_out,   CUE_SEL_CH_FADER_A                                   },
        {HP_OUT_CH_FADER_B,          midi_program_apply_hp_out,   CUE_SEL_CH_FADER_B                                   },
        {HP_OUT_THRU,          midi_program_apply_hp_out,   CUE_SEL_THRU                                   },
        {HP_OUT_MASTER,        midi_program_apply_hp_out,   CUE_SEL_MST                                    },
        {CH_FADER_AUX_SENSOR2_TO_A,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((2U << 4) | UI_CH_FADER_AUX_ASSIGN_A)},
        {CH_FADER_AUX_SENSOR2_TO_B,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((2U << 4) | UI_CH_FADER_AUX_ASSIGN_B)},
        {CH_FADER_AUX_SENSOR3_TO_A,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((3U << 4) | UI_CH_FADER_AUX_ASSIGN_A)},
        {CH_FADER_AUX_SENSOR3_TO_B,  midi_program_apply_ch_fader_aux_assignment, (uint8_t) ((3U << 4) | UI_CH_FADER_AUX_ASSIGN_B)},
        {SYNTH_RATIO_OCTAVE,         midi_program_apply_timecode_synth_ratio_set, TIMECODE_RATIO_OCTAVE                  },
        {SYNTH_RATIO_HARMONIC,       midi_program_apply_timecode_synth_ratio_set, TIMECODE_RATIO_HARMONIC                },
        {SYNTH_RATIO_CHORD,          midi_program_apply_timecode_synth_ratio_set, TIMECODE_RATIO_CHORD                   },
        {SYNTH_WARP_CLEAN,           midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_CLEAN                },
        {SYNTH_WARP_CROSSFOLD,       midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_CROSSFOLD            },
        {SYNTH_WARP_RING_MOD,        midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_RING_MOD             },
        {SYNTH_WARP_COMPARATOR,      midi_program_apply_timecode_synth_warp_algorithm, TIMECODE_WARP_COMPARATOR           },
    };

    for (uint32_t i = 0; i < TU_ARRAY_SIZE(commands); i++)
    {
        if (commands[i].command == program)
        {
            commands[i].handler(commands[i].arg);
            return true;
        }
    }
    return false;
}

static bool dispatch_midi_control_change(uint8_t channel, uint8_t number, uint8_t value)
{
    if (!s_ui.curve_edit_mode)
    {
        return false;
    }

    if (channel != MIDI_CH_15)
    {
        return false;
    }

    if (number == MIDI_CC_CH_FADER_CURVE_A)
    {
        if (ui_ch_fader_set_curve_width_from_cc(0U, value))
        {
            ui_ch_fader_mark_curve_dirty();
            SEGGER_RTT_printf(0, "Curve A updated by CC%u Ch15 -> width %.4f\r\n", (unsigned) number, (double) ui_ch_fader_get_curve_width(0U));
        }
        return true;
    }

    if (number == MIDI_CC_CH_FADER_CURVE_B)
    {
        if (ui_ch_fader_set_curve_width_from_cc(1U, value))
        {
            ui_ch_fader_mark_curve_dirty();
            SEGGER_RTT_printf(0, "Curve B updated by CC%u Ch15 -> width %.4f\r\n", (unsigned) number, (double) ui_ch_fader_get_curve_width(1U));
        }
        return true;
    }

    if (number == MIDI_CC_CH_FADER_DVS_DELAY)
    {
        ui_ch_fader_apply_dvs_delay(value);
        SEGGER_RTT_printf(0,
                          "DVS fader delay updated by CC%u Ch15 -> %u ms\r\n",
                          (unsigned) number,
                          (unsigned) ui_control_get_ch_fader_dvs_delay_ms());
        return true;
    }

    return false;
}

static void process_midi_rx(void)
{
    UiMidiEvent_t event;

    while (ui_midi_control_read(&event))
    {
        if (event.status == 0xC0U)
        {
            (void) dispatch_midi_program_change(event.channel, event.data1);
        }
        else if (event.status == 0xB0U)
        {
            (void) dispatch_midi_control_change(event.channel, event.data1, event.data2);
        }

        SEGGER_RTT_printf(0, "MIDI RX: 0x%02X 0x%02X 0x%02X(%d) 0x%02X(%d)\n",
                          event.raw_cin,
                          (uint8_t) (event.status | event.channel),
                          event.data1, event.data1,
                          event.data2, event.data2);
    }
}

void ui_control_task(void)
{
#if !ENABLE_DSP_RUNTIME_CONTROL
    return;
#endif

    if (!is_started_audio_control())
    {
        return;
    }

    if (ui_adc_control_is_complete())
    {
        const uint32_t* adc_samples = ui_adc_control_samples();

        ui_pot_control_process(adc_samples,
                               ui_routing_is_synth_mode_active(),
                               s_ui.mag_out_as_note,
                               ui_routing_get_return_assign());
        ui_ch_fader_process(adc_samples, s_ui.mag_out_as_note);
        process_midi_rx();
        ui_adc_control_clear_complete();
    }

    ui_uf2_control_service();

    // Keep due writes running even when no new ADC sample has arrived. Handle
    // MIDI routing changes first so queued values cannot replay on a new input.
    ui_ch_fader_service_dsp_outputs();
}

void start_audio_control(void)
{
    s_ui.is_start_audio_control = true;
    __DMB();
}

bool is_started_audio_control(void)
{
    return s_ui.is_start_audio_control;
}

// OLED Task専用。owner状態の軽量copyだけをscheduler停止区間で行い、
// dB/CC変換とsnapshot組立てはtask切替再開後に行う。
bool ui_control_get_display_snapshot(UI_DisplaySnapshot_t* snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }

    UI_PotDisplayState_t pot_state;
    UI_RoutingDisplayState_t routing_state;
    UI_ChFaderDisplayState_t ch_fader_state;
    UI_Uf2DisplayState_t uf2_state;
    bool curve_edit_mode;
    uint32_t sample_rate_hz;

    vTaskSuspendAll();
    ui_pot_control_capture_display_state(&pot_state);
    ui_routing_capture_display_state(&routing_state);
    ui_ch_fader_capture_display_state(&ch_fader_state);
    ui_uf2_control_capture_display_state(&uf2_state);
    curve_edit_mode = s_ui.curve_edit_mode;
    sample_rate_hz  = get_current_sample_rate_hz();
    (void) xTaskResumeAll();

    UI_DisplaySnapshot_t local;
    local.curve_edit_mode        = curve_edit_mode;
    local.uf2_transition_state   = uf2_state.state;
    local.uf2_seconds_remaining  = uf2_state.seconds_remaining;

    local.sample_rate_hz = sample_rate_hz;
    local.ch1_input_db   = convert_pot2dB_int(pot_state.ch1_input);
    local.ch2_input_db   = convert_pot2dB_int(pot_state.ch2_input);
    local.ch1_output_db  = convert_pot2dB_int(pot_state.ch1_output);
    local.ch2_output_db  = convert_pot2dB_int(pot_state.ch2_output);
    local.return_db      = convert_pot2dB_int(pot_state.return_input);
    local.hp_output_db   = convert_pot2dB_int(pot_state.hp_output);
    local.return_enabled = routing_state.return_enabled;

    local.ch_fader_curve_a_cc   = ui_ch_fader_curve_width_to_midi_cc(ch_fader_state.curve_width_a);
    local.ch_fader_curve_b_cc   = ui_ch_fader_curve_width_to_midi_cc(ch_fader_state.curve_width_b);
    local.ch_fader_dvs_delay_ms = ch_fader_state.dvs_delay_ms;
    local.dvs_enabled           = routing_state.dvs_enabled;
    local.ch_fader_reverse_a    = ch_fader_state.reverse_a;
    local.ch_fader_reverse_b    = ch_fader_state.reverse_b;

    local.input_source_a_text = routing_state.input_source_a_text;
    local.input_source_b_text = routing_state.input_source_b_text;
    local.input_type_a_text   = routing_state.input_type_a_text;
    local.input_type_b_text   = routing_state.input_type_b_text;
    local.thru_source_text    = routing_state.thru_source_text;
    local.return_source_text  = routing_state.return_source_text;
    local.hp_source_text      = routing_state.hp_source_text;

    local.input_source_a_mode_visible = routing_state.input_source_a_mode_visible;
    local.input_source_a_mode         = routing_state.input_source_a_mode;
    local.input_source_b_mode_visible = routing_state.input_source_b_mode_visible;
    local.input_source_b_mode         = routing_state.input_source_b_mode;

    *snapshot = local;
    return true;
}

void ui_control_get_persist_state(UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    ui_routing_capture_persist(state);
    ui_ch_fader_capture_persist(state);
    state->mag_out_as_note           = s_ui.mag_out_as_note;
}

bool ui_control_validate_persist_state(const UI_ControlPersistState_t* state)
{
    if (state == NULL)
    {
        return false;
    }

    if (!ui_routing_validate_persist(state) ||
        !ui_ch_fader_validate_persist(state))
    {
        return false;
    }

    return true;
}

bool ui_control_apply_persist_state(const UI_ControlPersistState_t* state)
{
    uint8_t input_ch_a;
    uint8_t input_ch_b;
    uint8_t input_ch_post;
    uint8_t input_ch_return;

    if (!ui_control_validate_persist_state(state))
    {
        return false;
    }

    if (!ui_routing_assign_to_input_ch(state->current_ch_fader_a_assign, &input_ch_a) ||
        !ui_routing_assign_to_input_ch(state->current_ch_fader_b_assign, &input_ch_b) ||
        !ui_routing_assign_to_input_ch(state->current_ch_fader_post_assign, &input_ch_post) ||
        !ui_routing_assign_to_return_input_ch(state->current_return_assign, &input_ch_return))
    {
        return false;
    }

    ui_routing_apply_input_type(INPUT_CH1, state->current_ch1_input_type);
    ui_routing_apply_input_type(INPUT_CH2, state->current_ch2_input_type);
    ui_routing_apply_ch_fader_assign_a(input_ch_a);
    ui_routing_apply_ch_fader_assign_b(input_ch_b);
    ui_routing_apply_ch_fader_assign_post(input_ch_post);
    ui_pot_control_apply_return_outputs(ui_routing_apply_return_source(input_ch_return));
    ui_routing_apply_hp_out_source(state->current_hp_out_source);
    ui_routing_apply_input_mode(INPUT_CH1, (UI_InputMode_t) state->current_ch1_input_mode);
    ui_control_reapply_ch_fader_outputs();
    ui_routing_apply_input_mode(INPUT_CH2, (UI_InputMode_t) state->current_ch2_input_mode);
    ui_control_reapply_ch_fader_outputs();
    ui_ch_fader_apply_dvs_delay(state->ch_fader_dvs_delay_ms);
    (void) ui_ch_fader_apply_aux_assignments(state->sensor2_aux_fade_down_assign,
                                             state->sensor3_aux_fade_down_assign);
    ui_ch_fader_set_curve_width(0U, state->current_ch_fader_curve_width_a);
    ui_ch_fader_set_curve_width(1U, state->current_ch_fader_curve_width_b);
    ui_ch_fader_set_reverse(0U, state->ch_fader_reverse_a);
    ui_ch_fader_set_reverse(1U, state->ch_fader_reverse_b);
    s_ui.mag_out_as_note    = state->mag_out_as_note;
    ui_ch_fader_mark_curve_dirty();

    return true;
}

void ui_control_reset_state(void)
{
    ui_ch_fader_reset();
    ui_uf2_control_reset();

    ui_adc_control_reset();
    ui_pot_control_reset();

    ui_routing_reset();
    s_ui.curve_edit_mode        = false;
}
