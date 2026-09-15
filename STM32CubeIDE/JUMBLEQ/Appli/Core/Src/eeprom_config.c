/*
 * eeprom_config.c
 *
 * EEPROM device configuration: record layout, CRC32, version migration,
 * defaults, runtime capture, semantic validation and runtime apply.
 *
 * The low-level I2C access stays in eeprom.c; this module owns everything
 * that knows about the product settings layout.
 */

#include "eeprom.h"
#include "eeprom_config_internal.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

#include "adau1466.h"
#include "timecode_synth.h"
#include "ui_control.h"
#include "ui_control_internal.h"

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
} EEPROM_ConfigHeader_t;

typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_ch_fader_a_assign;
    uint8_t current_ch_fader_b_assign;
    uint8_t current_ch_fader_post_assign;
    uint8_t current_return_assign;
    uint8_t current_hp_out_source;
    uint8_t current_ch1_dvs_enable;
    uint8_t current_ch2_dvs_enable;
    uint8_t mag_output_mode_flags;
    float current_ch_fader_curve_width_a;
    float current_ch_fader_curve_width_b;
    uint8_t sensor2_aux_fade_down_assign;
    uint8_t sensor3_aux_fade_down_assign;
    uint8_t ch_fader_reverse_flags;
} EEPROM_DeviceConfigV7_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    EEPROM_DeviceConfigV7_t payload;
    uint32_t crc32;
} EEPROM_ConfigRecordV7_t;

_Static_assert(sizeof(EEPROM_DeviceConfigV7_t) == 24U, "Unexpected EEPROM v7 payload layout");

typedef struct
{
    uint8_t current_ch1_input_type;
    uint8_t current_ch2_input_type;
    uint8_t current_ch_fader_a_assign;
    uint8_t current_ch_fader_b_assign;
    uint8_t current_ch_fader_post_assign;
    uint8_t current_return_assign;
    uint8_t current_hp_out_source;
    uint8_t current_ch1_input_mode;
    uint8_t current_ch2_input_mode;
    uint8_t ch_fader_dvs_delay_ms;
    uint8_t mag_output_mode_flags;
    float current_ch_fader_curve_width_a;
    float current_ch_fader_curve_width_b;
    uint8_t sensor2_aux_fade_down_assign;
    uint8_t sensor3_aux_fade_down_assign;
    uint8_t ch_fader_reverse_flags;
} EEPROM_DeviceConfigV8_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    EEPROM_DeviceConfigV8_t payload;
    uint32_t crc32;
} EEPROM_ConfigRecordV8_t;

_Static_assert(sizeof(EEPROM_DeviceConfigV8_t) == 24U, "Unexpected EEPROM v8 payload layout");

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    EEPROM_DeviceConfig_t payload;
    uint32_t crc32;
} EEPROM_ConfigRecord_t;

_Static_assert(sizeof(EEPROM_DeviceConfig_t) == 28U, "Unexpected EEPROM v9 payload layout");

static uint32_t EEPROM_CRC32(const uint8_t* data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            uint32_t mask = (uint32_t) (-(int32_t) (crc & 1UL));
            crc           = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

void EEPROM_ConfigSetDefaults(EEPROM_DeviceConfig_t* cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->current_ch1_input_type = 0U; /* INPUT_TYPE_LINE */
    cfg->current_ch2_input_type = 0U; /* INPUT_TYPE_LINE */
    cfg->current_ch_fader_a_assign     = 0U; /* INPUT_SRC_CH1_LN */
    cfg->current_ch_fader_b_assign     = 2U; /* INPUT_SRC_CH2_LN */
    cfg->current_ch_fader_post_assign  = 4U; /* INPUT_SRC_USB12 */
    cfg->current_return_assign  = 5U; /* INPUT_SRC_USB34 */
    cfg->current_hp_out_source  = CUE_SEL_MST;
    cfg->current_ch1_input_mode = UI_INPUT_MODE_DISABLED;
    cfg->current_ch2_input_mode = UI_INPUT_MODE_DISABLED;
    cfg->ch_fader_dvs_delay_ms  = UI_CH_FADER_DVS_DELAY_DEFAULT_MS;
    cfg->mag_output_mode_flags  = 0U;
    cfg->current_ch_fader_curve_width_a = UI_CH_FADER_CURVE_WIDTH_A_DEFAULT;
    cfg->current_ch_fader_curve_width_b = UI_CH_FADER_CURVE_WIDTH_B_DEFAULT;
    cfg->sensor2_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_A;
    cfg->sensor3_aux_fade_down_assign = UI_CH_FADER_AUX_ASSIGN_B;
    cfg->ch_fader_reverse_flags = 0U;
    cfg->timecode_synth_ratio_set = TIMECODE_RATIO_OCTAVE;
    cfg->timecode_synth_warp_algorithm = TIMECODE_WARP_CROSSFOLD;
}

void EEPROM_ConfigCaptureCurrent(EEPROM_DeviceConfig_t* cfg)
{
    UI_ControlPersistState_t state;

    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    ui_control_get_persist_state(&state);
    cfg->current_ch1_input_type = state.current_ch1_input_type;
    cfg->current_ch2_input_type = state.current_ch2_input_type;
    cfg->current_ch_fader_a_assign     = state.current_ch_fader_a_assign;
    cfg->current_ch_fader_b_assign     = state.current_ch_fader_b_assign;
    cfg->current_ch_fader_post_assign  = state.current_ch_fader_post_assign;
    cfg->current_return_assign  = state.current_return_assign;
    cfg->current_hp_out_source  = state.current_hp_out_source;
    cfg->current_ch1_input_mode = state.current_ch1_input_mode;
    cfg->current_ch2_input_mode = state.current_ch2_input_mode;
    cfg->ch_fader_dvs_delay_ms  = state.ch_fader_dvs_delay_ms;
    cfg->mag_output_mode_flags  = state.mag_out_as_note ? EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE : 0U;
    cfg->current_ch_fader_curve_width_a = state.current_ch_fader_curve_width_a;
    cfg->current_ch_fader_curve_width_b = state.current_ch_fader_curve_width_b;
    cfg->sensor2_aux_fade_down_assign = state.sensor2_aux_fade_down_assign;
    cfg->sensor3_aux_fade_down_assign = state.sensor3_aux_fade_down_assign;
    cfg->ch_fader_reverse_flags = 0U;
    if (state.ch_fader_reverse_a)
    {
        cfg->ch_fader_reverse_flags |= EEPROM_CFG_FLAG_CH_FADER_REVERSE_A;
    }
    if (state.ch_fader_reverse_b)
    {
        cfg->ch_fader_reverse_flags |= EEPROM_CFG_FLAG_CH_FADER_REVERSE_B;
    }
    cfg->timecode_synth_ratio_set =
        (uint8_t) timecode_synth_get_ratio_set();
    cfg->timecode_synth_warp_algorithm =
        (uint8_t) timecode_synth_get_warp_algorithm();
}

bool eeprom_config_is_mag_out_as_note(const EEPROM_DeviceConfig_t* cfg)
{
    return (cfg != NULL) &&
           ((cfg->mag_output_mode_flags & EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) != 0U);
}

bool eeprom_config_is_ch_fader_reverse_a_enabled(const EEPROM_DeviceConfig_t* cfg)
{
    return (cfg != NULL) &&
           ((cfg->ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_A) != 0U);
}

bool eeprom_config_is_ch_fader_reverse_b_enabled(const EEPROM_DeviceConfig_t* cfg)
{
    return (cfg != NULL) &&
           ((cfg->ch_fader_reverse_flags & EEPROM_CFG_FLAG_CH_FADER_REVERSE_B) != 0U);
}

bool eeprom_config_to_ui_persist_state(const EEPROM_DeviceConfig_t* cfg,
                                       UI_ControlPersistState_t* ui_state)
{
    if ((cfg == NULL) || (ui_state == NULL))
    {
        return false;
    }

    ui_state->current_ch1_input_type = cfg->current_ch1_input_type;
    ui_state->current_ch2_input_type = cfg->current_ch2_input_type;
    ui_state->current_ch_fader_a_assign     = cfg->current_ch_fader_a_assign;
    ui_state->current_ch_fader_b_assign     = cfg->current_ch_fader_b_assign;
    ui_state->current_ch_fader_post_assign  = cfg->current_ch_fader_post_assign;
    ui_state->current_return_assign  = cfg->current_return_assign;
    ui_state->current_hp_out_source  = cfg->current_hp_out_source;
    ui_state->current_ch1_input_mode = cfg->current_ch1_input_mode;
    ui_state->current_ch2_input_mode = cfg->current_ch2_input_mode;
    ui_state->ch_fader_dvs_delay_ms = cfg->ch_fader_dvs_delay_ms;
    ui_state->sensor2_aux_fade_down_assign = cfg->sensor2_aux_fade_down_assign;
    ui_state->sensor3_aux_fade_down_assign = cfg->sensor3_aux_fade_down_assign;
    ui_state->ch_fader_reverse_a = eeprom_config_is_ch_fader_reverse_a_enabled(cfg);
    ui_state->ch_fader_reverse_b = eeprom_config_is_ch_fader_reverse_b_enabled(cfg);
    ui_state->mag_out_as_note = eeprom_config_is_mag_out_as_note(cfg);
    ui_state->current_ch_fader_curve_width_a = cfg->current_ch_fader_curve_width_a;
    ui_state->current_ch_fader_curve_width_b = cfg->current_ch_fader_curve_width_b;

    return true;
}

static bool eeprom_config_mag_output_flags_are_valid(uint8_t flags)
{
    return (flags & (uint8_t) ~EEPROM_CFG_FLAG_MAG_OUT_AS_NOTE) == 0U;
}

static bool eeprom_config_reverse_flags_are_valid(uint8_t flags)
{
    return (flags & (uint8_t) ~(EEPROM_CFG_FLAG_CH_FADER_REVERSE_A |
                                EEPROM_CFG_FLAG_CH_FADER_REVERSE_B)) == 0U;
}

static bool eeprom_config_timecode_ratio_is_valid(uint8_t ratio_set)
{
    return ratio_set <= (uint8_t) TIMECODE_RATIO_CHORD;
}

static bool eeprom_config_timecode_warp_is_valid(uint8_t warp_algorithm)
{
    return warp_algorithm <= (uint8_t) TIMECODE_WARP_COMPARATOR;
}

bool eeprom_config_validate(const EEPROM_DeviceConfig_t* cfg)
{
    UI_ControlPersistState_t ui_state;

    if (cfg == NULL)
    {
        return false;
    }

    if (!eeprom_config_mag_output_flags_are_valid(cfg->mag_output_mode_flags) ||
        !eeprom_config_reverse_flags_are_valid(cfg->ch_fader_reverse_flags) ||
        !eeprom_config_timecode_ratio_is_valid(cfg->timecode_synth_ratio_set) ||
        !eeprom_config_timecode_warp_is_valid(cfg->timecode_synth_warp_algorithm))
    {
        return false;
    }

    if (!isfinite(cfg->current_ch_fader_curve_width_a) ||
        !isfinite(cfg->current_ch_fader_curve_width_b))
    {
        return false;
    }

    if (!eeprom_config_to_ui_persist_state(cfg, &ui_state))
    {
        return false;
    }

    return ui_control_validate_persist_state(&ui_state);
}

bool eeprom_config_apply(const EEPROM_DeviceConfig_t* cfg)
{
    UI_ControlPersistState_t ui_state;

    if (!eeprom_config_validate(cfg))
    {
        return false;
    }

    if (!eeprom_config_to_ui_persist_state(cfg, &ui_state))
    {
        return false;
    }

    if (!ui_control_apply_persist_state(&ui_state))
    {
        return false;
    }

    timecode_synth_set_ratio_set(
        (TimecodeOscillatorRatioSet_t) cfg->timecode_synth_ratio_set);
    timecode_synth_set_warp_algorithm(
        (TimecodeOscillatorWarpAlgorithm_t) cfg->timecode_synth_warp_algorithm);

    return true;
}

HAL_StatusTypeDef EEPROM_SaveConfig(I2C_HandleTypeDef* hi2c, const EEPROM_DeviceConfig_t* cfg)
{
    EEPROM_ConfigRecord_t rec;
    uint32_t crc_input_len;

    if ((hi2c == NULL) || (cfg == NULL))
    {
        return HAL_ERROR;
    }

    if (!eeprom_config_validate(cfg))
    {
        return HAL_ERROR;
    }

    memset(&rec, 0, sizeof(rec));
    rec.magic        = EEPROM_CONFIG_MAGIC;
    rec.version      = EEPROM_CONFIG_VERSION;
    rec.payload_size = (uint16_t) sizeof(EEPROM_DeviceConfig_t);
    rec.payload      = *cfg;

    crc_input_len = (uint32_t) offsetof(EEPROM_ConfigRecord_t, crc32);
    rec.crc32     = EEPROM_CRC32((const uint8_t*) &rec, crc_input_len);

    return EEPROM_Write(hi2c, EEPROM_CONFIG_ADDR, (const uint8_t*) &rec, (uint16_t) sizeof(rec));
}

HAL_StatusTypeDef EEPROM_LoadConfig(I2C_HandleTypeDef* hi2c, EEPROM_DeviceConfig_t* cfg)
{
    EEPROM_ConfigHeader_t header;
    EEPROM_DeviceConfig_t candidate;
    uint32_t expected_crc;
    HAL_StatusTypeDef status;

    if ((hi2c == NULL) || (cfg == NULL))
    {
        return HAL_ERROR;
    }

    status = EEPROM_Read(hi2c, EEPROM_CONFIG_ADDR, (uint8_t*) &header, (uint16_t) sizeof(header));
    if (status != HAL_OK)
    {
        return status;
    }

    if (header.magic != EEPROM_CONFIG_MAGIC)
    {
        return HAL_ERROR;
    }

    if ((header.version == EEPROM_CONFIG_VERSION) &&
        (header.payload_size == (uint16_t) sizeof(EEPROM_DeviceConfig_t)))
    {
        EEPROM_ConfigRecord_t rec;

        status = EEPROM_Read(hi2c, EEPROM_CONFIG_ADDR, (uint8_t*) &rec, (uint16_t) sizeof(rec));
        if (status != HAL_OK)
        {
            return status;
        }

        expected_crc = EEPROM_CRC32((const uint8_t*) &rec, (uint32_t) offsetof(EEPROM_ConfigRecord_t, crc32));
        if (expected_crc != rec.crc32)
        {
            return HAL_ERROR;
        }

        memcpy(&candidate, &rec.payload, sizeof(candidate));
    }
    else if ((header.version == EEPROM_CONFIG_VERSION_V8) &&
             (header.payload_size == (uint16_t) sizeof(EEPROM_DeviceConfigV8_t)))
    {
        EEPROM_ConfigRecordV8_t rec_v8;

        status = EEPROM_Read(hi2c, EEPROM_CONFIG_ADDR, (uint8_t*) &rec_v8, (uint16_t) sizeof(rec_v8));
        if (status != HAL_OK)
        {
            return status;
        }

        expected_crc = EEPROM_CRC32((const uint8_t*) &rec_v8, (uint32_t) offsetof(EEPROM_ConfigRecordV8_t, crc32));
        if (expected_crc != rec_v8.crc32)
        {
            return HAL_ERROR;
        }

        EEPROM_ConfigSetDefaults(&candidate);
        candidate.current_ch1_input_type = rec_v8.payload.current_ch1_input_type;
        candidate.current_ch2_input_type = rec_v8.payload.current_ch2_input_type;
        candidate.current_ch_fader_a_assign = rec_v8.payload.current_ch_fader_a_assign;
        candidate.current_ch_fader_b_assign = rec_v8.payload.current_ch_fader_b_assign;
        candidate.current_ch_fader_post_assign = rec_v8.payload.current_ch_fader_post_assign;
        candidate.current_return_assign = rec_v8.payload.current_return_assign;
        candidate.current_hp_out_source = rec_v8.payload.current_hp_out_source;
        candidate.current_ch1_input_mode = rec_v8.payload.current_ch1_input_mode;
        candidate.current_ch2_input_mode = rec_v8.payload.current_ch2_input_mode;
        candidate.ch_fader_dvs_delay_ms = rec_v8.payload.ch_fader_dvs_delay_ms;
        candidate.mag_output_mode_flags = rec_v8.payload.mag_output_mode_flags;
        candidate.current_ch_fader_curve_width_a = rec_v8.payload.current_ch_fader_curve_width_a;
        candidate.current_ch_fader_curve_width_b = rec_v8.payload.current_ch_fader_curve_width_b;
        candidate.sensor2_aux_fade_down_assign = rec_v8.payload.sensor2_aux_fade_down_assign;
        candidate.sensor3_aux_fade_down_assign = rec_v8.payload.sensor3_aux_fade_down_assign;
        candidate.ch_fader_reverse_flags = rec_v8.payload.ch_fader_reverse_flags;
    }
    else if ((header.version == EEPROM_CONFIG_VERSION_V7) &&
             (header.payload_size == (uint16_t) sizeof(EEPROM_DeviceConfigV7_t)))
    {
        EEPROM_ConfigRecordV7_t rec_v7;

        status = EEPROM_Read(hi2c, EEPROM_CONFIG_ADDR, (uint8_t*) &rec_v7, (uint16_t) sizeof(rec_v7));
        if (status != HAL_OK)
        {
            return status;
        }

        expected_crc = EEPROM_CRC32((const uint8_t*) &rec_v7, (uint32_t) offsetof(EEPROM_ConfigRecordV7_t, crc32));
        if (expected_crc != rec_v7.crc32)
        {
            return HAL_ERROR;
        }

        EEPROM_ConfigSetDefaults(&candidate);
        candidate.current_ch1_input_type = rec_v7.payload.current_ch1_input_type;
        candidate.current_ch2_input_type = rec_v7.payload.current_ch2_input_type;
        candidate.current_ch_fader_a_assign = rec_v7.payload.current_ch_fader_a_assign;
        candidate.current_ch_fader_b_assign = rec_v7.payload.current_ch_fader_b_assign;
        candidate.current_ch_fader_post_assign = rec_v7.payload.current_ch_fader_post_assign;
        candidate.current_return_assign = rec_v7.payload.current_return_assign;
        candidate.current_hp_out_source = rec_v7.payload.current_hp_out_source;
        candidate.current_ch1_input_mode = (rec_v7.payload.current_ch1_dvs_enable != 0U) ? UI_INPUT_MODE_DVS : UI_INPUT_MODE_DISABLED;
        candidate.current_ch2_input_mode = (rec_v7.payload.current_ch2_dvs_enable != 0U) ? UI_INPUT_MODE_DVS : UI_INPUT_MODE_DISABLED;
        candidate.mag_output_mode_flags = rec_v7.payload.mag_output_mode_flags;
        candidate.current_ch_fader_curve_width_a = rec_v7.payload.current_ch_fader_curve_width_a;
        candidate.current_ch_fader_curve_width_b = rec_v7.payload.current_ch_fader_curve_width_b;
        candidate.sensor2_aux_fade_down_assign = rec_v7.payload.sensor2_aux_fade_down_assign;
        candidate.sensor3_aux_fade_down_assign = rec_v7.payload.sensor3_aux_fade_down_assign;
        candidate.ch_fader_reverse_flags = rec_v7.payload.ch_fader_reverse_flags;
    }
    else
    {
        return HAL_ERROR;
    }

    *cfg = candidate;
    return HAL_OK;
}
