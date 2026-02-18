/*
 * ui_control.c
 *
 *  Created on: Feb 18, 2026
 */

#include "ui_control.h"

#include "audio_control.h"

#include "adc.h"
#include "hpdma.h"
#include "linked_list.h"

#include "adau1466.h"
#include "SigmaStudioFW.h"

extern DMA_QListTypeDef List_HPDMA1_Channel0;

enum
{
    INPUT_SRC_CH1_LN = 0,
    INPUT_SRC_CH1_PN,
    INPUT_SRC_CH2_LN,
    INPUT_SRC_CH2_PN,
    INPUT_SRC_USB,
};

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

typedef struct
{
    uint8_t  current_ch1_input_type;
    uint8_t  current_ch2_input_type;
    uint8_t  current_xfA_assign;
    uint8_t  current_xfB_assign;
    uint8_t  current_xfpost_assign;
    uint8_t  current_xfA_position;
    uint8_t  current_xfB_position;
    uint8_t  pot_ch;
    uint8_t  pot_ch_counter;
    uint16_t pot_ma_index[POT_NUM];
    uint32_t pot_val_ma[POT_NUM][POT_MA_SIZE];
    uint16_t pot_val[POT_NUM];
    uint16_t pot_val_prev[POT_NUM][2];
    uint16_t mag_calibration_count;
    uint16_t mag_val[MAG_SW_NUM];
    uint32_t mag_offset_sum[MAG_SW_NUM];
    uint16_t mag_offset[MAG_SW_NUM];
    float    xfade[MAG_SW_NUM];
    float    xfade_prev[MAG_SW_NUM];
    float    xfade_min[MAG_SW_NUM];
    float    xfade_max[MAG_SW_NUM];
    bool     is_start_audio_control;
} ui_control_state_t;

static ui_control_state_t s_ui = {
    .current_ch1_input_type = INPUT_TYPE_LINE,
    .current_ch2_input_type = INPUT_TYPE_LINE,
    .current_xfA_assign     = INPUT_SRC_CH2_LN,
    .current_xfB_assign     = INPUT_SRC_CH1_LN,
    .current_xfpost_assign  = INPUT_SRC_USB,
    .current_xfA_position   = 127,
    .current_xfB_position   = 127,
    .xfade                  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xfade_prev             = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xfade_min              = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .xfade_max              = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .is_start_audio_control = false,
};

static volatile bool is_adc_complete = false;

static void send_control_change(uint8_t number, uint8_t value, uint8_t channel)
{
    uint8_t control_change[3] = {0xB0 | channel, number, value};
    tud_midi_stream_write(0, control_change, 3);
}

uint8_t get_current_xfA_position(void)
{
    return s_ui.current_xfA_position;
}

uint8_t get_current_xfB_position(void)
{
    return s_ui.current_xfB_position;
}

int16_t get_current_ch1_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[6]);
}

int16_t get_current_ch2_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[4]);
}

int16_t get_current_master_db(void)
{
    return convert_pot2dB_int(s_ui.pot_val[5]);
}

int16_t get_current_dry_wet(void)
{
    int16_t pct = (int16_t) (((double) s_ui.pot_val[7] / 1023.0 * 100.0) + 0.5);
    if (pct < 0)
    {
        pct = 0;
    }
    if (pct > 100)
    {
        pct = 100;
    }
    return pct;
}

char* get_current_input_typeA_str(void)
{
    switch (s_ui.current_xfA_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH2_LN:
        return "[line]";
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_CH2_PN:
        return "[phono]";
    default:
        return "[]";
    }
}

char* get_current_input_typeB_str(void)
{
    switch (s_ui.current_xfB_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH2_LN:
        return " [line]";
    case INPUT_SRC_CH1_PN:
    case INPUT_SRC_CH2_PN:
        return "[phono]";
    default:
        return "     []";
    }
}

char* get_current_input_srcA_str(void)
{
    switch (s_ui.current_xfA_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return "A:Ch1";
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return "A:Ch2";
    case INPUT_SRC_USB:
        return "A:USB";
    default:
        return "A:";
    }
}

char* get_current_input_srcB_str(void)
{
    switch (s_ui.current_xfB_assign)
    {
    case INPUT_SRC_CH1_LN:
    case INPUT_SRC_CH1_PN:
        return "B:Ch1";
    case INPUT_SRC_CH2_LN:
    case INPUT_SRC_CH2_PN:
        return "B:Ch2";
    case INPUT_SRC_USB:
        return "B:USB";
    default:
        return "B:";
    }
}

char* get_current_input_srcP_str(void)
{
    switch (s_ui.current_xfpost_assign)
    {
    case INPUT_SRC_CH1_LN:
        return "THRU:Ch1[line]";
    case INPUT_SRC_CH1_PN:
        return "THRU:Ch1[phono]";
    case INPUT_SRC_CH2_LN:
        return "THRU:Ch2[line]";
    case INPUT_SRC_CH2_PN:
        return "THRU:Ch2[phono]";
    case INPUT_SRC_USB:
        return "THRU:USB";
    default:
        return "THRU:";
    }
}

void ui_control_dma_adc_cplt(DMA_HandleTypeDef* hdma)
{
    (void) hdma;
    is_adc_complete = true;
    __DSB();
}

void ui_control_set_adc_complete(bool complete)
{
    is_adc_complete = complete;
    __DMB();
}

void start_adc(void)
{
    if (MX_List_HPDMA1_Channel0_Config() != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_List_LinkQ(&handle_HPDMA1_Channel0, &List_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
    s_ui.pot_ch = 1;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);
    SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMACFG);

    handle_HPDMA1_Channel0.XferCpltCallback = ui_control_dma_adc_cplt;
    if (HAL_DMAEx_List_Start_IT(&handle_HPDMA1_Channel0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

static uint8_t input_src_from_channel_type(uint8_t input_ch, uint8_t input_type)
{
    switch (input_ch)
    {
    case INPUT_CH1:
        return (input_type == INPUT_TYPE_PHONO) ? INPUT_SRC_CH1_PN : INPUT_SRC_CH1_LN;
    case INPUT_CH2:
        return (input_type == INPUT_TYPE_PHONO) ? INPUT_SRC_CH2_PN : INPUT_SRC_CH2_LN;
    case INPUT_USB:
    default:
        return INPUT_SRC_USB;
    }
}

static uint8_t current_input_src_from_channel(uint8_t input_ch)
{
    switch (input_ch)
    {
    case INPUT_CH1:
        return input_src_from_channel_type(INPUT_CH1, s_ui.current_ch1_input_type);
    case INPUT_CH2:
        return input_src_from_channel_type(INPUT_CH2, s_ui.current_ch2_input_type);
    case INPUT_USB:
    default:
        return INPUT_SRC_USB;
    }
}

static void replace_assign_for_input_channel(uint8_t* assign, uint8_t input_ch, uint8_t new_src)
{
    const uint8_t ln_src = input_src_from_channel_type(input_ch, INPUT_TYPE_LINE);
    const uint8_t pn_src = input_src_from_channel_type(input_ch, INPUT_TYPE_PHONO);

    if (*assign == ln_src || *assign == pn_src)
    {
        *assign = new_src;
    }
}

static void apply_input_type_change(uint8_t input_ch, uint8_t input_type)
{
    const uint8_t new_src = input_src_from_channel_type(input_ch, input_type);

    select_input_type(input_ch, input_type);
    if (input_ch == INPUT_CH1)
    {
        s_ui.current_ch1_input_type = input_type;
    }
    else if (input_ch == INPUT_CH2)
    {
        s_ui.current_ch2_input_type = input_type;
    }

    replace_assign_for_input_channel(&s_ui.current_xfA_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_ui.current_xfB_assign, input_ch, new_src);
    replace_assign_for_input_channel(&s_ui.current_xfpost_assign, input_ch, new_src);
}

static void apply_xf_assign_a(uint8_t input_ch)
{
    select_xf_assignA_source(input_ch);
    s_ui.current_xfA_assign = current_input_src_from_channel(input_ch);
}

static void apply_xf_assign_b(uint8_t input_ch)
{
    select_xf_assignB_source(input_ch);
    s_ui.current_xfB_assign = current_input_src_from_channel(input_ch);
}

static void apply_xf_assign_post(uint8_t input_ch)
{
    select_xf_assignPost_source(input_ch);
    s_ui.current_xfpost_assign = current_input_src_from_channel(input_ch);
}

static void set_pot_mux_channel(uint8_t channel)
{
    static const uint8_t mux_bits[POT_NUM][3] = {
        {0, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 1, 1},
        {1, 0, 0}, {1, 1, 0}, {1, 0, 1}, {1, 1, 1},
    };

    if (channel >= POT_NUM)
    {
        channel = 0;
    }

    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, mux_bits[channel][0]);
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, mux_bits[channel][1]);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, mux_bits[channel][2]);
}

static void apply_pot_value(uint8_t channel, uint16_t value)
{
    switch (channel)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        send_control_change(channel, value, 0);
        break;
    case 4:
        control_input_from_ch2_gain(value);
        break;
    case 5:
        control_master_out_gain(value);
        break;
    case 6:
        control_input_from_ch1_gain(value);
        break;
    case 7:
        control_dryB_out_gain(value);
        control_wet_out_gain(value);
        break;
    default:
        break;
    }
}

static uint32_t read_pot_sample_from_adc(uint8_t channel, uint32_t adc_raw)
{
    switch (channel)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        return adc_raw >> 5;
    case 4:
    case 5:
    case 6:
    case 7:
        return adc_raw >> 2;
    default:
        return 0;
    }
}

static void ui_control_process_pot(void)
{
    if (s_ui.pot_ch_counter < POT_CH_SEL_WAIT)
    {
        set_pot_mux_channel(s_ui.pot_ch);
        s_ui.pot_ch_counter++;
    }
    else if (s_ui.pot_ch_counter >= POT_CH_SEL_WAIT)
    {
        s_ui.pot_val_ma[s_ui.pot_ch][s_ui.pot_ma_index[s_ui.pot_ch]] = read_pot_sample_from_adc(s_ui.pot_ch, adc_val[6]);
        s_ui.pot_ma_index[s_ui.pot_ch] = (s_ui.pot_ma_index[s_ui.pot_ch] + 1) % POT_MA_SIZE;

        float pot_sum = 0.0f;
        for (int j = 0; j < POT_MA_SIZE; j++)
        {
            pot_sum += (float) s_ui.pot_val_ma[s_ui.pot_ch][j];
        }
        s_ui.pot_val[s_ui.pot_ch] = round(pot_sum / (float) POT_MA_SIZE);

        uint8_t stable_count = 0;
        if (s_ui.pot_val[s_ui.pot_ch] == s_ui.pot_val_prev[s_ui.pot_ch][0])
        {
            stable_count++;
        }
        if (s_ui.pot_val[s_ui.pot_ch] == s_ui.pot_val_prev[s_ui.pot_ch][1])
        {
            stable_count++;
        }
        if (s_ui.pot_val_prev[s_ui.pot_ch][0] == s_ui.pot_val_prev[s_ui.pot_ch][1])
        {
            stable_count++;
        }

        if (stable_count <= 1)
        {
            apply_pot_value(s_ui.pot_ch, s_ui.pot_val[s_ui.pot_ch]);
        }

        s_ui.pot_val_prev[s_ui.pot_ch][1] = s_ui.pot_val_prev[s_ui.pot_ch][0];
        s_ui.pot_val_prev[s_ui.pot_ch][0] = s_ui.pot_val[s_ui.pot_ch];

        s_ui.pot_ch         = (s_ui.pot_ch + 1) % POT_NUM;
        s_ui.pot_ch_counter = 0;
    }
}

static void ui_control_update_mag_samples(void)
{
    for (int i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i] = (uint16_t) adc_val[i];

        if (s_ui.mag_calibration_count < MAG_CALIBRATION_COUNT_MAX)
        {
            s_ui.mag_offset_sum[i] += adc_val[i];
        }
        else if (s_ui.mag_calibration_count == MAG_CALIBRATION_COUNT_MAX)
        {
            s_ui.mag_offset[i] = s_ui.mag_offset_sum[i] / MAG_CALIBRATION_COUNT_MAX;
        }
    }
    if (s_ui.mag_calibration_count <= MAG_CALIBRATION_COUNT_MAX)
    {
        s_ui.mag_calibration_count++;
    }
}

static void ui_control_update_xfade_from_mag(void)
{
    int index[MAG_SW_NUM] = {0, 5, 1, 2, 3, 4};
    for (int j = 0; j < MAG_SW_NUM; j++)
    {
        int i = index[j];
        if (i == 0 || i == 5)
        {
            if (s_ui.mag_val[i] < s_ui.mag_offset[i] + MAG_XFADE_CUTOFF)
            {
                s_ui.xfade[i] = 0.0f;
            }
            else if (s_ui.mag_val[i] >= s_ui.mag_offset[i] + MAG_XFADE_CUTOFF && s_ui.mag_val[i] <= s_ui.mag_offset[i] + MAG_XFADE_RANGE)
            {
                s_ui.xfade[i] = (float) (s_ui.mag_val[i] - s_ui.mag_offset[i] - MAG_XFADE_CUTOFF) / (float) MAG_XFADE_RANGE;
            }
            else if (s_ui.mag_val[i] > s_ui.mag_offset[i] + MAG_XFADE_RANGE)
            {
                s_ui.xfade[i] = 1.0f;
            }

            if (s_ui.xfade[i] >= s_ui.xfade_max[i])
            {
                s_ui.xfade_max[i] = s_ui.xfade[i];

                if (i == 0)
                {
                    s_ui.xfade_min[1] = s_ui.xfade_max[i];
                }
                else if (i == 5)
                {
                    s_ui.xfade_min[4] = s_ui.xfade_max[i];
                }
            }
        }
        else
        {
            if (s_ui.mag_val[i] < s_ui.mag_offset[i] + MAG_XFADE_CUTOFF)
            {
                s_ui.xfade[i] = 1.0f;
            }
            else if (s_ui.mag_val[i] >= s_ui.mag_offset[i] + MAG_XFADE_CUTOFF && s_ui.mag_val[i] <= s_ui.mag_offset[i] + MAG_XFADE_RANGE)
            {
                s_ui.xfade[i] = 1.0f - ((float) (s_ui.mag_val[i] - s_ui.mag_offset[i] - MAG_XFADE_CUTOFF) / (float) MAG_XFADE_RANGE);
            }
            else if (s_ui.mag_val[i] > s_ui.mag_offset[i] + MAG_XFADE_RANGE)
            {
                s_ui.xfade[i] = 0.0f;
            }

            if (s_ui.xfade[i] <= s_ui.xfade_min[i])
            {
                s_ui.xfade_min[i] = s_ui.xfade[i];

                if (s_ui.xfade_min[i] < 0.05f)
                {
                    if (i == 1)
                    {
                        s_ui.xfade_max[0] = 0.0f;
                    }
                    else if (i == 4)
                    {
                        s_ui.xfade_max[5] = 0.0f;
                    }
                }
            }
        }
    }
}

static void ui_control_apply_xfade_updates(void)
{
    bool xfadeA_changed = false;
    bool xfadeB_changed = false;
    for (int i = 0; i < MAG_SW_NUM; i++)
    {
        if (fabs(s_ui.xfade[i] - s_ui.xfade_prev[i]) > 0.01f)
        {
            send_control_change(10 + (5 - i), (uint8_t) (127.0f - s_ui.xfade[i] * 127.0f), 0);

            if (i == 0 || i == 1)
            {
                xfadeB_changed = true;
            }
            if (i == 4 || i == 5)
            {
                xfadeA_changed = true;
            }

            s_ui.xfade_prev[i] = s_ui.xfade[i];
        }
    }

    if (xfadeA_changed)
    {
        const float xf = pow(s_ui.xfade_max[5] * s_ui.xfade_min[4], 1.0f / 3.0f);
        set_dc_inputA(xf);
        s_ui.current_xfA_position = (uint8_t) (xf * 128.0f);
    }

    if (xfadeB_changed)
    {
        const float xf = pow(s_ui.xfade_max[0] * s_ui.xfade_min[1], 1.0f / 3.0f);
        set_dc_inputB(xf);
        s_ui.current_xfB_position = (uint8_t) (xf * 128.0f);
    }
}

static void ui_control_process_mag(void)
{
    ui_control_update_mag_samples();

    if (s_ui.mag_calibration_count > MAG_CALIBRATION_COUNT_MAX)
    {
        ui_control_update_xfade_from_mag();
        ui_control_apply_xfade_updates();
    }
}

typedef void (*midi_program_handler_t)(uint8_t arg);

typedef struct
{
    uint8_t                command;
    midi_program_handler_t handler;
    uint8_t                arg;
} midi_program_cmd_t;

static void midi_program_set_input_type(uint8_t arg)
{
    uint8_t input_ch   = (arg >> 4) & 0x0F;
    uint8_t input_type = arg & 0x0F;
    apply_input_type_change(input_ch, input_type);
}

static void midi_program_apply_xf_a(uint8_t input_ch)
{
    apply_xf_assign_a(input_ch);
}

static void midi_program_apply_xf_b(uint8_t input_ch)
{
    apply_xf_assign_b(input_ch);
}

static void midi_program_apply_xf_post(uint8_t input_ch)
{
    apply_xf_assign_post(input_ch);
}

static bool ui_control_dispatch_midi_program_change(uint8_t program)
{
    static const midi_program_cmd_t commands[] = {
        {CH1_LINE, midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_LINE)},
        {CH1_PHONO, midi_program_set_input_type, (uint8_t) ((INPUT_CH1 << 4) | INPUT_TYPE_PHONO)},
        {CH2_LINE, midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_LINE)},
        {CH2_PHONO, midi_program_set_input_type, (uint8_t) ((INPUT_CH2 << 4) | INPUT_TYPE_PHONO)},
        {XF_ASSIGN_A_CH1, midi_program_apply_xf_a, INPUT_CH1},
        {XF_ASSIGN_A_CH2, midi_program_apply_xf_a, INPUT_CH2},
        {XF_ASSIGN_A_USB, midi_program_apply_xf_a, INPUT_USB},
        {XF_ASSIGN_B_CH1, midi_program_apply_xf_b, INPUT_CH1},
        {XF_ASSIGN_B_CH2, midi_program_apply_xf_b, INPUT_CH2},
        {XF_ASSIGN_B_USB, midi_program_apply_xf_b, INPUT_USB},
        {XF_ASSIGN_POST_CH1, midi_program_apply_xf_post, INPUT_CH1},
        {XF_ASSIGN_POST_CH2, midi_program_apply_xf_post, INPUT_CH2},
        {XF_ASSIGN_POST_USB, midi_program_apply_xf_post, INPUT_USB},
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

static void ui_control_process_midi_rx(void)
{
    while (tud_midi_available())
    {
        uint8_t packet[4];
        tud_midi_packet_read(packet);

        if ((packet[1] & 0xF0) == 0xC0)
        {
            (void) ui_control_dispatch_midi_program_change(packet[2]);
        }

        SEGGER_RTT_printf(0, "MIDI RX: 0x%02X 0x%02X 0x%02X(%d) 0x%02X(%d)\n", packet[0], packet[1], packet[2], packet[2], packet[3], packet[3]);
    }
}

void ui_control_task(void)
{
#if !ENABLE_DSP_RUNTIME_CONTROL
    return;
#endif

    if (!is_started_audio_control() || !is_adc_complete)
    {
        return;
    }

    ui_control_process_pot();
    ui_control_process_mag();
    ui_control_process_midi_rx();
    is_adc_complete = false;
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

void ui_control_reset_state(void)
{
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        adc_val[i] = 0;
    }

    for (uint16_t i = 0; i < POT_NUM; i++)
    {
        s_ui.pot_ma_index[i]    = 0;
        s_ui.pot_val[i]         = 0;
        s_ui.pot_val_prev[i][0] = 0;
        s_ui.pot_val_prev[i][1] = 0;
        for (uint16_t j = 0; j < POT_MA_SIZE; j++)
        {
            s_ui.pot_val_ma[i][j] = 0;
        }
    }

    s_ui.mag_calibration_count = 0;
    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.mag_val[i]        = 0;
        s_ui.mag_offset_sum[i] = 0;
        s_ui.mag_offset[i]     = 0;
    }

    s_ui.current_ch1_input_type = INPUT_TYPE_LINE;
    s_ui.current_ch2_input_type = INPUT_TYPE_LINE;
    s_ui.current_xfA_assign     = INPUT_SRC_CH2_LN;
    s_ui.current_xfB_assign     = INPUT_SRC_CH1_LN;
    s_ui.current_xfpost_assign  = INPUT_SRC_USB;
    s_ui.current_xfA_position   = 127;
    s_ui.current_xfB_position   = 127;

    for (uint16_t i = 0; i < MAG_SW_NUM; i++)
    {
        s_ui.xfade[i]      = 1.0f;
        s_ui.xfade_prev[i] = 1.0f;
        s_ui.xfade_min[i]  = 1.0f;
        s_ui.xfade_max[i]  = 0.0f;
    }

    s_ui.pot_ch         = 0;
    s_ui.pot_ch_counter = 0;
    is_adc_complete     = false;
}
