/*
 * ui_adc_control.c
 *
 * ADC/HPDMA lifecycle and the ISR -> Task completion notification.
 * The DMA destination buffer keeps the adc_val symbol for linked_list.c.
 */

#include "ui_control.h"
#include "ui_control_internal.h"
#include "ui_adc_control_internal.h"
#include "ui_pot_control_internal.h"

#include "adc.h"
#include "hpdma.h"
#include "linked_list.h"

extern DMA_QListTypeDef List_HPDMA1_Channel0;

__attribute__((section("noncacheable_buffer"), aligned(32))) uint32_t adc_val[ADC_NUM] = {0};

static volatile bool is_adc_complete  = false;

bool ui_adc_control_is_complete(void)
{
    return is_adc_complete;
}

void ui_adc_control_clear_complete(void)
{
    is_adc_complete = false;
}

const uint32_t* ui_adc_control_samples(void)
{
    return adc_val;
}

void ui_adc_control_reset(void)
{
    for (uint16_t i = 0; i < ADC_NUM; i++)
    {
        adc_val[i] = 0;
    }

    is_adc_complete = false;
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
    ui_pot_control_set_initial_channel();

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
