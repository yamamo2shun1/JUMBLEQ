#include "led_status.h"
#include "main.h"

static led_status_t g_status = LED_STATUS_BOOT_JUMP;
static uint32_t g_t0 = 0;
static uint8_t g_phase = 0;

static void led_all_off(void)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

static void led_set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  HAL_GPIO_WritePin(port, pin, state);
}

void led_status_set(led_status_t status)
{
  g_status = status;
  g_t0 = HAL_GetTick();
  g_phase = 0;
  led_all_off();

  if (status == LED_STATUS_BOOT_JUMP)
  {
    led_set(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  }
  else if (status == LED_STATUS_VERIFY_OK)
  {
    led_set(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    led_set(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    led_set(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  }
}

void led_status_tick(uint32_t now_ms)
{
  uint32_t elapsed = now_ms - g_t0;

  switch (g_status)
  {
    case LED_STATUS_BOOT_JUMP:
      if (elapsed > 1000u)
      {
        led_all_off();
      }
      break;

    case LED_STATUS_UF2_IDLE:
      if (elapsed >= 500u)
      {
        g_t0 = now_ms;
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      }
      break;

    case LED_STATUS_UF2_WRITE:
      if (elapsed >= 125u)
      {
        g_t0 = now_ms;
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      }
      break;

    case LED_STATUS_VERIFY:
      if (elapsed >= 1000u)
      {
        g_t0 = now_ms;
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }
      break;

    case LED_STATUS_VERIFY_OK:
      if (elapsed > 200u)
      {
        led_all_off();
      }
      break;

    case LED_STATUS_ERROR:
      if (elapsed >= 100u)
      {
        g_t0 = now_ms;
        g_phase = (uint8_t)((g_phase + 1u) % 10u);
      }
      if (g_phase == 0u || g_phase == 2u)
      {
        led_set(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      }
      else
      {
        led_set(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      }
      break;

    case LED_STATUS_USB_MISSING:
      if (elapsed >= 100u)
      {
        g_t0 = now_ms;
        g_phase = (uint8_t)((g_phase + 1u) % 12u);
      }
      if (g_phase == 0u || g_phase == 2u || g_phase == 4u)
      {
        led_set(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      }
      else
      {
        led_set(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      }
      break;

    default:
      break;
  }
}
