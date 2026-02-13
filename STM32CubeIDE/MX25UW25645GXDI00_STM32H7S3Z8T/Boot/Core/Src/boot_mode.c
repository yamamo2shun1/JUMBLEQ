#include "boot_mode.h"
#include "main.h"
#include "stm32h7rsxx.h"

bool boot_mode_sw2_pressed(void)
{
  return (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET);
}

bool boot_mode_usb_vbus_present(void)
{
#ifdef USB_OTG_HS
  return ((USB_OTG_HS->GOTGCTL & USB_OTG_GOTGCTL_BSESVLD) != 0u);
#else
  return false;
#endif
}

bool boot_mode_should_enter_uf2(void)
{
  return boot_mode_sw2_pressed() && boot_mode_usb_vbus_present();
}
