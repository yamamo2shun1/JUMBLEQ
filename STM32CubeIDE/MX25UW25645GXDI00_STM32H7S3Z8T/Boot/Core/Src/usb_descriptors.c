/*
 * TinyUSB device descriptors for UF2 MSC bootloader
 */
#include "stm32h7rsxx_hal.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "boot_config.h"
#include <string.h>
#include <stdio.h>

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = BOOT_USB_VID,
    .idProduct          = BOOT_USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 0x01,
};

uint8_t const* tud_descriptor_device_cb(void)
{
  return (uint8_t const*)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

// Endpoint numbers for MSC
#if defined(TUD_ENDPOINT_ONE_DIRECTION_ONLY)
  #define EPNUM_MSC_OUT 0x01
  #define EPNUM_MSC_IN  0x82
#else
  #define EPNUM_MSC_OUT 0x01
  #define EPNUM_MSC_IN  0x81
#endif

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EPNUM_MSC_OUT, EPNUM_MSC_IN, CFG_TUD_MSC_EP_BUFSIZE),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

#if TUD_OPT_HIGH_SPEED
static tusb_desc_device_qualifier_t const desc_device_qualifier = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved          = 0x00,
};

uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
  return (uint8_t const*)&desc_device_qualifier;
}

uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}
#endif

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
static char const* string_desc_arr[] = {
    (const char[]) {0x09, 0x04}, // 0: English (0x0409)
    "JUMBLEQ",                  // 1: Manufacturer
    BOOT_UF2_VOLUME_LABEL,      // 2: Product
    NULL,                       // 3: Serial (use UID)
};

static uint16_t _desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  size_t chr_count;

  switch (index)
  {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
    {
      char sstr[30] = "";
      snprintf(sstr, sizeof(sstr), "%08lX%08lX%08lX",
               (unsigned long)HAL_GetUIDw0(),
               (unsigned long)HAL_GetUIDw1(),
               (unsigned long)HAL_GetUIDw2());
      chr_count = strlen(sstr);
      if (chr_count > 31)
      {
        chr_count = 31;
      }
      for (uint8_t i = 0; i < chr_count; i++)
      {
        _desc_str[1 + i] = sstr[i];
      }
      break;
    }

    default:
    {
      if (!(index < (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))))
      {
        return NULL;
      }
      const char* str = string_desc_arr[index];
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
      if (chr_count > max_count)
      {
        chr_count = max_count;
      }
      for (size_t i = 0; i < chr_count; i++)
      {
        _desc_str[1 + i] = str[i];
      }
      break;
    }
  }

  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
