#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <stdint.h>

enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL
};

enum
{
  ITF_NUM_MSC = 0,
  ITF_NUM_TOTAL
};

#endif /* USB_DESCRIPTORS_H */
