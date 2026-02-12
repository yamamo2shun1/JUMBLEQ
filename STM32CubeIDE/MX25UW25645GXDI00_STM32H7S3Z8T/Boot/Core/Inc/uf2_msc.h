#ifndef UF2_MSC_H
#define UF2_MSC_H

#include <stdint.h>

void uf2_msc_init(void);
void uf2_msc_task(void);
void uf2_msc_on_block_write(const uint8_t *buf, uint32_t len);

#endif /* UF2_MSC_H */
