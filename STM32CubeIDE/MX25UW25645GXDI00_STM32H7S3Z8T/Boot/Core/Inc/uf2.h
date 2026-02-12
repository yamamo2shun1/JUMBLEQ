#ifndef UF2_H
#define UF2_H

#include <stdbool.h>
#include <stdint.h>

void uf2_init(void);
bool uf2_process_block(const uint8_t *block512);
bool uf2_is_complete(void);
bool uf2_finalize(void);

uint32_t uf2_received_blocks(void);
uint32_t uf2_expected_blocks(void);
uint32_t uf2_image_size(void);

#endif /* UF2_H */
