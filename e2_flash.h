#ifndef __E2_FLASH_HEADER__
#define __E2_FLASH_HEADER__
#include <stdint.h>
uint8_t e2_is_blank(uint16_t *addr);
uint8_t e2_erase(uint16_t *addr);
uint8_t e2_write(uint16_t dat, uint16_t *addr);
uint16_t e2_read(uint16_t *addr);
void e2_clear_status(uint16_t *addr);
#endif
