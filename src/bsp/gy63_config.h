// FILE: src/bsp/gy63_config.h
#ifndef __GY63_CONFIG_H__
#define __GY63_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform/hal/i2c_pico.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


i2c_pico_status_t gy63_bsp_init(void);
void              gy63_bsp_deinit(void);

i2c_pico_t       *gy63_bsp_i2c(void);
uint8_t           gy63_bsp_addr7(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __GY63_CONFIG_H__ */
