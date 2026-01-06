// FILE: src/drivers/ms5611.h
#ifndef __MS5611_H__
#define __MS5611_H__

#include <stdbool.h>
#include <stdint.h>

#include "i2c_pico.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// 0: OK
// <0: error
// I2C 오류는 i2c_pico_status_t를 그대로 패스스루(예: -5, -13 등)
typedef int32_t ms5611_status_t;

enum {
    MS5611_OK       = 0,

    MS5611_EINVAL   = -2000,
    MS5611_ESTATE   = -2001,
    MS5611_EPROM    = -2002,
    MS5611_ECRC     = -2003,
    MS5611_ERANGE   = -2004
};

const char *ms5611_status_str(ms5611_status_t st);

// OSR (conversion command에 매핑)
typedef enum {
    MS5611_OSR_256  = 256,
    MS5611_OSR_512  = 512,
    MS5611_OSR_1024 = 1024,
    MS5611_OSR_2048 = 2048,
    MS5611_OSR_4096 = 4096,
} ms5611_osr_t;

typedef struct {
    ms5611_osr_t osr; // pressure/temperature에 동일 OSR 적용(간단/안전)
} ms5611_config_t;

typedef struct {
    i2c_pico_t *i2c;
    uint8_t addr7;
    bool initialized;

    // PROM words (0..7)
    uint16_t prom[8];
} ms5611_t;

void ms5611_config_default(ms5611_config_t *cfg);

// init: reset -> PROM read -> CRC check
ms5611_status_t ms5611_init(ms5611_t *dev, i2c_pico_t *i2c, uint8_t addr7);

// low-level
ms5611_status_t ms5611_reset(ms5611_t *dev);
ms5611_status_t ms5611_read_prom(ms5611_t *dev, uint16_t out_prom[8]); // also stores in dev

// read
// returns: temp_c_x100 (0.01°C), press_pa (Pa)
ms5611_status_t ms5611_read(ms5611_t *dev,
                            const ms5611_config_t *cfg,
                            int32_t *temp_c_x100,
                            uint32_t *press_pa);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __MS5611_H__ */