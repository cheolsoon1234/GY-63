// FILE: src/bsp/gy63_config.c

#include "gy63_config.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

static i2c_pico_t s_i2c;
static bool s_inited = false;

static const uint     SDA_GPIO = 8;         // GPIO8 SDA
static const uint     SCL_GPIO = 9;         // GPIO9 SCL
static const uint32_t BAUD_HZ  = 400000;    // 400kHz
static const uint8_t  ADDR7    = 0x77;      // CSB  PULL DOWN   (NC: 0x77 / IF HIGH 0x76)
                                            // SDO  PULL UP     (NC)
                                            // PS   PULL UP     (NC: I2C mode)

static const uint32_t TIMEOUT_US = 20000;
static const bool     ENABLE_PULLUPS = true;

i2c_pico_status_t gy63_bsp_init(void) {
    if (s_inited) return I2C_PICO_OK;

    i2c_pico_config_t cfg = {
        .instance       = i2c0,
        .sda_pin        = SDA_GPIO,
        .scl_pin        = SCL_GPIO,
        .baudrate_hz    = BAUD_HZ,
        .timeout_us     = TIMEOUT_US,
        .enable_pullups = ENABLE_PULLUPS,
    };

    i2c_pico_status_t st = i2c_pico_init(&s_i2c, &cfg);
    if (st != I2C_PICO_OK) {
        memset(&s_i2c, 0, sizeof(s_i2c));
        s_inited = false;
        return st;
    }

    s_inited = true;
    return I2C_PICO_OK;
}

void gy63_bsp_deinit(void) {
    if (!s_inited) return;
    i2c_pico_deinit(&s_i2c);
    memset(&s_i2c, 0, sizeof(s_i2c));
    s_inited = false;
}

i2c_pico_t *gy63_bsp_i2c(void) {
    return s_inited ? &s_i2c : NULL;
}

uint8_t gy63_bsp_addr7(void) {
    return ADDR7;
}
