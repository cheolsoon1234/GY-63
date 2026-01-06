// FILE: src/platform/hal/i2c_pico.h
#ifndef __I2C_PICO_H__
#define __I2C_PICO_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef enum {
    I2C_PICO_OK = 0,

    // Parameter / state
    I2C_PICO_EINVAL  = -1,   // invalid argument
    I2C_PICO_ESTATE  = -2,   // not initialized / bad state

    // Transport / bus
    I2C_PICO_ETIMEOUT = -10, // timeout
    I2C_PICO_ENOACK   = -11, // address/data NACK
    I2C_PICO_EARBLST  = -12, // arbitration lost
    I2C_PICO_EBUS     = -13, // bus / controller abort (other)
    I2C_PICO_EIO      = -14  // unknown I/O failure
} i2c_pico_status_t;

typedef struct {
    uint8_t  address_7bit;

    size_t   write_requested;
    size_t   write_completed;

    size_t   read_requested;
    size_t   read_completed;

    // Pico SDK return code from the *last* low-level call:
    // >=0 bytes transferred, or PICO_ERROR_TIMEOUT / PICO_ERROR_GENERIC / etc.
    int      pico_result;

    // TX abort source snapshot (OR-ed if multiple ops in one API call)
    uint32_t abort_source_register;

    // whether the last op used nostop (repeated start)
    bool     nostop;
} i2c_pico_diagnostics_t;

typedef struct {
    i2c_inst_t *instance;

    uint sda_pin;
    uint scl_pin;

    uint32_t baudrate_hz;   // e.g. 100000, 400000
    uint32_t timeout_us;    // e.g. 5000~20000

    bool enable_pullups;    // true => gpio_pull_up(sda/scl)
} i2c_pico_config_t;

typedef struct {
    i2c_inst_t *instance;
    uint32_t timeout_us;
    bool is_initialized;

    i2c_pico_diagnostics_t last_diag;
} i2c_pico_t;

// Init / deinit
i2c_pico_status_t i2c_pico_init(i2c_pico_t *ctx, const i2c_pico_config_t *cfg);
void              i2c_pico_deinit(i2c_pico_t *ctx);

// Basic ops
i2c_pico_status_t i2c_pico_write(i2c_pico_t *ctx,
                                 uint8_t addr_7bit,
                                 const uint8_t *data,
                                 size_t len,
                                 bool nostop);

i2c_pico_status_t i2c_pico_read(i2c_pico_t *ctx,
                                uint8_t addr_7bit,
                                uint8_t *data,
                                size_t len,
                                bool nostop);

// Common pattern: write register address, then read payload (repeated-start)
i2c_pico_status_t i2c_pico_write_read(i2c_pico_t *ctx,
                                      uint8_t addr_7bit,
                                      const uint8_t *write_data,
                                      size_t write_len,
                                      uint8_t *read_data,
                                      size_t read_len);

// Probe helper (safe-ish for scanner): try 1-byte read and check ACK
i2c_pico_status_t i2c_pico_probe(i2c_pico_t *ctx, uint8_t addr_7bit);

// Diagnostics / strings
const i2c_pico_diagnostics_t *i2c_pico_last_diagnostics(const i2c_pico_t *ctx);
const char *i2c_pico_status_str(i2c_pico_status_t st);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __I2C_PICO_H__ */