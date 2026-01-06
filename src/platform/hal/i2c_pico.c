// FILE: src/platform/hal/i2c_pico.c
#include "i2c_pico.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/error.h"

#include "hardware/gpio.h"
#include "hardware/structs/i2c.h"
#include "hardware/regs/i2c.h"  // I2C_IC_TX_ABRT_SOURCE_* bits

// ---------- internal helpers ----------

static void diag_begin(i2c_pico_t *ctx,
                       uint8_t addr_7bit,
                       size_t wreq,
                       size_t rreq,
                       bool nostop) {
    memset(&ctx->last_diag, 0, sizeof(ctx->last_diag));
    ctx->last_diag.address_7bit      = addr_7bit;
    ctx->last_diag.write_requested   = wreq;
    ctx->last_diag.read_requested    = rreq;
    ctx->last_diag.nostop            = nostop;
    ctx->last_diag.pico_result       = 0;
    ctx->last_diag.abort_source_register = 0;
}

static void diag_capture_abort_source(i2c_pico_t *ctx) {
    if (!ctx || !ctx->instance) return;

    i2c_hw_t *hw = i2c_get_hw(ctx->instance);

    // tx_abort_source is valid if an abort occurred. Reading CLR_TX_ABRT clears the latched abort.
    uint32_t src = hw->tx_abrt_source;
    if (src) {
        ctx->last_diag.abort_source_register |= src;
        (void)hw->clr_tx_abrt; // clear abort
    }
}

static i2c_pico_status_t map_pico_result_to_status(i2c_pico_t *ctx, int pico_result) {
    if (pico_result >= 0) {
        return I2C_PICO_OK;
    }

    if (pico_result == PICO_ERROR_TIMEOUT) {
        return I2C_PICO_ETIMEOUT;
    }

    if (pico_result == PICO_ERROR_GENERIC) {
        // Try to classify using abort source register
        uint32_t abrt = ctx ? ctx->last_diag.abort_source_register : 0;

        // Address NACK / Data NACK
        if (abrt & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS |
                    I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_BITS |
                    I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_BITS |
                    I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)) {
            return I2C_PICO_ENOACK;
        }

        // Arbitration lost
        if (abrt & I2C_IC_TX_ABRT_SOURCE_ARB_LOST_BITS) {
            return I2C_PICO_EARBLST;
        }

        // Other aborts: bus issues, illegal start/stop, etc.
        return I2C_PICO_EBUS;
    }

    return I2C_PICO_EIO;
}

static bool valid_addr7(uint8_t addr_7bit) {
    // 7-bit address ranges 0x08~0x77 are typical; 0x00~0x07 reserved; 0x78~0x7F reserved
    // 하지만 스캐너/특수장치 고려해서 "0x00~0x7F"는 허용하되,
    // 사용자 실수 방지 차원에서 0x80 이상만 차단.
    return (addr_7bit < 0x80);
}

// ---------- public API ----------

i2c_pico_status_t i2c_pico_init(i2c_pico_t *ctx, const i2c_pico_config_t *cfg) {
    if (!ctx || !cfg) return I2C_PICO_EINVAL;
    if (!cfg->instance) return I2C_PICO_EINVAL;
    if (cfg->baudrate_hz == 0) return I2C_PICO_EINVAL;
    if (cfg->timeout_us == 0) return I2C_PICO_EINVAL;

    ctx->instance = cfg->instance;
    ctx->timeout_us = cfg->timeout_us;
    ctx->is_initialized = false;
    memset(&ctx->last_diag, 0, sizeof(ctx->last_diag));

    // Init controller
    i2c_init(ctx->instance, cfg->baudrate_hz);

    // Configure GPIO
    gpio_set_function(cfg->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(cfg->scl_pin, GPIO_FUNC_I2C);

    if (cfg->enable_pullups) {
        gpio_pull_up(cfg->sda_pin);
        gpio_pull_up(cfg->scl_pin);
    } else {
        // Leave as-is (external pull-ups expected)
    }

    ctx->is_initialized = true;
    return I2C_PICO_OK;
}

void i2c_pico_deinit(i2c_pico_t *ctx) {
    if (!ctx) return;
    if (ctx->instance) {
        i2c_deinit(ctx->instance);
    }
    ctx->instance = NULL;
    ctx->timeout_us = 0;
    ctx->is_initialized = false;
    memset(&ctx->last_diag, 0, sizeof(ctx->last_diag));
}

i2c_pico_status_t i2c_pico_write(i2c_pico_t *ctx,
                                 uint8_t addr_7bit,
                                 const uint8_t *data,
                                 size_t len,
                                 bool nostop) {
    if (!ctx || !ctx->is_initialized || !ctx->instance) return I2C_PICO_ESTATE;
    if (!valid_addr7(addr_7bit)) return I2C_PICO_EINVAL;
    if (len > 0 && data == NULL) return I2C_PICO_EINVAL;

    diag_begin(ctx, addr_7bit, len, 0, nostop);

    int ret = i2c_write_timeout_us(ctx->instance, addr_7bit, data, len, nostop, ctx->timeout_us);
    ctx->last_diag.pico_result = ret;

    // For error classification
    diag_capture_abort_source(ctx);

    if (ret >= 0) {
        ctx->last_diag.write_completed = (size_t)ret;
        // If partial write occurred without error code, treat as I/O error
        if ((size_t)ret != len) return I2C_PICO_EIO;
        return I2C_PICO_OK;
    }

    // error
    ctx->last_diag.write_completed = 0;
    return map_pico_result_to_status(ctx, ret);
}

i2c_pico_status_t i2c_pico_read(i2c_pico_t *ctx,
                                uint8_t addr_7bit,
                                uint8_t *data,
                                size_t len,
                                bool nostop) {
    if (!ctx || !ctx->is_initialized || !ctx->instance) return I2C_PICO_ESTATE;
    if (!valid_addr7(addr_7bit)) return I2C_PICO_EINVAL;
    if (len > 0 && data == NULL) return I2C_PICO_EINVAL;

    diag_begin(ctx, addr_7bit, 0, len, nostop);

    int ret = i2c_read_timeout_us(ctx->instance, addr_7bit, data, len, nostop, ctx->timeout_us);
    ctx->last_diag.pico_result = ret;

    diag_capture_abort_source(ctx);

    if (ret >= 0) {
        ctx->last_diag.read_completed = (size_t)ret;
        if ((size_t)ret != len) return I2C_PICO_EIO;
        return I2C_PICO_OK;
    }

    ctx->last_diag.read_completed = 0;
    return map_pico_result_to_status(ctx, ret);
}

i2c_pico_status_t i2c_pico_write_read(i2c_pico_t *ctx,
                                      uint8_t addr_7bit,
                                      const uint8_t *write_data,
                                      size_t write_len,
                                      uint8_t *read_data,
                                      size_t read_len) {
    if (!ctx || !ctx->is_initialized || !ctx->instance) return I2C_PICO_ESTATE;
    if (!valid_addr7(addr_7bit)) return I2C_PICO_EINVAL;
    if (write_len > 0 && write_data == NULL) return I2C_PICO_EINVAL;
    if (read_len > 0 && read_data == NULL) return I2C_PICO_EINVAL;

    // One combined diagnostics record
    diag_begin(ctx, addr_7bit, write_len, read_len, true);

    // 1) write with repeated-start (nostop=true) if write_len>0
    if (write_len > 0) {
        int wret = i2c_write_timeout_us(ctx->instance, addr_7bit, write_data, write_len, true, ctx->timeout_us);
        ctx->last_diag.pico_result = wret;
        diag_capture_abort_source(ctx);

        if (wret < 0) {
            ctx->last_diag.write_completed = 0;
            ctx->last_diag.read_completed = 0;
            return map_pico_result_to_status(ctx, wret);
        }

        ctx->last_diag.write_completed = (size_t)wret;
        if ((size_t)wret != write_len) {
            // Partial write in a combined op => treat as I/O error
            return I2C_PICO_EIO;
        }
    }

    // 2) read with stop (nostop=false) if read_len>0
    if (read_len > 0) {
        // update nostop to false for the second leg (for transparency)
        ctx->last_diag.nostop = false;

        int rret = i2c_read_timeout_us(ctx->instance, addr_7bit, read_data, read_len, false, ctx->timeout_us);
        ctx->last_diag.pico_result = rret;
        diag_capture_abort_source(ctx);

        if (rret < 0) {
            ctx->last_diag.read_completed = 0;
            return map_pico_result_to_status(ctx, rret);
        }

        ctx->last_diag.read_completed = (size_t)rret;
        if ((size_t)rret != read_len) {
            return I2C_PICO_EIO;
        }
    }

    return I2C_PICO_OK;
}

i2c_pico_status_t i2c_pico_probe(i2c_pico_t *ctx, uint8_t addr_7bit) {
    if (!ctx || !ctx->is_initialized || !ctx->instance) return I2C_PICO_ESTATE;
    if (!valid_addr7(addr_7bit)) return I2C_PICO_EINVAL;

    // Many scanners do 1-byte read; devices that NACK will fail without changing state much.
    uint8_t dummy = 0;
    return i2c_pico_read(ctx, addr_7bit, &dummy, 1, false);
}

const i2c_pico_diagnostics_t *i2c_pico_last_diagnostics(const i2c_pico_t *ctx) {
    if (!ctx) return NULL;
    return &ctx->last_diag;
}

const char *i2c_pico_status_str(i2c_pico_status_t st) {
    switch (st) {
    case I2C_PICO_OK:       return "I2C_PICO_OK";
    case I2C_PICO_EINVAL:   return "I2C_PICO_EINVAL";
    case I2C_PICO_ESTATE:   return "I2C_PICO_ESTATE";
    case I2C_PICO_ETIMEOUT: return "I2C_PICO_ETIMEOUT";
    case I2C_PICO_ENOACK:   return "I2C_PICO_ENOACK";
    case I2C_PICO_EARBLST:  return "I2C_PICO_EARBLST";
    case I2C_PICO_EBUS:     return "I2C_PICO_EBUS";
    case I2C_PICO_EIO:      return "I2C_PICO_EIO";
    default:                return "I2C_PICO_UNKNOWN";
    }
}
