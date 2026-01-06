// FILE: src/platform/hal/i2c_pico.c
#include "i2c_pico.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/error.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"          // i2c_*(), i2c_get_hw()
#include "hardware/structs/i2c.h"
#include "hardware/regs/i2c.h"     // I2C_IC_TX_ABRT_SOURCE_* bits

// ---------- internal helpers ----------

static bool valid_addr7(uint8_t addr_7bit) {
    // 7-bit address ranges 0x08~0x77 are typical; 0x00~0x07 reserved; 0x78~0x7F reserved
    // 0x80 이상 차단.
    return (addr_7bit < 0x80);
}

static i2c_pico_status_t validate_ready(i2c_pico_t *ctx) {
    if (!ctx || !ctx->is_initialized || !ctx->instance) return I2C_PICO_ESTATE;
    return I2C_PICO_OK;
}

static i2c_pico_status_t validate_addr(uint8_t addr_7bit) {
    if (!valid_addr7(addr_7bit)) return I2C_PICO_EINVAL;
    return I2C_PICO_OK;
}

static i2c_pico_status_t validate_buffer(const void *buf, size_t len) {
    if (len > 0 && buf == NULL) return I2C_PICO_EINVAL;
    return I2C_PICO_OK;
}

static void diag_begin(i2c_pico_t *ctx,
                       uint8_t addr_7bit,
                       size_t wreq,
                       size_t rreq,
                       bool nostop) {
    memset(&ctx->last_diag, 0, sizeof(ctx->last_diag));
    ctx->last_diag.address_7bit          = addr_7bit;
    ctx->last_diag.write_requested       = wreq;
    ctx->last_diag.read_requested        = rreq;
    ctx->last_diag.nostop                = nostop;
    ctx->last_diag.pico_result           = 0;
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

// 전송 결과를 공통 처리:
// - pico_result 저장
// - abort source 캡처
// - completed 기록
// - partial 전송이면 EIO
// - 에러면 map
static i2c_pico_status_t transfer_finish(i2c_pico_t *ctx,
                                        int pico_ret,
                                        size_t requested,
                                        size_t *completed_out) {
    ctx->last_diag.pico_result = pico_ret;

    // For error classification (OR accumulate across legs)
    diag_capture_abort_source(ctx);

    if (pico_ret >= 0) {
        size_t done = (size_t)pico_ret;
        if (completed_out) *completed_out = done;

        // partial이면 I/O error로 취급.
        if (done != requested) return I2C_PICO_EIO;
        return I2C_PICO_OK;
    }

    if (completed_out) *completed_out = 0;
    return map_pico_result_to_status(ctx, pico_ret);
}

static i2c_pico_status_t transfer_write_leg(i2c_pico_t *ctx,
                                            uint8_t addr_7bit,
                                            const uint8_t *data,
                                            size_t len,
                                            bool nostop,
                                            size_t *completed_out) {
    int ret = i2c_write_timeout_us(ctx->instance, addr_7bit, data, len, nostop, ctx->timeout_us);
    return transfer_finish(ctx, ret, len, completed_out);
}

static i2c_pico_status_t transfer_read_leg(i2c_pico_t *ctx,
                                           uint8_t addr_7bit,
                                           uint8_t *data,
                                           size_t len,
                                           bool nostop,
                                           size_t *completed_out) {
    int ret = i2c_read_timeout_us(ctx->instance, addr_7bit, data, len, nostop, ctx->timeout_us);
    return transfer_finish(ctx, ret, len, completed_out);
}

static i2c_pico_status_t init_validate_args(const i2c_pico_config_t *cfg) {
    if (!cfg) return I2C_PICO_EINVAL;
    if (!cfg->instance) return I2C_PICO_EINVAL;
    if (cfg->baudrate_hz == 0) return I2C_PICO_EINVAL;
    if (cfg->timeout_us == 0) return I2C_PICO_EINVAL;
    return I2C_PICO_OK;
}

static void init_configure_controller(i2c_pico_t *ctx, const i2c_pico_config_t *cfg) {
    ctx->instance = cfg->instance;
    ctx->timeout_us = cfg->timeout_us;

    // Init controller
    i2c_init(ctx->instance, cfg->baudrate_hz);
}

static void init_configure_gpio(const i2c_pico_config_t *cfg) {
    gpio_set_function(cfg->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(cfg->scl_pin, GPIO_FUNC_I2C);

    if (cfg->enable_pullups) {
        gpio_pull_up(cfg->sda_pin);
        gpio_pull_up(cfg->scl_pin);
    } else {
        // Leave as-is (external pull-ups expected)
    }
}

// ---------- public API ----------

i2c_pico_status_t i2c_pico_init(i2c_pico_t *ctx, const i2c_pico_config_t *cfg) {
    if (!ctx) return I2C_PICO_EINVAL;

    i2c_pico_status_t vst = init_validate_args(cfg);
    if (vst != I2C_PICO_OK) return vst;

    ctx->is_initialized = false;
    memset(&ctx->last_diag, 0, sizeof(ctx->last_diag));

    init_configure_controller(ctx, cfg);
    init_configure_gpio(cfg);

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
    i2c_pico_status_t st;

    st = validate_ready(ctx);
    if (st != I2C_PICO_OK) return st;

    st = validate_addr(addr_7bit);
    if (st != I2C_PICO_OK) return st;

    st = validate_buffer(data, len);
    if (st != I2C_PICO_OK) return st;

    diag_begin(ctx, addr_7bit, len, 0, nostop);

    return transfer_write_leg(ctx, addr_7bit, data, len, nostop, &ctx->last_diag.write_completed);
}

i2c_pico_status_t i2c_pico_read(i2c_pico_t *ctx,
                                uint8_t addr_7bit,
                                uint8_t *data,
                                size_t len,
                                bool nostop) {
    i2c_pico_status_t st;

    st = validate_ready(ctx);
    if (st != I2C_PICO_OK) return st;

    st = validate_addr(addr_7bit);
    if (st != I2C_PICO_OK) return st;

    st = validate_buffer(data, len);
    if (st != I2C_PICO_OK) return st;

    diag_begin(ctx, addr_7bit, 0, len, nostop);

    return transfer_read_leg(ctx, addr_7bit, data, len, nostop, &ctx->last_diag.read_completed);
}

i2c_pico_status_t i2c_pico_write_read(i2c_pico_t *ctx,
                                      uint8_t addr_7bit,
                                      const uint8_t *write_data,
                                      size_t write_len,
                                      uint8_t *read_data,
                                      size_t read_len) {
    i2c_pico_status_t st;

    st = validate_ready(ctx);
    if (st != I2C_PICO_OK) return st;

    st = validate_addr(addr_7bit);
    if (st != I2C_PICO_OK) return st;

    st = validate_buffer(write_data, write_len);
    if (st != I2C_PICO_OK) return st;

    st = validate_buffer(read_data, read_len);
    if (st != I2C_PICO_OK) return st;

    // One combined diagnostics record
    diag_begin(ctx, addr_7bit, write_len, read_len, true);

    // 1) write with repeated-start (nostop=true)
    if (write_len > 0) {
        ctx->last_diag.nostop = true;
        st = transfer_write_leg(ctx, addr_7bit, write_data, write_len, true, &ctx->last_diag.write_completed);
        if (st != I2C_PICO_OK) {
            // 실패 시 read_completed는 0 유지
            ctx->last_diag.read_completed = 0;
            return st;
        }
    }

    // 2) read with stop (nostop=false)
    if (read_len > 0) {
        ctx->last_diag.nostop = false;
        st = transfer_read_leg(ctx, addr_7bit, read_data, read_len, false, &ctx->last_diag.read_completed);
        if (st != I2C_PICO_OK) return st;
    }

    return I2C_PICO_OK;
}

i2c_pico_status_t i2c_pico_probe(i2c_pico_t *ctx, uint8_t addr_7bit) {
    i2c_pico_status_t st;

    st = validate_ready(ctx);
    if (st != I2C_PICO_OK) return st;

    st = validate_addr(addr_7bit);
    if (st != I2C_PICO_OK) return st;

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
