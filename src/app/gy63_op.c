// FILE: src/app/gy63_op.c
#include "gy63_op.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "gy63_config.h"
#include "ms5611.h"

// ---- internal helpers (file-local) ----
static void fatal_i2c(const char *tag, i2c_pico_status_t st) {
    printf("%s failed: %s (%d)\n", tag, i2c_pico_status_str(st), (int)st);
    while (true) tight_loop_contents();
}

static void fatal_ms(const char *tag, ms5611_status_t st) {
    printf("%s failed: %s (%ld)\n", tag, ms5611_status_str(st), (long)st);
    while (true) tight_loop_contents();
}

void gy63_init(gy63_ctx_t *ctx) {
    if (!ctx) return;

    i2c_pico_status_t bst = gy63_bsp_init();
    if (bst != I2C_PICO_OK) {
        fatal_i2c("gy63_bsp_init", bst);
    }

    i2c_pico_t *bus = gy63_bsp_i2c();
    uint8_t addr = gy63_bsp_addr7();

    ms5611_status_t st = ms5611_init(&ctx->dev, bus, addr);
    if (st != MS5611_OK) {
        fatal_ms("ms5611_init", st);
    }

    ms5611_config_default(&ctx->cfg);
    ctx->cfg.osr = MS5611_OSR_4096;
}

ms5611_status_t gy63_read(gy63_ctx_t *ctx, int32_t *t_x100, uint32_t *p_pa) {
    if (!ctx || !t_x100 || !p_pa) return MS5611_EINVAL;
    return ms5611_read(&ctx->dev, &ctx->cfg, t_x100, p_pa);
}

void gy63_operation(gy63_ctx_t *ctx) {
    if (!ctx) return;

    int32_t  t_x100 = 0;
    uint32_t p_pa   = 0;

    ms5611_status_t st = gy63_read(ctx, &t_x100, &p_pa);
    if (st != MS5611_OK) {
        printf("ms5611_read failed: %s (%ld)\n", ms5611_status_str(st), (long)st);
        return;
    }

    printf("T=%.2f C, P=%u Pa\n", (double)t_x100 / 100.0, (unsigned)p_pa);
}
