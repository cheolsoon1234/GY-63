// FILE: src/app/main.c
#include <stdio.h>              // printf
#include "pico/stdlib.h"        // Pico SDK stdlib

#include "bsp/gy63_config.h"    // gy63 BSP
#include "drivers/ms5611.h"     // MS5611 driver

int main() {
    stdio_init_all();
    sleep_ms(10000);

    i2c_pico_status_t bst = gy63_bsp_init();
    if (bst != I2C_PICO_OK) {
        printf("gy63_bsp_init failed: %s (%d)\n", i2c_pico_status_str(bst), (int)bst);
        while (true) tight_loop_contents();
    }

    i2c_pico_t *bus = gy63_bsp_i2c();
    uint8_t addr = gy63_bsp_addr7();

    ms5611_t dev;
    ms5611_status_t st = ms5611_init(&dev, bus, addr);
    if (st != MS5611_OK) {
        printf("ms5611_init failed: %s (%ld)\n", ms5611_status_str(st), (long)st);
        while (true) tight_loop_contents();
    }

    ms5611_config_t cfg;
    ms5611_config_default(&cfg);
    cfg.osr = MS5611_OSR_4096;

    while (true) {
        int32_t t_x100 = 0;
        uint32_t p_pa = 0;

        st = ms5611_read(&dev, &cfg, &t_x100, &p_pa);
        if (st != MS5611_OK) {
            printf("ms5611_read failed: %s (%ld)\n", ms5611_status_str(st), (long)st);
        } else {
            printf("T=%.2f C, P=%u Pa\n", (double)t_x100 / 100.0, (unsigned)p_pa);
        }

        sleep_ms(100);
    }
}
