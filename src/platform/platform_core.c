// FILE: src/platform/platform_core.c
#include "platform_core.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

bool platform_init(void) {
    stdio_init_all();
    if (cyw43_arch_init()) return false;

    cyw43_arch_enable_sta_mode();
    return true;
}

void platform_deinit(void) {
    cyw43_arch_deinit();
}

void platform_poll(void) {
    cyw43_arch_poll();
}

uint64_t platform_millis(void) {
    return (uint64_t)to_ms_since_boot(get_absolute_time());
}

void platform_sleep_ms(uint32_t ms) {
    sleep_ms(ms);
}

void platform_yield(void) {
    tight_loop_contents();
}
