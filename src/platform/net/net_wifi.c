// FILE: src/platform/net/net_wifi.c
#include "net_wifi.h"
#include "pico/cyw43_arch.h"

bool net_wifi_connect_wpa2(const char *ssid, const char *password, uint32_t timeout_ms) {
    if (!ssid || !password) return false;

    int st = cyw43_arch_wifi_connect_timeout_ms(
        ssid, password,
        CYW43_AUTH_WPA2_AES_PSK,
        timeout_ms
    );
    return (st == 0);
}
