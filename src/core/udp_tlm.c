// FILE: src/core/udp_tlm.c
#include "udp_tlm.h"

#include <stdio.h>

#include "platform_core.h"
#include "net_wifi.h"
#include "net_udp.h"
#include "net_config.h"

bool udp_telemetry_run(telemetry_build_fn build_fn, void *user_ctx) {
    if (!build_fn) return false;

    if (!platform_init()) return false;

    platform_sleep_ms(1500);
    printf("udp_telemetry_run start\n");

    printf("Connecting Wi-Fi...\n");
    if (!net_wifi_connect_wpa2(CFG_WIFI_SSID, CFG_WIFI_PASSWORD, CFG_WIFI_TIMEOUT_MS)) {
        printf("Wi-Fi connect failed\n");
        platform_deinit();
        return false;
    }
    printf("Wi-Fi connected\n");

    net_udp_client_t *udp = NULL;
    if (!net_udp_open(&udp, CFG_UDP_DST_IP, (uint16_t)CFG_UDP_DST_PORT)) {
        printf("UDP open failed\n");
        platform_deinit();
        return false;
    }
    printf("UDP ready -> %s:%u\n", CFG_UDP_DST_IP, (unsigned)CFG_UDP_DST_PORT);

    uint64_t next_ms = platform_millis() + (uint64_t)CFG_SEND_PERIOD_MS;

    while (true) {
        platform_poll();

        uint64_t now = platform_millis();
        if ((int64_t)(now - next_ms) >= 0) {
            next_ms += (uint64_t)CFG_SEND_PERIOD_MS;

            char msg[128];
            size_t n = build_fn(msg, sizeof(msg), now, user_ctx);
            if (n > 0) {
                (void)net_udp_send(udp, msg, n);
            }
        }

        platform_yield();
    }

    // never
    net_udp_close(udp);
    platform_deinit();
    return true;
}
