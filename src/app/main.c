// FILE: src/app/main.c
#include <stdio.h>
#include "pico/stdlib.h"

#include "gy63_op.h"

#include "platform_core.h"
#include "net_wifi.h"
#include "net_udp.h"
#include "net_config.h"

int main() {
    stdio_init_all();
    sleep_ms(10000);

    // 1) Wi-Fi 플랫폼 초기화 (cyw43 init + STA)
    if (!platform_init()) {
        printf("platform_init failed\n");
        while (true) tight_loop_contents();
    }

    // 2) Wi-Fi 연결
    printf("Connecting Wi-Fi...\n");
    if (!net_wifi_connect_wpa2(CFG_WIFI_SSID, CFG_WIFI_PASSWORD, CFG_WIFI_TIMEOUT_MS)) {
        printf("Wi-Fi connect failed\n");
        while (true) tight_loop_contents();
    }
    printf("Wi-Fi connected\n");

    // 3) UDP 오픈
    net_udp_client_t *udp = NULL;
    if (!net_udp_open(&udp, CFG_UDP_DST_IP, (uint16_t)CFG_UDP_DST_PORT)) {
        printf("net_udp_open failed (%s:%u)\n", CFG_UDP_DST_IP, (unsigned)CFG_UDP_DST_PORT);
        while (true) tight_loop_contents();
    }
    printf("UDP ready -> %s:%u\n", CFG_UDP_DST_IP, (unsigned)CFG_UDP_DST_PORT);

    // 4) 센서 init
    gy63_ctx_t ctx;
    gy63_init(&ctx);

    // 5) 메인 루프: 1회 측정 -> UDP 송신
    while (true) {
        int32_t  t_x100 = 0;
        uint32_t p_pa   = 0;

        ms5611_status_t st = gy63_read(&ctx, &t_x100, &p_pa);
        if (st != MS5611_OK) {
            printf("gy63_read failed: %s (%ld)\n", ms5611_status_str(st), (long)st);
        } else {
            // (옵션) 로컬 로그
            printf("T=%.2f C, P=%u Pa\n", (double)t_x100 / 100.0, (unsigned)p_pa);

            // UDP payload (텍스트)
            const uint64_t ms = platform_millis();
            char msg[128];
            int n = snprintf(msg, sizeof(msg),
                             "ms=%llu,t_x100=%ld,p_pa=%u\n",
                             (unsigned long long)ms,
                             (long)t_x100,
                             (unsigned)p_pa);

            if (n > 0) {
                (void)net_udp_send(udp, msg, (size_t)n);
            }
        }

        sleep_ms(100);
    }
}
