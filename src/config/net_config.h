#ifndef __NET_CONFIG_H__
#define __NET_CONFIG_H__

#include "platform_core.h" // config는 platform에만 의존

#define CFG_WIFI_SSID        "hotspot"
#define CFG_WIFI_PASSWORD    "hotspot1111"
#define CFG_WIFI_TIMEOUT_MS  (30000u)

#define CFG_UDP_DST_IP       "192.168.144.104"
#define CFG_UDP_DST_PORT     (5005u)

#define CFG_SEND_PERIOD_MS   (200u)

#endif /* __NET_CONFIG_H__ */ 