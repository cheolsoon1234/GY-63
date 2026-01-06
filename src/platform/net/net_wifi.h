// FILE: src/platform/net/net_wifi.h
#ifndef __NET_WIFI_H__
#define __NET_WIFI_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

bool net_wifi_connect_wpa2(const char *ssid, const char *password, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __NET_WIFI_H__
