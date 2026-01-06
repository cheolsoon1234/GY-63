// FILE: src/platform/net/net_udp.h
#ifndef __NET_UDP_H__
#define __NET_UDP_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// lwIP 타입 노출 방지: opaque handle
typedef struct net_udp_client net_udp_client_t;

bool net_udp_open(net_udp_client_t **out, const char *dst_ip, uint16_t dst_port);
bool net_udp_send(net_udp_client_t *c, const void *data, size_t len);
void net_udp_close(net_udp_client_t *c);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __NET_UDP_H__
