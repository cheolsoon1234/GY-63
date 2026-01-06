// FILE: src/platform/net/net_udp.c
#include "net_udp.h"

#include <string.h>
#include <stdlib.h>

#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

struct net_udp_client {
    struct udp_pcb *pcb;
};

bool net_udp_open(net_udp_client_t **out, const char *dst_ip, uint16_t dst_port) {
    if (!out || !dst_ip || dst_port == 0) return false;
    *out = NULL;

    ip_addr_t addr;
    if (!ipaddr_aton(dst_ip, &addr)) return false;

    net_udp_client_t *c = (net_udp_client_t *)calloc(1, sizeof(*c));
    if (!c) return false;

    c->pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (!c->pcb) { free(c); return false; }

    cyw43_arch_lwip_begin();
    err_t e = udp_connect(c->pcb, &addr, dst_port);
    cyw43_arch_lwip_end();

    if (e != ERR_OK) {
        udp_remove(c->pcb);
        free(c);
        return false;
    }

    *out = c;
    return true;
}

bool net_udp_send(net_udp_client_t *c, const void *data, size_t len) {
    if (!c || !c->pcb || !data || len == 0) return false;
    if (len > 0xFFFF) return false;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (!p) return false;

    memcpy(p->payload, data, len);

    cyw43_arch_lwip_begin();
    err_t e = udp_send(c->pcb, p);
    cyw43_arch_lwip_end();

    pbuf_free(p);
    return (e == ERR_OK);
}

void net_udp_close(net_udp_client_t *c) {
    if (!c) return;
    if (c->pcb) {
        udp_remove(c->pcb);
        c->pcb = NULL;
    }
    free(c);
}
