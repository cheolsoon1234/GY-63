// FILE: src/core/udp_tlm.h
#ifndef __UDP_TLM_H__
#define __UDP_TLM_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// payload builder: out에 메시지 써서 길이 리턴 (0이면 skip)
typedef size_t (*telemetry_build_fn)(char *out, size_t out_sz, uint64_t now_ms, void *user_ctx);

bool udp_telemetry_run(telemetry_build_fn build_fn, void *user_ctx);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __UDP_TLM_H__ */
