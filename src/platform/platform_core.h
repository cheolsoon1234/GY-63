// FILE: src/platform/platform_core.h
#ifndef __PLATFORM_CORE_H__
#define __PLATFORM_CORE_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// cyw43 초기화 + STA 모드 활성화까지 포함 (Wi-Fi/UDP 공통 기반)
bool     platform_init(void);
void     platform_deinit(void);

// cyw43/lwIP poll
void     platform_poll(void);

// time & yield (app에서 pico SDK 직접 호출 막기)
uint64_t platform_millis(void);
void     platform_sleep_ms(uint32_t ms);
void     platform_yield(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __PLATFORM_CORE_H__ */
