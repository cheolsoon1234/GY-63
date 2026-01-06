// FILE: src/app/gy63_op.h
#ifndef __GY63_OP_H__
#define __GY63_OP_H__

#include <stdbool.h>
#include <stdint.h>

#include "drivers/ms5611.h" // ms5611_t, ms5611_config_t

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct {
    ms5611_t dev;
    ms5611_config_t cfg;
} gy63_ctx_t;

// 1회만 호출 (BSP + MS5611 init + cfg 세팅)
void gy63_init(gy63_ctx_t *ctx);

// 1회 측정만 수행(값 반환)
ms5611_status_t gy63_read(gy63_ctx_t *ctx, int32_t *t_x100, uint32_t *p_pa);

// 측정 후 결과 출력
void gy63_operation(gy63_ctx_t *ctx);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __GY63_OP_H__