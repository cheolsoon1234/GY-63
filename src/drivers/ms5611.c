// FILE: src/drivers/ms5611.c
#include "ms5611.h"

#include <string.h>
#include "pico/stdlib.h"

// MS5611 commands (datasheet / command table)
#define MS5611_CMD_RESET    0x1E
#define MS5611_CMD_ADC_READ 0x00
#define MS5611_CMD_PROM_RD  0xA0 // 0xA0..0xAE step 2

#define MS5611_CMD_CONV_D1_BASE 0x40 // pressure
#define MS5611_CMD_CONV_D2_BASE 0x50 // temperature

#define MS5611_CONV_MARGIN_US 200u

// ---------- internal helpers ----------

static ms5611_status_t ms5611_from_i2c_status(i2c_pico_status_t st) {
    return (ms5611_status_t)st; // project convention: i2c error codes passthrough
}

static ms5611_status_t validate_init_args(const ms5611_t *dev, const i2c_pico_t *i2c, uint8_t addr7) {
    if (!dev || !i2c) return MS5611_EINVAL;
    if (addr7 >= 0x80) return MS5611_EINVAL;
    return MS5611_OK;
}

static ms5611_status_t validate_read_args(const ms5611_t *dev,
                                         const ms5611_config_t *cfg,
                                         int32_t *temp_c_x100,
                                         uint32_t *press_pa) {
    if (!dev || !dev->i2c || !cfg || !temp_c_x100 || !press_pa) return MS5611_EINVAL;
    if (!dev->initialized) return MS5611_ESTATE;
    return MS5611_OK;
}

static ms5611_status_t i2c_cmd_write(i2c_pico_t *i2c, uint8_t addr7, uint8_t cmd) {
    if (!i2c) return MS5611_EINVAL;
    i2c_pico_status_t st = i2c_pico_write(i2c, addr7, &cmd, 1, false);
    return ms5611_from_i2c_status(st);
}

static ms5611_status_t i2c_cmd_read(i2c_pico_t *i2c, uint8_t addr7, uint8_t cmd, uint8_t *buf, size_t len) {
    if (!i2c || (len && !buf)) return MS5611_EINVAL;
    i2c_pico_status_t st = i2c_pico_write_read(i2c, addr7, &cmd, 1, buf, len);
    return ms5611_from_i2c_status(st);
}

// datasheet conversion time "max" (us) for each OSR
// (OSR 256: 0.60ms, 512: 1.17ms, 1024: 2.28ms, 2048: 4.54ms, 4096: 9.04ms)
static uint32_t conv_time_us_max(ms5611_osr_t osr) {
    switch (osr) {
    case MS5611_OSR_256:  return 600;   // 0.60ms
    case MS5611_OSR_512:  return 1170;  // 1.17ms
    case MS5611_OSR_1024: return 2280;  // 2.28ms
    case MS5611_OSR_2048: return 4540;  // 4.54ms
    case MS5611_OSR_4096: return 9040;  // 9.04ms
    default:              return 9040;
    }
}

// command offset mapping per OSR (0x40/0x42/0x44/0x46/0x48, 0x50/0x52/0x54/0x56/0x58)
static uint8_t osr_cmd_offset(ms5611_osr_t osr) {
    switch (osr) {
    case MS5611_OSR_256:  return 0x00;
    case MS5611_OSR_512:  return 0x02;
    case MS5611_OSR_1024: return 0x04;
    case MS5611_OSR_2048: return 0x06;
    case MS5611_OSR_4096: return 0x08;
    default:              return 0x08;
    }
}

static uint8_t build_conv_cmd(bool is_temp, ms5611_osr_t osr) {
    const uint8_t base = is_temp ? MS5611_CMD_CONV_D2_BASE : MS5611_CMD_CONV_D1_BASE;
    return (uint8_t)(base + osr_cmd_offset(osr));
}

static void wait_conversion_done(ms5611_osr_t osr) {
    sleep_us(conv_time_us_max(osr) + MS5611_CONV_MARGIN_US);
}

static ms5611_status_t start_conversion(ms5611_t *dev, bool is_temp, ms5611_osr_t osr) {
    if (!dev || !dev->i2c) return MS5611_EINVAL;
    const uint8_t cmd = build_conv_cmd(is_temp, osr);
    return i2c_cmd_write(dev->i2c, dev->addr7, cmd);
}

static ms5611_status_t read_adc24(ms5611_t *dev, uint32_t *out_adc) {
    if (!dev || !dev->i2c || !out_adc) return MS5611_EINVAL;

    uint8_t buf[3] = {0};
    ms5611_status_t st = i2c_cmd_read(dev->i2c, dev->addr7, MS5611_CMD_ADC_READ, buf, 3);
    if (st != MS5611_OK) return st;

    *out_adc = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    return MS5611_OK;
}

static ms5611_status_t convert_and_read(ms5611_t *dev, bool is_temp, ms5611_osr_t osr, uint32_t *out_adc) {
    if (!out_adc) return MS5611_EINVAL;

    ms5611_status_t st = start_conversion(dev, is_temp, osr);
    if (st != MS5611_OK) return st;

    wait_conversion_done(osr);

    return read_adc24(dev, out_adc);
}

static ms5611_status_t read_prom_word(ms5611_t *dev, int idx, uint16_t *out_word) {
    if (!dev || !dev->i2c || !out_word) return MS5611_EINVAL;
    if (idx < 0 || idx > 7) return MS5611_EINVAL;

    const uint8_t cmd = (uint8_t)(MS5611_CMD_PROM_RD + (idx * 2));
    uint8_t buf[2] = {0};

    ms5611_status_t st = i2c_cmd_read(dev->i2c, dev->addr7, cmd, buf, 2);
    if (st != MS5611_OK) return st;

    *out_word = ((uint16_t)buf[0] << 8) | buf[1];
    return MS5611_OK;
}

static ms5611_status_t prom_sanity_check(const uint16_t prom[8]) {
    if (!prom) return MS5611_EINVAL;

    bool all0 = true, allf = true;
    for (int i = 0; i < 8; i++) {
        if (prom[i] != 0x0000) all0 = false;
        if (prom[i] != 0xFFFF) allf = false;
    }
    if (all0 || allf) return MS5611_EPROM;
    return MS5611_OK;
}

// CRC4 (MS5611 datasheet algorithm widely used)
static uint8_t crc4_calc(const uint16_t prom[8]) {
    uint16_t n_prom[8];
    memcpy(n_prom, prom, sizeof(n_prom));

    uint16_t n_rem = 0;
    n_prom[7] &= 0xFF00; // clear CRC nibble

    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) n_rem ^= (uint16_t)(n_prom[cnt >> 1] & 0x00FF);
        else         n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);

        for (int n_bit = 0; n_bit < 8; n_bit++) {
            if (n_rem & 0x8000) n_rem = (uint16_t)((n_rem << 1) ^ 0x3000);
            else                n_rem = (uint16_t)(n_rem << 1);
        }
    }
    n_rem = (n_rem >> 12) & 0x000F;
    return (uint8_t)n_rem;
}

static ms5611_status_t prom_crc_check(const uint16_t prom[8]) {
    if (!prom) return MS5611_EINVAL;

    const uint8_t crc_read = (uint8_t)(prom[7] & 0x000F);
    const uint8_t crc_calc = crc4_calc(prom);
    if (crc_calc != crc_read) return MS5611_ECRC;

    return MS5611_OK;
}

static void store_prom(ms5611_t *dev, const uint16_t prom[8]) {
    // memcpy 반환값 체크는 일반적으로 의미가 없어(항상 dest 반환).
    memcpy(dev->prom, prom, sizeof(dev->prom));
}

typedef struct {
    int64_t C1, C2, C3, C4, C5, C6;
} ms5611_coeffs_t;

static void load_coeffs(const ms5611_t *dev, ms5611_coeffs_t *c) {
    c->C1 = dev->prom[1];
    c->C2 = dev->prom[2];
    c->C3 = dev->prom[3];
    c->C4 = dev->prom[4];
    c->C5 = dev->prom[5];
    c->C6 = dev->prom[6];
}

static ms5611_status_t compensate_and_check(const ms5611_coeffs_t *c,
                                           uint32_t D1,
                                           uint32_t D2,
                                           int32_t *out_temp_c_x100,
                                           uint32_t *out_press_pa) {
    if (!c || !out_temp_c_x100 || !out_press_pa) return MS5611_EINVAL;

    // ---- compensation (datasheet) ----
    // dT = D2 - C5*2^8
    int64_t dT = (int64_t)D2 - (c->C5 << 8);

    // TEMP = 2000 + dT*C6 / 2^23  (0.01°C)
    int64_t TEMP = 2000 + ((dT * c->C6) >> 23);

    // OFF  = C2*2^16 + (C4*dT)/2^7
    int64_t OFF  = (c->C2 << 16) + ((c->C4 * dT) >> 7);

    // SENS = C1*2^15 + (C3*dT)/2^8
    int64_t SENS = (c->C1 << 15) + ((c->C3 * dT) >> 8);

    // Second-order temperature compensation (low temp)
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        T2 = (dT * dT) >> 31;

        int64_t t = TEMP - 2000;
        OFF2  = (5 * t * t) >> 1;
        SENS2 = (5 * t * t) >> 2;

        if (TEMP < -1500) {
            int64_t t2 = TEMP + 1500;
            OFF2  += 7 * t2 * t2;
            SENS2 += (11 * t2 * t2) >> 1;
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    // P = (D1*SENS/2^21 - OFF)/2^15
    int64_t P = (((int64_t)D1 * SENS) >> 21) - OFF;
    P = P >> 15;

    // MS5611 datasheet example output pressure unit is 0.01 mbar.
    // 0.01 mbar == 1 Pa 이므로, 여기서는 Pa로 그대로 해석 가능.
    if (P < 0 || P > 200000) { // 느슨한 가드: (정상 대기압 부근 기준으로 충분히 넓게)
        return MS5611_ERANGE;
    }

    *out_temp_c_x100 = (int32_t)TEMP;
    *out_press_pa    = (uint32_t)P;
    return MS5611_OK;
}

// ---------- public API ----------

const char *ms5611_status_str(ms5611_status_t st) {
    switch (st) {
    case MS5611_OK:     return "MS5611_OK";
    case MS5611_EINVAL: return "MS5611_EINVAL";
    case MS5611_ESTATE: return "MS5611_ESTATE";
    case MS5611_EPROM:  return "MS5611_EPROM";
    case MS5611_ECRC:   return "MS5611_ECRC";
    case MS5611_ERANGE: return "MS5611_ERANGE";
    default:
        // likely I2C layer error code
        return i2c_pico_status_str((i2c_pico_status_t)st);
    }
}

void ms5611_config_default(ms5611_config_t *cfg) {
    if (!cfg) return;
    cfg->osr = MS5611_OSR_4096; // 가장 고해상도(시간은 가장 김)
}

ms5611_status_t ms5611_reset(ms5611_t *dev) {
    if (!dev || !dev->i2c) return MS5611_EINVAL;

    ms5611_status_t st = i2c_cmd_write(dev->i2c, dev->addr7, MS5611_CMD_RESET);
    if (st != MS5611_OK) return st;

    // PROM reload typ ~2.8ms (datasheet)
    sleep_ms(3);
    return MS5611_OK;
}

ms5611_status_t ms5611_read_prom(ms5611_t *dev, uint16_t out_prom[8]) {
    if (!dev || !dev->i2c || !out_prom) return MS5611_EINVAL;

    // 1) read all words (return-value checks mandatory)
    for (int i = 0; i < 8; i++) {
        ms5611_status_t st = read_prom_word(dev, i, &out_prom[i]);
        if (st != MS5611_OK) return st;
    }

    // 2) sanity + CRC
    ms5611_status_t st = prom_sanity_check(out_prom);
    if (st != MS5611_OK) return st;

    st = prom_crc_check(out_prom);
    if (st != MS5611_OK) return st;

    // 3) store
    store_prom(dev, out_prom);
    return MS5611_OK;
}

ms5611_status_t ms5611_init(ms5611_t *dev, i2c_pico_t *i2c, uint8_t addr7) {
    ms5611_status_t st = validate_init_args(dev, i2c, addr7);
    if (st != MS5611_OK) return st;

    memset(dev, 0, sizeof(*dev));
    dev->i2c = i2c;
    dev->addr7 = addr7;

    st = ms5611_reset(dev);
    if (st != MS5611_OK) return st;

    uint16_t prom[8] = {0};
    st = ms5611_read_prom(dev, prom);
    if (st != MS5611_OK) return st;

    dev->initialized = true;
    return MS5611_OK;
}

ms5611_status_t ms5611_read(ms5611_t *dev,
                            const ms5611_config_t *cfg,
                            int32_t *temp_c_x100,
                            uint32_t *press_pa) {
    ms5611_status_t st = validate_read_args(dev, cfg, temp_c_x100, press_pa);
    if (st != MS5611_OK) return st;

    // 1) conversions (return-value checks mandatory)
    uint32_t D2 = 0; // temperature ADC
    st = convert_and_read(dev, true, cfg->osr, &D2);
    if (st != MS5611_OK) return st;

    uint32_t D1 = 0; // pressure ADC
    st = convert_and_read(dev, false, cfg->osr, &D1);
    if (st != MS5611_OK) return st;

    // 2) compensation
    ms5611_coeffs_t c;
    load_coeffs(dev, &c);

    return compensate_and_check(&c, D1, D2, temp_c_x100, press_pa);
}
