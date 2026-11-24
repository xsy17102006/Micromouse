/*
 * vl53l0x.c - Port toi gian thu vien Pololu VL53L0X sang STM32 HAL
 */
#include "vl53l0x.h"

#define SYSRANGE_START                          0x00
#define SYSTEM_SEQUENCE_CONFIG                  0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define SYSTEM_INTERRUPT_CLEAR                  0x0B
#define RESULT_INTERRUPT_STATUS                 0x13
#define RESULT_RANGE_STATUS                     0x14
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD     0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET        0x4F
#define PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define MSRC_CONFIG_CONTROL                     0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define I2C_SLAVE_DEVICE_ADDRESS                0x8A
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6
#define IDENTIFICATION_MODEL_ID                 0xC0

#define IO_TIMEOUT_MS 100

/* ---------------- I2C ---------------- */
static void wr8(vl53l0x_t *d, uint8_t reg, uint8_t v)
{
    if (HAL_I2C_Mem_Write(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 20) != HAL_OK)
        d->err = 1;
}
static void wr16(vl53l0x_t *d, uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    if (HAL_I2C_Mem_Write(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT, b, 2, 20) != HAL_OK)
        d->err = 1;
}
static void wrN(vl53l0x_t *d, uint8_t reg, uint8_t *b, uint16_t n)
{
    if (HAL_I2C_Mem_Write(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT, b, n, 20) != HAL_OK)
        d->err = 1;
}
static uint8_t rd8(vl53l0x_t *d, uint8_t reg)
{
    uint8_t v = 0;
    if (HAL_I2C_Mem_Read(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 20) != HAL_OK)
        d->err = 1;
    return v;
}
static uint16_t rd16(vl53l0x_t *d, uint8_t reg)
{
    uint8_t b[2] = { 0, 0 };
    if (HAL_I2C_Mem_Read(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT, b, 2, 20) != HAL_OK)
        d->err = 1;
    return (uint16_t)((b[0] << 8) | b[1]);
}
static void rdN(vl53l0x_t *d, uint8_t reg, uint8_t *b, uint16_t n)
{
    if (HAL_I2C_Mem_Read(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT, b, n, 20) != HAL_OK)
        d->err = 1;
}

/* ---------------- Timing helpers ---------------- */
typedef struct { uint8_t tcc, msrc, dss, pre_range, final_range; } seq_en_t;
typedef struct {
    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} seq_to_t;

static uint32_t macro_period_ns(uint16_t vcsel) { return ((2304UL * vcsel * 1655UL) + 500) / 1000; }
static uint32_t mclks_to_us(uint16_t mclks, uint16_t vcsel)
{
    uint32_t mp = macro_period_ns(vcsel);
    return ((mclks * mp) + 500) / 1000;
}
static uint32_t us_to_mclks(uint32_t us, uint16_t vcsel)
{
    uint32_t mp = macro_period_ns(vcsel);
    return ((us * 1000) + (mp / 2)) / mp;
}
static uint16_t decode_timeout(uint16_t v) { return (uint16_t)(((v & 0x00FF) << ((v & 0xFF00) >> 8)) + 1); }
static uint16_t encode_timeout(uint32_t mclks)
{
    if (mclks == 0) return 0;
    uint32_t ls = mclks - 1;
    uint16_t ms = 0;
    while ((ls & 0xFFFFFF00UL) > 0) { ls >>= 1; ms++; }
    return (uint16_t)((ms << 8) | (ls & 0xFF));
}

static void get_seq_en(vl53l0x_t *d, seq_en_t *e)
{
    uint8_t s = rd8(d, SYSTEM_SEQUENCE_CONFIG);
    e->tcc = (s >> 4) & 1; e->dss = (s >> 3) & 1; e->msrc = (s >> 2) & 1;
    e->pre_range = (s >> 6) & 1; e->final_range = (s >> 7) & 1;
}
static void get_seq_to(vl53l0x_t *d, const seq_en_t *e, seq_to_t *t)
{
    t->pre_range_vcsel_pclks = (uint16_t)((rd8(d, PRE_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1);
    t->msrc_dss_tcc_mclks = (uint16_t)(rd8(d, MSRC_CONFIG_TIMEOUT_MACROP) + 1);
    t->msrc_dss_tcc_us = mclks_to_us(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_pclks);
    t->pre_range_mclks = decode_timeout(rd16(d, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t->pre_range_us = mclks_to_us(t->pre_range_mclks, t->pre_range_vcsel_pclks);
    t->final_range_vcsel_pclks = (uint16_t)((rd8(d, FINAL_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1);
    t->final_range_mclks = decode_timeout(rd16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = mclks_to_us(t->final_range_mclks, t->final_range_vcsel_pclks);
}

static uint32_t get_budget(vl53l0x_t *d)
{
    seq_en_t e; seq_to_t t;
    get_seq_en(d, &e); get_seq_to(d, &e, &t);
    uint32_t b = 1910 + 960;
    if (e.tcc) b += t.msrc_dss_tcc_us + 590;
    if (e.dss) b += 2 * (t.msrc_dss_tcc_us + 690);
    else if (e.msrc) b += t.msrc_dss_tcc_us + 660;
    if (e.pre_range) b += t.pre_range_us + 660;
    if (e.final_range) b += t.final_range_us + 550;
    d->budget_us = b;
    return b;
}

int vl53l0x_set_timing_budget(vl53l0x_t *d, uint32_t budget_us)
{
    if (budget_us < 20000) return 0;
    seq_en_t e; seq_to_t t;
    get_seq_en(d, &e); get_seq_to(d, &e, &t);
    uint32_t used = 1910 + 960;
    if (e.tcc) used += t.msrc_dss_tcc_us + 590;
    if (e.dss) used += 2 * (t.msrc_dss_tcc_us + 690);
    else if (e.msrc) used += t.msrc_dss_tcc_us + 660;
    if (e.pre_range) used += t.pre_range_us + 660;
    if (e.final_range) {
        used += 550;
        if (used > budget_us) return 0;
        uint32_t fr = us_to_mclks(budget_us - used, t.final_range_vcsel_pclks);
        if (e.pre_range) fr += t.pre_range_mclks;
        wr16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(fr));
        d->budget_us = budget_us;
    }
    return 1;
}

static int get_spad_info(vl53l0x_t *d, uint8_t *count, uint8_t *aperture)
{
    wr8(d, 0x80, 0x01); wr8(d, 0xFF, 0x01); wr8(d, 0x00, 0x00);
    wr8(d, 0xFF, 0x06); wr8(d, 0x83, rd8(d, 0x83) | 0x04);
    wr8(d, 0xFF, 0x07); wr8(d, 0x81, 0x01);
    wr8(d, 0x80, 0x01);
    wr8(d, 0x94, 0x6B); wr8(d, 0x83, 0x00);
    uint32_t t0 = HAL_GetTick();
    while (rd8(d, 0x83) == 0x00) {
        if (HAL_GetTick() - t0 > IO_TIMEOUT_MS) return 0;
    }
    wr8(d, 0x83, 0x01);
    uint8_t tmp = rd8(d, 0x92);
    *count = tmp & 0x7F;
    *aperture = (tmp >> 7) & 0x01;
    wr8(d, 0x81, 0x00); wr8(d, 0xFF, 0x06);
    wr8(d, 0x83, rd8(d, 0x83) & ~0x04);
    wr8(d, 0xFF, 0x01); wr8(d, 0x00, 0x01);
    wr8(d, 0xFF, 0x00); wr8(d, 0x80, 0x00);
    return 1;
}

static int single_ref_cal(vl53l0x_t *d, uint8_t vhv)
{
    wr8(d, SYSRANGE_START, 0x01 | vhv);
    uint32_t t0 = HAL_GetTick();
    while ((rd8(d, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (HAL_GetTick() - t0 > IO_TIMEOUT_MS) return 0;
    }
    wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(d, SYSRANGE_START, 0x00);
    return 1;
}

static const uint8_t TUNING[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},{0x24,0x01},{0x25,0xFF},{0x75,0x00},
    {0xFF,0x01},{0x4E,0x2C},{0x48,0x00},{0x30,0x20},
    {0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},{0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},
    {0x50,0x06},{0x51,0x00},{0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},{0x65,0x00},{0x66,0xA0},
    {0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},{0x4A,0x00},
    {0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},
    {0xFF,0x01},{0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},{0x0E,0x06},{0x20,0x1A},{0x43,0x40},
    {0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},
    {0xFF,0x00},{0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},{0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},
    {0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},
    {0xFF,0x01},{0x8E,0x01},{0x00,0x01},{0xFF,0x00},{0x80,0x00},
};

int vl53l0x_init(vl53l0x_t *d, I2C_HandleTypeDef *hi2c)
{
    d->hi2c = hi2c;
    d->addr = VL53L0X_DEFAULT_ADDR;
    d->err = 0;

    if (rd8(d, IDENTIFICATION_MODEL_ID) != 0xEE) return 0;

    /* I/O 2V8 */
    wr8(d, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, rd8(d, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    /* I2C standard mode */
    wr8(d, 0x88, 0x00);
    wr8(d, 0x80, 0x01); wr8(d, 0xFF, 0x01); wr8(d, 0x00, 0x00);
    d->stop_variable = rd8(d, 0x91);
    wr8(d, 0x00, 0x01); wr8(d, 0xFF, 0x00); wr8(d, 0x80, 0x00);
    /* tat gioi han MSRC va PRE_RANGE */
    wr8(d, MSRC_CONFIG_CONTROL, rd8(d, MSRC_CONFIG_CONTROL) | 0x12);
    /* signal rate limit 0.25 MCPS (Q9.7) */
    wr16(d, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* SPAD tham chieu */
    uint8_t spad_count, aperture;
    if (!get_spad_info(d, &spad_count, &aperture)) return 0;
    uint8_t map[6];
    rdN(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);
    wr8(d, 0xFF, 0x01);
    wr8(d, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    wr8(d, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    wr8(d, 0xFF, 0x00);
    wr8(d, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    uint8_t first = aperture ? 12 : 0, enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count) map[i / 8] &= (uint8_t)~(1 << (i % 8));
        else if ((map[i / 8] >> (i % 8)) & 1) enabled++;
    }
    wrN(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);

    /* tuning mac dinh cua ST */
    for (unsigned k = 0; k < sizeof(TUNING) / sizeof(TUNING[0]); k++)
        wr8(d, TUNING[k][0], TUNING[k][1]);

    /* ngat GPIO: co mau moi */
    wr8(d, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    wr8(d, GPIO_HV_MUX_ACTIVE_HIGH, rd8(d, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01);

    uint32_t b = get_budget(d);
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    vl53l0x_set_timing_budget(d, b);

    /* hieu chuan tham chieu */
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!single_ref_cal(d, 0x40)) return 0;
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!single_ref_cal(d, 0x00)) return 0;
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return d->err == 0;
}

void vl53l0x_set_address(vl53l0x_t *d, uint8_t new_addr)
{
    wr8(d, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    d->addr = new_addr & 0x7F;
}

void vl53l0x_start_continuous(vl53l0x_t *d)
{
    wr8(d, 0x80, 0x01); wr8(d, 0xFF, 0x01); wr8(d, 0x00, 0x00);
    wr8(d, 0x91, d->stop_variable);
    wr8(d, 0x00, 0x01); wr8(d, 0xFF, 0x00); wr8(d, 0x80, 0x00);
    wr8(d, SYSRANGE_START, 0x02);           /* back-to-back */
}

int vl53l0x_read_if_ready(vl53l0x_t *d, uint16_t *mm)
{
    if ((rd8(d, RESULT_INTERRUPT_STATUS) & 0x07) == 0) return 0;
    *mm = rd16(d, RESULT_RANGE_STATUS + 10);
    wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01);
    return 1;
}
