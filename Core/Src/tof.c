/*
 * tof.c - Khoi dong 4 VL53L0X bang XSHUT va gan dia chi rieng
 *   L = PB15, FL = PB14, FR = PB13, R = PB12
 */
#include "tof.h"
#include "vl53l0x.h"
#include "mm_config.h"

extern I2C_HandleTypeDef hi2c1;

static vl53l0x_t dev[TOF_N];
static uint8_t okMask;
static volatile uint16_t val[TOF_N] = { TOF_FAR, TOF_FAR, TOF_FAR, TOF_FAR };
static volatile uint16_t cnt[TOF_N];

/* bu sai so tung cam bien (mm), chinh neu cam bien doc lech */
static const int16_t OFFSET[TOF_N] = { 0, 0, 0, 0 };

static GPIO_TypeDef *const XS_PORT[TOF_N] = { XSHUT_L_PORT, XSHUT_FL_PORT, XSHUT_FR_PORT, XSHUT_R_PORT };
static const uint16_t XS_PIN[TOF_N] = { XSHUT_L_PIN, XSHUT_FL_PIN, XSHUT_FR_PIN, XSHUT_R_PIN };

uint8_t tof_init(void)
{
    okMask = 0;
    for (int i = 0; i < TOF_N; i++) HAL_GPIO_WritePin(XS_PORT[i], XS_PIN[i], GPIO_PIN_RESET);
    HAL_Delay(20);
    for (int i = 0; i < TOF_N; i++) {
        HAL_GPIO_WritePin(XS_PORT[i], XS_PIN[i], GPIO_PIN_SET);
        HAL_Delay(10);
        if (vl53l0x_init(&dev[i], &hi2c1)) {
            vl53l0x_set_address(&dev[i], (uint8_t)(0x30 + i));
            vl53l0x_set_timing_budget(&dev[i], TOF_TIMING_BUDGET_US);
            vl53l0x_start_continuous(&dev[i]);
            okMask |= (uint8_t)(1 << i);
        }
    }
    return okMask;
}

void tof_update(void)
{
    for (int i = 0; i < TOF_N; i++) {
        if (!(okMask & (1 << i))) continue;
        uint16_t mm;
        if (vl53l0x_read_if_ready(&dev[i], &mm)) {
            int v = (mm >= 2000) ? TOF_FAR : (int)mm + OFFSET[i];
            if (v < 0) v = 0;
            if (v > TOF_FAR) v = TOF_FAR;
            val[i] = (uint16_t)v;
            cnt[i]++;
        }
    }
}

uint16_t tof_get(int i) { return val[i]; }

uint16_t tof_front(void)
{
    uint16_t a = val[TOF_FL], b = val[TOF_FR];
    if (a >= TOF_FAR) return b;
    if (b >= TOF_FAR) return a;
    return (uint16_t)((a + b) / 2);
}

void tof_wait_fresh(int samples, uint32_t timeout_ms)
{
    uint16_t c0[TOF_N];
    for (int i = 0; i < TOF_N; i++) c0[i] = cnt[i];
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        tof_update();
        int done = 1;
        for (int i = 0; i < TOF_N; i++)
            if ((okMask & (1 << i)) && (uint16_t)(cnt[i] - c0[i]) < samples) done = 0;
        if (done) break;
    }
}
