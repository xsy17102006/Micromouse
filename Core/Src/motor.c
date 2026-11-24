/*
 * motor.c - Dieu khien TB6612FNG va doc encoder
 *   Trai : PWMA = PA2 (TIM2_CH3), AIN1 = PA4, AIN2 = PA3
 *   Phai : PWMB = PB0 (TIM3_CH3), BIN1 = PA6, BIN2 = PA7
 *   STBY = PA5
 *   Encoder trai PB8/PB9, phai PB10/PB11 (EXTI ca 2 suon)
 *   (PB8..PB11 khong dung duoc che do Encoder cua timer nen giai ma bang phan mem)
 */
#include "motor.h"
#include "mm_config.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

volatile int32_t g_encL = 0;
volatile int32_t g_encR = 0;
static volatile uint8_t stL, stR;

/* Bang giai ma quadrature: chi so = (trang_thai_cu << 2) | trang_thai_moi */
static const int8_t QT[16] = { 0, -1, 1, 0,
                               1, 0, 0, -1,
                              -1, 0, 0, 1,
                               0, 1, -1, 0 };

static inline uint8_t rdL(uint32_t idr) { return (uint8_t)((((idr >> 8) & 1) << 1) | ((idr >> 9) & 1)); }
static inline uint8_t rdR(uint32_t idr) { return (uint8_t)((((idr >> 10) & 1) << 1) | ((idr >> 11) & 1)); }

void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    uint32_t idr = GPIOB->IDR;
    stL = rdL(idr);
    stR = rdR(idr);
    g_encL = g_encR = 0;
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
    motor_brake();
}

static void drive(int v, GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2,
                  TIM_HandleTypeDef *ht)
{
    if (v > 1000) v = 1000;
    if (v < -1000) v = -1000;
    if (v > 0) {
        HAL_GPIO_WritePin(p1, n1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(p2, n2, GPIO_PIN_RESET);
    } else if (v < 0) {
        HAL_GPIO_WritePin(p1, n1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p2, n2, GPIO_PIN_SET);
        v = -v;
    } else {                                   /* phanh ngan mach */
        HAL_GPIO_WritePin(p1, n1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(p2, n2, GPIO_PIN_SET);
    }
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(ht);
    __HAL_TIM_SET_COMPARE(ht, TIM_CHANNEL_3, (uint32_t)v * (arr + 1) / 1000);
}

void motor_set(int left, int right)
{
    drive(left * MOTOR_SIGN_L, AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN, &htim2);
    drive(right * MOTOR_SIGN_R, BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN, &htim3);
}

void motor_brake(void)
{
    motor_set(0, 0);
}

void enc_exti(uint16_t pin)
{
    uint32_t idr = GPIOB->IDR;
    if (pin == GPIO_PIN_8 || pin == GPIO_PIN_9) {
        uint8_t s = rdL(idr);
        g_encL += ENC_SIGN_L * QT[(stL << 2) | s];
        stL = s;
    } else if (pin == GPIO_PIN_10 || pin == GPIO_PIN_11) {
        uint8_t s = rdR(idr);
        g_encR += ENC_SIGN_R * QT[(stR << 2) | s];
        stR = s;
    }
}
