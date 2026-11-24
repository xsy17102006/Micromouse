/*
 * motor.h - TB6612FNG + encoder N20 (giai ma x4 bang ngat EXTI)
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

extern volatile int32_t g_encL;   /* duong = banh trai tien */
extern volatile int32_t g_encR;   /* duong = banh phai tien */

void motor_init(void);
void motor_set(int left, int right);   /* -1000..1000, duong = tien */
void motor_brake(void);
void enc_exti(uint16_t pin);           /* goi trong HAL_GPIO_EXTI_Callback */

#endif
