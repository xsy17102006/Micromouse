/*
 * vl53l0x.h - Driver VL53L0X cho STM32 HAL (port tu thu vien Pololu VL53L0X)
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include "main.h"
#include <stdint.h>

#define VL53L0X_DEFAULT_ADDR 0x29     /* dia chi 7 bit */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr;           /* 7 bit */
    uint8_t  stop_variable;
    uint8_t  err;
    uint32_t budget_us;
} vl53l0x_t;

int  vl53l0x_init(vl53l0x_t *d, I2C_HandleTypeDef *hi2c);   /* 1 = OK */
void vl53l0x_set_address(vl53l0x_t *d, uint8_t new_addr);
int  vl53l0x_set_timing_budget(vl53l0x_t *d, uint32_t budget_us);
void vl53l0x_start_continuous(vl53l0x_t *d);
/* Khong chan: tra ve 1 neu co mau moi (mm ghi vao *mm) */
int  vl53l0x_read_if_ready(vl53l0x_t *d, uint16_t *mm);

#endif
