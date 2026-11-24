/*
 * tof.h - Quan ly 4 cam bien VL53L0X tren chung bus I2C1 (PB6/PB7)
 */
#ifndef TOF_H
#define TOF_H

#include <stdint.h>

enum { TOF_L = 0, TOF_FL, TOF_FR, TOF_R, TOF_N };
#define TOF_FAR 999

uint8_t  tof_init(void);              /* tra ve bitmask cam bien OK (0x0F = du 4) */
void     tof_update(void);            /* goi lien tuc trong vong lap chinh        */
uint16_t tof_get(int i);              /* mm, TOF_FAR neu khong thay vat           */
uint16_t tof_front(void);             /* trung binh FL, FR                        */
void     tof_wait_fresh(int samples, uint32_t timeout_ms);

#endif
