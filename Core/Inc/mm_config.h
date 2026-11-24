/*
 * mm_config.h  -  Cau hinh chung cho Micromouse (STM32F103C8T6 Blue Pill)
 *
 * Moi thong so can chinh (calibrate) deu nam o file nay.
 */
#ifndef MM_CONFIG_H
#define MM_CONFIG_H

#include "main.h"

/* ============================================================
 *  ME CUNG (theo so do: 4 cot x 6 hang, o 18 x 18 cm)
 *  Toa do: x = cot (0..3, trai -> phai), y = hang (0..5, duoi -> tren)
 *  Huong: 0 = Bac (len), 1 = Dong (phai), 2 = Nam (xuong), 3 = Tay (trai)
 * ============================================================ */
#define MAZE_W            4
#define MAZE_H            6
#define START_X           0      /* o S: goc duoi ben trai           */
#define START_Y           0
#define START_HEADING     0      /* xe dat o S, mui huong len (Bac)  */
#define GOAL_X            2      /* o GOAL: cot 3, hang 4 tinh tu duoi */
#define GOAL_Y            3
#define CELL_MM           180.0f

/* ============================================================
 *  CO KHI / ENCODER  (PHAI CALIBRATE)
 * ============================================================ */
/* So xung encoder (da nhan 4 - giai ma x4) tren 1 mm quang duong.
 * Cach do: mode 1 cho xe chay 1 o, do quang duong thuc te bang thuoc,
 * TICKS_PER_MM_moi = TICKS_PER_MM_cu * 180 / quang_duong_thuc_te_mm   */
#define TICKS_PER_MM      28.0f
#define CELL_TICKS        5040
//(CELL_MM * TICKS_PER_MM)
/* So xung MOI BANH de quay tai cho 90 do (mode 2 de chinh). */
#define TURN90_TICKS      1735.0f

/* Dau (+1/-1). Dung mode 7 de kiem tra:
 *  - banh quay LUI khi lenh tien      -> doi MOTOR_SIGN cua banh do
 *  - banh quay TIEN nhung dbgEnc giam -> doi ENC_SIGN  cua banh do   */
#define MOTOR_SIGN_L      (-1)
#define MOTOR_SIGN_R      (-1)
#define ENC_SIGN_L        (+1)
#define ENC_SIGN_R        (-1)

/* ============================================================
 *  DIEU KHIEN CHUYEN DONG (vong 1 kHz trong ngat TIM4)
 *  Toc do tinh bang xung/ms, gia toc bang xung/ms^2, PWM 0..1000
 * ============================================================ */
#define EXPLORE_VMAX      2.5f    /* ~100 mm/s khi do me cung           */
#define RUN_VMAX          6.0f    /* ~200 mm/s khi chay nhanh (mode 5)  */
#define TURN_VMAX         2.0f
#define VMIN              0.15f
#define ACC_STRAIGHT      0.015f
#define ACC_TURN          0.008f

#define KFF               40.0f   /* feed-forward theo toc do           */
#define KP                6.0f    /* PWM tren 1 xung sai lech           */
#define KD                40.0f
#define KC                6.0f    /* bu lech 2 banh (giu thang)         */
#define PWM_LIMIT         900
#define SETTLE_TOL        30.0f   /* xung                               */
#define ENC_SCALE_R       0.997f   /* xe cong PHAI khi di thang: GIAM (0.995, 0.990...) */


/* ============================================================
 *  CAM BIEN VL53L0X (mm)  - dung mode 3 de doc va chinh
 *  Gia dinh: FL va FR deu chieu THANG ra truoc, L va R chieu ngang.
 * ============================================================ */
#define TOF_TIMING_BUDGET_US  20000
#define SIDE_WALL_MM      110     /* L/R nho hon -> co tuong ben         */
#define FRONT_WALL_MM     40    /* trung binh FL,FR nho hon -> co tuong truoc */
#define SIDE_CENTER_MM    49    /* L hoac R khi xe nam giua o          */
#define FRONT_AT_CENTER_MM 40     /* FL/FR khi xe nam giua o, tuong truoc */
#define FRONT_LR_OFFSET   0       /* FR - FL khi xe vuong goc voi tuong   */
#define FRONT_CORR_MAX_MM 250     /* chi dung FL/FR de chinh diem dung khi < gia tri nay */

#define KWALL             2.0f    /* PWM tren 1 mm lech tam               */
#define STEER_MAX         150.0f
#define ALIGN_MS          400
#define KA_DIST           6.0f
#define KA_ANGLE          6.0f
#define ALIGN_PWM         250

/* ============================================================
 *  CHAN KET NOI (khong can dat label trong CubeMX)
 * ============================================================ */
/* TB6612FNG - dong co TRAI = kenh A, PHAI = kenh B */
#define AIN1_PORT GPIOA
#define AIN1_PIN  GPIO_PIN_4
#define AIN2_PORT GPIOA
#define AIN2_PIN  GPIO_PIN_3
#define BIN1_PORT GPIOA
#define BIN1_PIN  GPIO_PIN_6
#define BIN2_PORT GPIOA
#define BIN2_PIN  GPIO_PIN_7
#define STBY_PORT GPIOA
#define STBY_PIN  GPIO_PIN_5
/* PWMA = PA2 = TIM2_CH3 ; PWMB = PB0 = TIM3_CH3 */

/* Encoder (giai ma phan mem bang EXTI): trai PB8/PB9, phai PB10/PB11 */

/* XSHUT VL53L0X */
#define XSHUT_L_PORT  GPIOB
#define XSHUT_L_PIN   GPIO_PIN_15
#define XSHUT_FL_PORT GPIOB
#define XSHUT_FL_PIN  GPIO_PIN_14
#define XSHUT_FR_PORT GPIOB
#define XSHUT_FR_PIN  GPIO_PIN_13
#define XSHUT_R_PORT  GPIOB
#define XSHUT_R_PIN   GPIO_PIN_12

/* Nut bam (keo len noi, nhan = GND) */
#define SW2_PORT  GPIOA       /* PA11: doi mode     */
#define SW2_PIN   GPIO_PIN_11
#define SW3_PORT  GPIOA       /* PA12: chay / dung  */
#define SW3_PIN   GPIO_PIN_12

/* LED tren Blue Pill (sang muc 0) */
#define LED_PORT  GPIOC
#define LED_PIN   GPIO_PIN_13

/* Trang flash cuoi (64 KB) de luu ban do me cung */
#define MAZE_FLASH_ADDR   0x0800FC00UL

#endif
