/*
 * mm_app.c - Micromouse (STM32F103 HAL)
 *
 * NUT BAM
 *   SW2 (PA11): doi mode 1..6 (LED PC13 nhay dung so lan = so mode)
 *   SW3 (PA12): chay mode dang chon (sau 1 s). Bam SW3 khi dang chay = dung khan cap
 *
 * MODE
 *   1  Di thang 1 o (CELL_TICKS), khong dung cam bien
 *   2  Re thu: phai 90, trai 90, quay dau 180
 *   3  Tranh vat can: chay thang, gap vat thi ne (chay mai, bam SW3 de dung)
 *   4  CHI DO MAP bang flood fill (toi dich roi tu ve START)  (explore.c)
 *   5  Chay START -> DICH: flood fill, tung o mot, on dinh    (runflood.c)
 *   6  Chay START -> DICH: chay nhanh, gop doan thang         (fastrun.c)
 *
 * Dung: dat xe o START (mui len) -> mode 4 -> xe tu ve START -> mode 5 hoac 6.
 * Ban do nam trong RAM nen tat nguon la mat, phai chay lai mode 4.
 *
 * DEBUG: STM32CubeIDE -> Live Expressions
 *   Ghi dbgCmd = 1..6 de chay mode, dbgCmd = -1 de dung.
 */
#include "mm_app.h"
#include "mm_config.h"
#include "motor.h"
#include "motion.h"
#include "tof.h"
#include "maze.h"
#include "solver.h"
#include <string.h>
#include <math.h>

extern TIM_HandleTypeDef htim4;

/* ----------------- bien debug (Live Expressions) ----------------- */
volatile int32_t  dbgCmd = 0;
volatile int32_t  dbgMode = 1;
volatile int32_t  dbgState = 0;      /* 0 cho, 1 dang chay, 2 xong, 3 loi, 4 bi dung */
volatile int32_t  dbgErr = 0;        /* 1 khong co duong, 2 thieu cam bien */
volatile int32_t  dbgX, dbgY, dbgHead;
volatile int32_t  dbgEncL, dbgEncR;
volatile uint16_t dbgL, dbgFL, dbgFR, dbgR;
volatile int32_t  dbgWallL, dbgWallF, dbgWallR;
volatile int32_t  dbgTofMask;
volatile int32_t  dbgRunL, dbgRunR;  /* so xung moi banh trong lan chay mode 1 vua roi */
volatile int32_t  dbgStep = 0;       /* buoc dang chay trong mode 4 / 5 / 6 */
volatile uint8_t  dbgFlood[MAZE_H][MAZE_W];   /* dbgFlood[y][x] */
volatile uint8_t  dbgWalls[MAZE_H][MAZE_W];   /* bit0 N, bit1 E, bit2 S, bit3 W */

enum { ST_IDLE = 0, ST_RUN, ST_DONE, ST_FAIL, ST_ABORT };
#define NUM_MODES 6

static int cx, cy, head;
static uint8_t abortFlag;

/* ======================= tien ich ======================= */
static void led(int on) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

static void blink(int n, int ms)
{
    for (int i = 0; i < n; i++) { led(1); HAL_Delay(ms); led(0); HAL_Delay(ms); }
}

typedef struct { GPIO_TypeDef *port; uint16_t pin; uint8_t last; uint32_t t; } btn_t;
static btn_t sw2 = { SW2_PORT, SW2_PIN, 1, 0 };
static btn_t sw3 = { SW3_PORT, SW3_PIN, 1, 0 };

static int btn_pressed(btn_t *b)          /* 1 khi vua nhan (suon xuong) */
{
    uint8_t v = (uint8_t)HAL_GPIO_ReadPin(b->port, b->pin);
    uint32_t now = HAL_GetTick();
    if (v != b->last && now - b->t > 30) {
        b->t = now;
        b->last = v;
        return v == 0;
    }
    return 0;
}

static void dbg_refresh(void)
{
    dbgL = tof_get(TOF_L); dbgFL = tof_get(TOF_FL);
    dbgFR = tof_get(TOF_FR); dbgR = tof_get(TOF_R);
    dbgWallL = dbgL < SIDE_WALL_MM;
    dbgWallF = tof_front() < FRONT_WALL_MM;
    dbgWallR = dbgR < SIDE_WALL_MM;
    dbgEncL = g_encL; dbgEncR = g_encR;
    dbgX = cx; dbgY = cy; dbgHead = head;
    for (int y = 0; y < MAZE_H; y++)
        for (int x = 0; x < MAZE_W; x++) {
            dbgFlood[y][x] = maze_flood[maze_idx(x, y)];
            dbgWalls[y][x] = maze_walls[maze_idx(x, y)];
        }
}

static void poll_run(void)
{
    tof_update();
    dbg_refresh();
    if (btn_pressed(&sw3) || dbgCmd == -1) { abortFlag = 1; dbgCmd = 0; }
}

static void wait_ms(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) poll_run();
}

/* ======================= chuyen dong ======================= */
static float wall_steer(void)
{
    int l = tof_get(TOF_L), r = tof_get(TOF_R);
    int hasL = l < SIDE_WALL_MM, hasR = r < SIDE_WALL_MM;
    float e = 0;
    if (hasL && hasR) e = 0.5f * (float)(r - l);
    else if (hasL)    e = (float)(SIDE_CENTER_MM - l);
    else if (hasR)    e = (float)(r - SIDE_CENTER_MM);
    if (tof_front() < 70) e = 0;     /* sat tuong truoc: cam bien ben de nhieu */
    return KWALL * e;                /* duong = lai sang phai */
}

/* Thay tuong truoc trong doan cuoi -> dung som cho dung khoang cach. KHONG bao gio keo dai. */
static void front_correct(void)
{
    float rem = motion_remaining();
    if (rem > CELL_TICKS) return;
    uint16_t f = tof_front();
    if (f > FRONT_CORR_MAX_MM) return;
    float want = ((float)f - FRONT_AT_CENTER_MM) * TICKS_PER_MM;
    if (want < 0) want = 0;
    if (want < rem && want > rem - CELL_TICKS * 0.4f)
        motion_set_remaining(want);
}

/* straight = 1: dung cam bien (bam tuong, dung theo tuong truoc); 0: chi encoder */
static int wait_motion(int straight)
{
    while (motion_busy()) {
        poll_run();
        if (abortFlag) { motion_stop(); return 0; }
        if (straight) {
            motion_set_steer(wall_steer());
            front_correct();
        }
    }
    motion_set_steer(0);
    return 1;
}

/* rel: 0 thang, 1 phai, 2 quay dau, 3 trai */
static int turn_rel(int rel)
{
    if (rel == 0) return 1;
    float t = (rel == 2) ? 2.0f * TURN90_TICKS : TURN90_TICKS;
    int dir = (rel == 3) ? -1 : 1;
    motion_turn(t, dir, TURN_VMAX);
    if (!wait_motion(0)) return 0;
    wait_ms(30);
    return !abortFlag;
}

/* ======================= MODE 3: tranh vat can ======================= */
/* Xe chay thang lien tuc. Vat ben qua gan -> lai tranh ra.
   Vat phia truoc -> dung, re sang ben trong hon (ca 2 ben bi chan -> quay dau).
   Chay mai cho toi khi bam SW3 (hoac dbgCmd = -1).                              */
#define AVOID_FRONT_MM   30     /* vat truoc gan hon so nay (mm) -> dung va ne   */
#define AVOID_SIDE_MM    40      /* vat ben gan hon so nay (mm)   -> lai tranh ra */
#define AVOID_KSIDE      4.0f    /* do manh khi lai tranh vat ben                 */
#define AVOID_VMAX       EXPLORE_VMAX

static int mode_avoid(void)
{
    if (dbgTofMask != 0x0F) { dbgErr = 2; return 0; }

    while (!abortFlag) {
        /* 1. di thang cho toi khi gap vat phia truoc */
        motion_straight(100000.0f, AVOID_VMAX);           /* "di mai" (~3.5 m) */
        while (motion_busy()) {
            poll_run();
            if (abortFlag) { motion_stop(); return 0; }

            if (tof_front() < AVOID_FRONT_MM) { motion_stop(); break; }

            motion_set_steer(wall_steer());     /* bam tuong: giu giua 2 tuong / cach 1 tuong */
        }
        motion_set_steer(0);
        if (abortFlag) return 0;

        /* 2. dung lai nhin 2 ben, chon huong trong hon */
        wait_ms(200);
        int l = tof_get(TOF_L), r = tof_get(TOF_R);
        int rel;
        if (l >= SIDE_WALL_MM)       rel = 3;   /* trai trong -> re trai (uu tien) */
        else if (r >= SIDE_WALL_MM)  rel = 1;   /* phai trong -> re phai */
        else                         rel = 2;   /* ca 2 bi chan -> quay dau */
        if (!turn_rel(rel)) return 0;
        wait_ms(150);
    }
    return 0;
}

/* ======================= ham ho tro cho explore.c / fastrun.c ======================= */
void mm_poll(void)                       { poll_run(); }
void mm_wait_ms(uint32_t ms)             { wait_ms(ms); }
int  mm_wait_motion(int straight)        { return wait_motion(straight); }
int  mm_turn_rel(int rel)                { return turn_rel(rel); }
int  mm_aborted(void)                    { return abortFlag; }
void mm_blink(int n, int ms)             { blink(n, ms); }
void mm_set_pose(int x, int y, int h)    { cx = x; cy = y; head = h; }

/* ======================= chay mode ======================= */
static void run_mode(int m)
{
    abortFlag = 0;
    dbgErr = 0;
    dbgStep = 0;
    dbgState = ST_RUN;
    led(1);
    wait_ms(1000);                 /* bo tay ra khoi xe */
    led(0);
    int ok = 1;

    switch (m) {
    case 1: {                                       /* di 1 o, KHONG dung cam bien */
        int32_t l = g_encL, r = g_encR;
        motion_straight(CELL_TICKS, EXPLORE_VMAX);
        ok = wait_motion(0);
        dbgRunL = g_encL - l;
        dbgRunR = g_encR - r;
        break;
    }
    case 2:                                         /* phai 90, trai 90, quay dau */
        ok = turn_rel(1);
        if (ok) { wait_ms(500); ok = turn_rel(3); }
        if (ok) { wait_ms(500); ok = turn_rel(2); }
        break;
    case 3: ok = mode_avoid(); break;
    case 4: ok = explore_run(); break;              /* chi do map */
    case 5: ok = runflood_run(); break;             /* START -> DICH: tung o */
    case 6: ok = fastrun_run(); break;              /* START -> DICH: nhanh */
    default: break;
    }
    motion_stop();
    if (abortFlag)  dbgState = ST_ABORT;
    else if (!ok) { dbgState = ST_FAIL; blink(10, 50); }
    else            dbgState = ST_DONE;
}

/* ======================= init / loop ======================= */
void mm_app_init(void)
{
    led(0);
    motor_init();
    HAL_TIM_Base_Start_IT(&htim4);           /* vong dieu khien 1 kHz */
    dbgTofMask = tof_init();
    maze_reset();
    cx = START_X; cy = START_Y; head = START_HEADING;
    if (dbgTofMask != 0x0F) blink(10, 50);    /* thieu cam bien */
    blink(dbgMode, 150);
}

void mm_app_loop(void)
{
    tof_update();
    dbg_refresh();

    if (btn_pressed(&sw2)) {
        dbgMode = dbgMode % NUM_MODES + 1;
        blink(dbgMode, 150);
    }
    if (btn_pressed(&sw3)) {
        run_mode(dbgMode);
    }
    if (dbgCmd > 0 && dbgCmd <= NUM_MODES) {
        int m = dbgCmd;
        dbgCmd = 0;
        dbgMode = m;
        run_mode(m);
    }
}
