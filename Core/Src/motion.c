/*
 * motion.c - Bo dieu khien vi tri cho 2 banh (thay cho moveForward()/turn() Arduino)
 *
 * Moi 1 ms: tang/giam toc do theo hinh thang, cong don ra diem dat (setpoint),
 * PD vi tri cho tung banh + bu lech 2 banh + bu lai theo tuong ben.
 */
#include "motion.h"
#include "motor.h"
#include "mm_config.h"
#include <math.h>

typedef enum { MOT_IDLE = 0, MOT_STRAIGHT, MOT_TURN, MOT_MANUAL } mot_mode_t;

static volatile mot_mode_t mode = MOT_IDLE;
static volatile float sp, vel, target, vmax, acc;
static volatile int32_t l0, r0;
static volatile int8_t dl = 1, dr = 1;
static volatile float steer;
static volatile int manL, manR;
static float pel, per;
static uint16_t settle, tmo;

static inline float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static float measured(void)
{
    float mL = (float)((g_encL - l0) * dl);
    float mR = (float)((g_encR - r0) * dr) * ENC_SCALE_R;
    return 0.5f * (mL + mR);
}

void motion_tick(void)
{
    if (mode == MOT_IDLE) return;
    if (mode == MOT_MANUAL) { motor_set(manL, manR); return; }

    /* profile toc do */
    float rem = target - sp;
    float vlim = sqrtf(2.0f * acc * (rem > 0 ? rem : 0));
    if (vlim < VMIN) vlim = VMIN;
    float v = vel + acc;
    if (v > vmax) v = vmax;
    if (v > vlim) v = vlim;
    if (rem <= 0) { v = 0; sp = target; }
    else { sp += v; if (sp > target) sp = target; }
    vel = v;

    /* PD vi tri (banh phai nhan he so bu ENC_SCALE_R) */
    float mL = (float)((g_encL - l0) * dl);
    float mR = (float)((g_encR - r0) * dr) * ENC_SCALE_R;
    float eL = sp - mL, eR = sp - mR;

    /* tu ngat neu banh khong theo kip lenh (encoder loi / sai chieu) */
    if (fabsf(eL) > 2500.0f || fabsf(eR) > 2500.0f) {
        motor_brake();
        mode = MOT_IDLE;
        return;
    }

    float c  = KC * (mL - mR);
    float uL = KFF * v + KP * eL + KD * (eL - pel) - c;
    float uR = KFF * v + KP * eR + KD * (eR - per) + c;
    pel = eL; per = eR;
    if (mode == MOT_STRAIGHT) { uL += steer; uR -= steer; }
    uL = clampf(uL, PWM_LIMIT);
    uR = clampf(uR, PWM_LIMIT);
    motor_set(dl * (int)uL, dr * (int)uR);

    /* ket thuc khi den dich va on dinh */
    if (sp >= target) {
        if (fabsf(eL) < SETTLE_TOL && fabsf(eR) < SETTLE_TOL) settle++; else settle = 0;
        if (settle > 15 || ++tmo > 400) {
            motor_brake();
            mode = MOT_IDLE;
        }
    }
}

static void start(mot_mode_t m, float ticks, int8_t a, int8_t b, float vm, float ac)
{
    __disable_irq();
    l0 = g_encL; r0 = g_encR;
    dl = a; dr = b;
    sp = 0; vel = 0; target = ticks; vmax = vm; acc = ac;
    steer = 0; pel = per = 0; settle = 0; tmo = 0;
    mode = m;
    __enable_irq();
}

void motion_straight(float ticks, float vm) { start(MOT_STRAIGHT, ticks, 1, 1, vm, ACC_STRAIGHT); }

void motion_turn(float ticks, int dir, float vm)
{
    int8_t a = dir > 0 ? 1 : -1;
    start(MOT_TURN, ticks, a, (int8_t)-a, vm, ACC_TURN);
}

void motion_manual(int left, int right)
{
    manL = left; manR = right;
    mode = MOT_MANUAL;
}

void motion_stop(void)
{
    mode = MOT_IDLE;
    motor_brake();
}

int motion_busy(void) { return mode == MOT_STRAIGHT || mode == MOT_TURN; }

float motion_remaining(void) { return target - measured(); }

void motion_set_remaining(float ticks)
{
    __disable_irq();
    float t = measured() + ticks;
    if (t < sp) t = sp;
    target = t;
    __enable_irq();
}

void motion_set_steer(float s) { steer = clampf(s, STEER_MAX); }

