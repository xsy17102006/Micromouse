/*
 * motion.h - Di thang / quay voi profile hinh thang, chay trong ngat TIM4 (1 kHz)
 */
#ifndef MOTION_H
#define MOTION_H

void  motion_tick(void);                          /* goi moi 1 ms (TIM4)        */
void  motion_straight(float ticks, float vmax);   /* di thang ticks xung        */
void  motion_turn(float ticks, int dir, float vmax); /* dir +1 = phai, -1 = trai */
void  motion_manual(int left, int right);         /* PWM truc tiep              */
void  motion_stop(void);
int   motion_busy(void);
float motion_remaining(void);                     /* xung con lai               */
void  motion_set_remaining(float ticks);          /* doi diem dung (chinh theo tuong truoc) */
void  motion_set_steer(float steer);              /* bu lai theo tuong ben      */

#endif
