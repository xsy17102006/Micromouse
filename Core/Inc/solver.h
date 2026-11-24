/*
 * solver.h - 3 phan me cung tach rieng:
 *   explore_run()  (explore.c)  : MODE 4 - chi do map bang flood fill
 *   runflood_run() (runflood.c) : MODE 5 - chay START -> DICH, flood fill tung o
 *   fastrun_run()  (fastrun.c)  : MODE 6 - chay START -> DICH, nhanh, gop doan thang
 * Ca 3 tra ve 1 neu xong, 0 neu bi dung / loi (xem dbgErr).
 */
#ifndef MM_SOLVER_H_
#define MM_SOLVER_H_

#include <stdint.h>

int explore_run(void);
int runflood_run(void);
int fastrun_run(void);

/* ---- ham ho tro, dinh nghia o cuoi mm_app.c ---- */
void mm_poll(void);                      /* tof_update + debug + kiem tra nut dung  */
void mm_wait_ms(uint32_t ms);            /* cho ms nhung van poll                   */
int  mm_wait_motion(int straight);       /* 1 = bam tuong; tra 0 neu bi dung        */
int  mm_turn_rel(int rel);               /* 0 thang, 1 phai, 2 quay dau, 3 trai     */
int  mm_aborted(void);
void mm_blink(int n, int ms);
void mm_set_pose(int x, int y, int heading);   /* cap nhat bien debug dbgX/Y/Head   */

/* ---- bien debug dung chung (dinh nghia trong mm_app.c) ---- */
extern volatile int32_t dbgErr;          /* 1 khong co duong, 2 thieu cam bien */
extern volatile int32_t dbgStep;
extern volatile int32_t dbgTofMask;

#endif
