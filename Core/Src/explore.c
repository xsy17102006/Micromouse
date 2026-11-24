/*
 * explore.c - MODE 4: DO HET MAP (di moi o) roi moi DEN DICH (flood fill)
 *
 * Giai doan 1 - di het cac o: moi lan dung o tam 1 o
 *   1. Doc tuong trai / truoc / phai bang ToF (bo phieu 5 mau cho on dinh)
 *   2. maze_set_wall -> cap nhat ban do, danh dau o da tham
 *   3. Flood fill voi DICH = TAT CA o CHUA tham (canh chua biet coi nhu THONG)
 *      -> xe luon di toi o chua tham GAN NHAT
 *   4. maze_next_dir chon huong, quay, di thang
 *      (neu phia truoc la cac o DA tham thi di mot mach nhieu o)
 *   Het o chua tham (hoac con o nhung bi tuong bao kin, khong toi duoc) -> sang giai doan 2.
 * Giai doan 2 - den dich: flood fill ve GOAL tren map da biet.
 *
 * Toi dich xong, neu EXPLORE_RETURN_HOME = 1 thi xe tu ve START va quay mui ve
 * huong xuat phat de MODE 5 / 6 chay luon duoc. Dat 0 de xe dung lai o GOAL.
 *
 * Ban do nam trong RAM (maze_walls / maze_known / maze_visited), MODE 5 / 6 dung lai.
 */
#include "solver.h"
#include "mm_config.h"
#include "maze.h"
#include "motion.h"
#include "tof.h"

#ifndef EXPLORE_RETURN_HOME
#define EXPLORE_RETURN_HOME  0
#endif

/* Nguong tuong truoc khi xe dung giua o. KHONG dung FRONT_WALL_MM (40) vi
   FRONT_AT_CENTER_MM cung = 40 nen nhieu se doc nham la khong co tuong. */
#ifndef EXP_FRONT_WALL_MM
#define EXP_FRONT_WALL_MM    100
#endif

#define SENSE_SAMPLES        5
#define SENSE_GAP_MS         20

static int cx, cy, head;

/* Doc 3 tuong quanh o hien tai, ghi vao ban do, danh dau da tham */
static void sense_and_update(void)
{
    int vl = 0, vf = 0, vr = 0;
    for (int i = 0; i < SENSE_SAMPLES; i++) {
        mm_wait_ms(SENSE_GAP_MS);
        if (tof_get(TOF_L) < SIDE_WALL_MM)       vl++;
        if (tof_front()    < EXP_FRONT_WALL_MM)  vf++;
        if (tof_get(TOF_R) < SIDE_WALL_MM)       vr++;
    }
    int half = SENSE_SAMPLES / 2;                /* can >= 3/5 phieu */
    maze_set_wall(cx, cy, (head + 3) & 3, vl > half);   /* trai  */
    maze_set_wall(cx, cy, head,           vf > half);   /* truoc */
    maze_set_wall(cx, cy, (head + 1) & 3, vr > half);   /* phai  */
    maze_visited[maze_idx(cx, cy)] = 1;
}

/* Flood fill toi TAT CA o chua tham.
 * Tra 1 neu con o chua tham ma xe toi duoc, 0 neu da di het (hoac con o bi bao kin). */
static int flood_to_unvisited(void)
{
    uint8_t tx[MAZE_N], ty[MAZE_N];
    int n = 0;
    for (int y = 0; y < MAZE_H; y++)
        for (int x = 0; x < MAZE_W; x++)
            if (!maze_visited[maze_idx(x, y)]) { tx[n] = (uint8_t)x; ty[n] = (uint8_t)y; n++; }
    if (n == 0) return 0;
    maze_flood_fill(tx, ty, n, 0);               /* 0: canh chua biet coi nhu thong */
    return maze_flood[maze_idx(cx, cy)] != 255;  /* 255 = khong toi duoc o nao */
}

/* Di 1 buoc theo flood hien tai (quay + chay thang). Tra 1 neu ok, 0 neu loi / bi dung */
static int step_along_flood(void)
{
    int d = maze_next_dir(cx, cy, head, 0);
    if (d < 0) { dbgErr = 1; return 0; }          /* ket: khong con duong */

    int rel = (d - head + 4) & 3;                 /* 0 thang, 1 phai, 2 quay dau, 3 trai */
    if (!mm_turn_rel(rel)) return 0;
    head = d;
    mm_set_pose(cx, cy, head);

    int n = maze_run_length(cx, cy, d, 0, 1);     /* chi chay mach qua o DA tham */
    motion_straight((float)n * (float)CELL_TICKS, EXPLORE_VMAX);
    if (!mm_wait_motion(1)) return 0;             /* 1: bam tuong + dung theo tuong truoc */

    cx += MAZE_DX[d] * n;
    cy += MAZE_DY[d] * n;
    dbgStep++;
    return 1;
}

/* Di den o (tx,ty) tren map da biet. Tra 1 neu toi noi. */
static int go_to(uint8_t tx, uint8_t ty)
{
    while (1) {
        sense_and_update();
        if (mm_aborted()) return 0;
        mm_set_pose(cx, cy, head);

        maze_flood_fill(&tx, &ty, 1, 0);
        if (cx == tx && cy == ty) return 1;

        if (!step_along_flood()) return 0;
    }
}

int explore_run(void)
{
    if (dbgTofMask != 0x0F) { dbgErr = 2; return 0; }

    maze_reset();
    cx = START_X; cy = START_Y; head = START_HEADING;
    mm_set_pose(cx, cy, head);
    dbgStep = 0;

    /* ---- Giai doan 1: di het moi o ---- */
    while (1) {
        sense_and_update();
        if (mm_aborted()) return 0;
        mm_set_pose(cx, cy, head);

        if (!flood_to_unvisited()) break;         /* da di het */
        if (!step_along_flood()) return 0;
    }

    /* ---- Giai doan 2: den dich ---- */
    if (!go_to(GOAL_X, GOAL_Y)) return 0;

#if EXPLORE_RETURN_HOME
    if (!go_to(START_X, START_Y)) return 0;
    {
        int rel = (START_HEADING - head + 4) & 3; /* quay mui ve huong xuat phat */
        if (!mm_turn_rel(rel)) return 0;
        head = START_HEADING;
        mm_set_pose(cx, cy, head);
    }
#endif

    return 1;
}
