/*
 * fastrun.c - MODE 6: CHAY NHANH START -> DICH tren map da do (MODE 4)
 *
 * Dieu kien: da chay MODE 4 (map con trong RAM) va xe dang dat o START,
 * mui huong START_HEADING (MODE 4 tu quay ve dung cho nay).
 *
 * Khac MODE 5 (runflood.c):
 *   - Chi di qua canh DA BIET la thong (canh chua biet coi nhu tuong)
 *   - Tim duong theo THOI GIAN, khong phai so o: moi o thang tinh 1,
 *     moi lan re 90 do tinh TURN_COST. Nen duong it re hon duoc chon
 *     du dai hon vai o.
 *     Bellman-Ford tren trang thai (o, huong): 24 o x 4 huong, rat nhe.
 *   - Gop cac o thang lien tiep thanh 1 doan, chay voi RUN_VMAX
 *   - Khong dung lai do tuong
 */
#include "solver.h"
#include "mm_config.h"
#include "maze.h"
#include "motion.h"
#include "tof.h"

#ifndef TURN_COST
#define TURN_COST   3       /* 1 lan re 90 do ~ di 3 o thang. Chinh theo thuc te. */
#endif

#define INF         30000
#define MAX_LEGS    32

static int16_t C[MAZE_N][4];            /* C[o][huong] = chi phi tu day toi dich */

typedef struct { uint8_t rel; uint8_t cells; } leg_t;   /* quay roi di thang */
static leg_t plan[MAX_LEGS];
static int   nLegs;

static void build_cost(void)
{
    for (int i = 0; i < MAZE_N; i++)
        for (int h = 0; h < 4; h++) C[i][h] = INF;
    for (int h = 0; h < 4; h++) C[maze_idx(GOAL_X, GOAL_Y)][h] = 0;

    for (int it = 0; it < 200; it++) {
        int changed = 0;
        for (int y = 0; y < MAZE_H; y++)
        for (int x = 0; x < MAZE_W; x++)
        for (int h = 0; h < 4; h++) {
            if (x == GOAL_X && y == GOAL_Y) continue;
            int i = maze_idx(x, y);
            int best = C[i][h];

            if (maze_open(x, y, h, 1)) {                    /* di thang 1 o */
                int n = C[maze_idx(x + MAZE_DX[h], y + MAZE_DY[h])][h];
                if (n < INF && n + 1 < best) best = n + 1;
            }
            int r = C[i][(h + 1) & 3], l = C[i][(h + 3) & 3];   /* re tai cho */
            if (r < INF && r + TURN_COST < best) best = r + TURN_COST;
            if (l < INF && l + TURN_COST < best) best = l + TURN_COST;

            if (best < C[i][h]) { C[i][h] = (int16_t)best; changed = 1; }
        }
        if (!changed) break;
    }
}

/* Theo chi phi, di tu START toi GOAL va gom thanh cac doan {quay, so o} */
static int build_plan(void)
{
    nLegs = 0;
    int x = START_X, y = START_Y, h = START_HEADING, pend = 0;

    for (int guard = 0; guard < 200; guard++) {
        if (x == GOAL_X && y == GOAL_Y) return 1;

        int cur = C[maze_idx(x, y)][h];
        if (cur >= INF) return 0;

        int nf = INF;
        if (maze_open(x, y, h, 1)) nf = C[maze_idx(x + MAZE_DX[h], y + MAZE_DY[h])][h];

        if (nf < INF && nf + 1 == cur) {                    /* di thang */
            if (pend != 0 || nLegs == 0) {
                if (nLegs >= MAX_LEGS) return 0;
                plan[nLegs].rel = (uint8_t)pend;
                plan[nLegs].cells = 1;
                nLegs++;
                pend = 0;
            } else {
                plan[nLegs - 1].cells++;
            }
            x += MAZE_DX[h];
            y += MAZE_DY[h];
        } else if (C[maze_idx(x, y)][(h + 1) & 3] + TURN_COST == cur) {   /* re phai */
            h = (h + 1) & 3;
            pend = (pend + 1) & 3;
        } else if (C[maze_idx(x, y)][(h + 3) & 3] + TURN_COST == cur) {   /* re trai */
            h = (h + 3) & 3;
            pend = (pend + 3) & 3;
        } else {
            return 0;
        }
    }
    return 0;
}

int fastrun_run(void)
{
    if (dbgTofMask != 0x0F) { dbgErr = 2; return 0; }

    /* kiem tra co duong da biet tu START toi GOAL khong (flood cung dung de debug) */
    uint8_t gx = GOAL_X, gy = GOAL_Y;
    maze_flood_fill(&gx, &gy, 1, 1);
    if (maze_flood[maze_idx(START_X, START_Y)] == 255) { dbgErr = 1; return 0; }

    build_cost();
    if (!build_plan()) { dbgErr = 1; return 0; }

    int x = START_X, y = START_Y, h = START_HEADING;
    mm_set_pose(x, y, h);
    dbgStep = 0;

    for (int i = 0; i < nLegs; i++) {
        dbgStep = i + 1;

        if (!mm_turn_rel(plan[i].rel)) return 0;            /* rel 0: khong quay */
        h = (h + plan[i].rel) & 3;

        motion_straight((float)plan[i].cells * (float)CELL_TICKS, RUN_VMAX);
        if (!mm_wait_motion(1)) return 0;

        x += MAZE_DX[h] * plan[i].cells;
        y += MAZE_DY[h] * plan[i].cells;
        mm_set_pose(x, y, h);
    }

    mm_blink(5, 80);                                        /* toi GOAL */
    return 1;
}
