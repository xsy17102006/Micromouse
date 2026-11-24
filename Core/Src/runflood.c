/*
 * runflood.c - MODE 5: CHAY START -> DICH bang FLOOD FILL (don gian, on dinh)
 *
 * Dung map da do o MODE 4 (con trong RAM), xe dat o START, mui START_HEADING.
 *   1. Flood fill tu dich, chi di qua canh DA BIET la thong (chua biet = tuong)
 *   2. Moi o: chon o ke co flood nho nhat (hoa thi uu tien di thang), quay, di 1 o
 *   3. Dung lai tung o, toc do EXPLORE_VMAX, khong do lai tuong
 * Cham nhung it sai so nhat. Muon nhanh hon dung MODE 6.
 */
#include "solver.h"
#include "mm_config.h"
#include "maze.h"
#include "motion.h"
#include "tof.h"

int runflood_run(void)
{
    if (dbgTofMask != 0x0F) { dbgErr = 2; return 0; }

    uint8_t gx = GOAL_X, gy = GOAL_Y;
    maze_flood_fill(&gx, &gy, 1, 1);                 /* 1: canh chua biet coi nhu tuong */
    if (maze_flood[maze_idx(START_X, START_Y)] == 255) { dbgErr = 1; return 0; }

    int x = START_X, y = START_Y, h = START_HEADING;
    mm_set_pose(x, y, h);
    dbgStep = 0;

    while (!(x == GOAL_X && y == GOAL_Y)) {
        int d = maze_next_dir(x, y, h, 1);
        if (d < 0) { dbgErr = 1; return 0; }

        if (!mm_turn_rel((d - h + 4) & 3)) return 0; /* 0 thang, 1 phai, 2 quay dau, 3 trai */
        h = d;

        motion_straight((float)CELL_TICKS, EXPLORE_VMAX);
        if (!mm_wait_motion(1)) return 0;            /* 1: bam tuong */

        x += MAZE_DX[d];
        y += MAZE_DY[d];
        mm_set_pose(x, y, h);
        dbgStep++;
    }

    mm_blink(5, 80);                                 /* toi GOAL */
    return 1;
}
