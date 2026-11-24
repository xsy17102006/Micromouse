/*
 * maze.h - Ban do me cung + thuat toan flood fill (port tu code Arduino)
 */
#ifndef MM_MAZE_H_
#define MM_MAZE_H_

#include <stdint.h>

#ifndef MAZE_W
#include "mm_config.h"
#endif

#define MAZE_N   (MAZE_W * MAZE_H)

#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

extern const int8_t MAZE_DX[4];
extern const int8_t MAZE_DY[4];

extern uint8_t maze_walls[MAZE_N];    /* bit d = 1: co tuong huong d       */
extern uint8_t maze_known[MAZE_N];    /* bit d = 1: da biet canh huong d   */
extern uint8_t maze_visited[MAZE_N];
extern uint8_t maze_flood[MAZE_N];    /* khoang cach (so o) toi dich       */

static inline uint8_t maze_idx(int x, int y) { return (uint8_t)(y * MAZE_W + x); }
static inline int maze_in(int x, int y) { return x >= 0 && x < MAZE_W && y >= 0 && y < MAZE_H; }

void maze_reset(void);
void maze_set_wall(int x, int y, int dir, int present);
int  maze_has_wall(int x, int y, int dir);
/* Di duoc tu (x,y) theo dir khong. unknownIsWall=1: canh chua biet coi nhu tuong */
int  maze_open(int x, int y, int dir, int unknownIsWall);
void maze_flood_fill(const uint8_t *tx, const uint8_t *ty, int nTargets, int unknownIsWall);
/* Chon huong di tiep (flood nho nhat, uu tien di thang). -1 neu ket. */
int  maze_next_dir(int x, int y, int heading, int unknownIsWall);
/* So o di thang lien tiep theo dir (doan "tunnel" nhu code goc). */
int  maze_run_length(int x, int y, int dir, int unknownIsWall, int onlyThroughVisited);

#endif
