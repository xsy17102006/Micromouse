/*
 * maze.c - Flood fill cho me cung MAZE_W x MAZE_H
 *
 * Giu y tuong cua code Arduino goc:
 *   - Moi o luu: flood (khoang cach toi dich), tuong (4 bit), visited
 *   - Hang doi vong (CircularBufferQueue) de loang
 *   - Chon o ke co flood nho nhat, hoa thi uu tien: thang > re > quay dau
 *   - Di thang nhieu o mot lan qua cac o da tham ("tunnel")
 * Khac code goc: moi lan cap nhat tuong se loang lai toan bo tu dich (BFS).
 * Voi 24 o viec nay mat < 0.1 ms va khong bao gio sai.
 */
#include "maze.h"
#include <string.h>

const int8_t MAZE_DX[4] = { 0, 1, 0, -1 };
const int8_t MAZE_DY[4] = { 1, 0, -1, 0 };

uint8_t maze_walls[MAZE_N];
uint8_t maze_known[MAZE_N];
uint8_t maze_visited[MAZE_N];
uint8_t maze_flood[MAZE_N];

/* ---------- Hang doi vong (thay cho class CircularBufferQueue) ---------- */
#define QSIZE 64
typedef struct { uint8_t buf[QSIZE]; uint8_t head, tail, count; } cqueue_t;

static void q_init(cqueue_t *q) { q->head = q->tail = q->count = 0; }
static int  q_empty(const cqueue_t *q) { return q->count == 0; }
static int  q_push(cqueue_t *q, uint8_t v)
{
    if (q->count >= QSIZE) return 0;
    q->buf[q->tail] = v;
    q->tail = (uint8_t)((q->tail + 1) % QSIZE);
    q->count++;
    return 1;
}
static uint8_t q_pop(cqueue_t *q)
{
    uint8_t v = q->buf[q->head];
    q->head = (uint8_t)((q->head + 1) % QSIZE);
    q->count--;
    return v;
}

/* ---------------------------------------------------------------------- */
void maze_reset(void)
{
    memset(maze_walls, 0, sizeof maze_walls);
    memset(maze_known, 0, sizeof maze_known);
    memset(maze_visited, 0, sizeof maze_visited);
    for (int y = 0; y < MAZE_H; y++)
        for (int x = 0; x < MAZE_W; x++) {
            uint8_t i = maze_idx(x, y), b = 0;
            if (y == MAZE_H - 1) b |= 1 << DIR_N;
            if (x == MAZE_W - 1) b |= 1 << DIR_E;
            if (y == 0)          b |= 1 << DIR_S;
            if (x == 0)          b |= 1 << DIR_W;
            maze_walls[i] = b;
            maze_known[i] = b;
        }
}

void maze_set_wall(int x, int y, int dir, int present)
{
    if (!maze_in(x, y)) return;
    uint8_t i = maze_idx(x, y), bit = (uint8_t)(1 << dir);
    maze_known[i] |= bit;
    if (present) maze_walls[i] |= bit; else maze_walls[i] &= (uint8_t)~bit;

    int nx = x + MAZE_DX[dir], ny = y + MAZE_DY[dir];
    if (maze_in(nx, ny)) {                      /* canh do cung la cua o ben canh */
        uint8_t j = maze_idx(nx, ny), ob = (uint8_t)(1 << ((dir + 2) & 3));
        maze_known[j] |= ob;
        if (present) maze_walls[j] |= ob; else maze_walls[j] &= (uint8_t)~ob;
    }
}

int maze_has_wall(int x, int y, int dir)
{
    return (maze_walls[maze_idx(x, y)] >> dir) & 1;
}

int maze_open(int x, int y, int dir, int unknownIsWall)
{
    uint8_t i = maze_idx(x, y), bit = (uint8_t)(1 << dir);
    if (maze_walls[i] & bit) return 0;
    if (unknownIsWall && !(maze_known[i] & bit)) return 0;
    return maze_in(x + MAZE_DX[dir], y + MAZE_DY[dir]);
}

void maze_flood_fill(const uint8_t *tx, const uint8_t *ty, int nTargets, int unknownIsWall)
{
    cqueue_t q;
    q_init(&q);
    memset(maze_flood, 255, sizeof maze_flood);
    for (int k = 0; k < nTargets; k++) {
        uint8_t i = maze_idx(tx[k], ty[k]);
        maze_flood[i] = 0;
        q_push(&q, i);
    }
    while (!q_empty(&q)) {
        uint8_t c = q_pop(&q);
        int x = c % MAZE_W, y = c / MAZE_W;
        for (int d = 0; d < 4; d++) {
            if (!maze_open(x, y, d, unknownIsWall)) continue;
            uint8_t n = maze_idx(x + MAZE_DX[d], y + MAZE_DY[d]);
            if (maze_flood[n] == 255) {
                maze_flood[n] = (uint8_t)(maze_flood[c] + 1);
                q_push(&q, n);
            }
        }
    }
}

int maze_next_dir(int x, int y, int heading, int unknownIsWall)
{
    /* diem uu tien theo huong tuong doi: thang, phai, quay dau, trai
       (giong targetScoreFromDirection = {0,1,2,1} trong code goc) */
    static const uint8_t score[4] = { 0, 1, 2, 1 };
    int best = -1;
    uint8_t bestF = 255, bestS = 255;
    for (int d = 0; d < 4; d++) {
        if (!maze_open(x, y, d, unknownIsWall)) continue;
        uint8_t f = maze_flood[maze_idx(x + MAZE_DX[d], y + MAZE_DY[d])];
        uint8_t s = score[(d - heading + 4) & 3];
        if (f < bestF || (f == bestF && s < bestS)) { bestF = f; bestS = s; best = d; }
    }
    if (best < 0 || bestF == 255 || bestF >= maze_flood[maze_idx(x, y)]) return -1;
    return best;
}

int maze_run_length(int x, int y, int dir, int unknownIsWall, int onlyThroughVisited)
{
    int n = 1;
    int cx = x + MAZE_DX[dir], cy = y + MAZE_DY[dir];
    while (1) {
        uint8_t c = maze_idx(cx, cy);
        if (maze_flood[c] == 0) break;                         /* toi dich: dung   */
        if (onlyThroughVisited && !maze_visited[c]) break;     /* o moi: dung de do tuong */
        if (!maze_open(cx, cy, dir, unknownIsWall)) break;
        int nx = cx + MAZE_DX[dir], ny = cy + MAZE_DY[dir];
        if (maze_flood[maze_idx(nx, ny)] != (uint8_t)(maze_flood[c] - 1)) break;
        cx = nx; cy = ny; n++;
    }
    return n;
}
