/**
 * optical_flow.h
 * ──────────────
 * SAD block matching optical flow for STM32F303RET6
 *
 * RAM budget (64KB total):
 *   frame1:        9,216 B
 *   frame2:        9,216 B
 *   best_dx/dy:      288 B  (144 blocks × 2)
 *   HAL + stack:  ~4,000 B
 *   Total used:  ~22,720 B  ← leaves 41KB free
 */

#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdint.h>

/* ── Image size ── */
#define OF_W          96
#define OF_H          96
#define OF_FRAME_SIZE (OF_W * OF_H)   /* 9216 bytes */

/* ── Block matching ── */
#define BLOCK_SIZE        8    /* 8×8 px blocks                  */
#define SEARCH_RADIUS     6   /* search ±4 px → 9×9 = 81 positions */
#define GRID_COLS         (OF_W / BLOCK_SIZE)          /* 12 */
#define GRID_ROWS         (OF_H / BLOCK_SIZE)          /* 12 */
#define MAX_BLOCKS        (GRID_COLS * GRID_ROWS)      /* 144 */

/* ── Thresholds ── */
#define VARIANCE_THRESH   15   /* skip blocks below this variance   */
#define MIN_VALID_BLOCKS  5    /* need at least this many for output */

/* ── Histogram ── */
#define HIST_SIZE         (2 * SEARCH_RADIUS + 1)  /* 9 bins */
#define HIST_OFFSET       SEARCH_RADIUS             /* value+offset = index */

/* ── EIS (Electronic Image Stabilization) ── */
#define EIS_OUT_W      80   /* stabilized output window width  */
#define EIS_OUT_H      80   /* stabilized output window height */
#define EIS_MARGIN_X   ((OF_W - EIS_OUT_W) / 2)   /* 8 px each side */
#define EIS_MARGIN_Y   ((OF_H - EIS_OUT_H) / 2)

/* ── Output packet ── */
typedef struct {
    int16_t  vx;           /* smoothed horizontal flow (subpixel ×4) */
    int16_t  vy;           /* smoothed vertical   flow (subpixel ×4) */
    uint8_t  valid_blocks; /* number of blocks that passed variance   */
    uint8_t  confidence;   /* 0-100: histogram peak strength          */

    /* EIS crop origin for THIS frame — read these to know where
     * to crop the ORIGINAL (non-resized) image on the PC side
     * for visualization. Values are in 96x96-space pixels. */
    int16_t  eis_crop_x;   /* 0 .. (OF_W - EIS_OUT_W)  */
    int16_t  eis_crop_y;   /* 0 .. (OF_H - EIS_OUT_H)  */
} FlowResult;

/* ── Kalman state (one per axis) ── */
typedef struct {
    float x;   /* estimate  */
    float P;   /* uncertainty */
    float Q;   /* process noise  */
    float R;   /* measurement noise */
} KF;

/* ── Public API ── */
void of_init(void);
void of_compute(
    const uint8_t f1[OF_H][OF_W],
    const uint8_t f2[OF_H][OF_W],
    FlowResult *out);

/**
 * Reset the EIS crop position back to centered.
 * Call this once at startup, or whenever starting a new
 * sequence of frame pairs (so accumulated drift doesn't carry
 * over from an unrelated previous test).
 */
void of_reset_eis(void);

#endif
