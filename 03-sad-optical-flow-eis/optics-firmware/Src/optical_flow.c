/**
 * optical_flow.c
 * ──────────────
 * Full pipeline for STM32F303RET6:
 *
 *  1. Variance check  → skip flat/uninformative blocks
 *  2. SAD search      → find best displacement per block (USAD8 SIMD)
 *  3. Histogram vote  → reject outlier motion vectors
 *  4. Subpixel refine → parabolic interpolation for ×4 accuracy
 *  5. Kalman filter   → smooth vx, vy over time
 */

#include "optical_flow.h"
#include <stdint.h>
#include <string.h>

/* ═══════════════════════════════════════════════
 * ARM Cortex-M4 USAD8 — sum of abs diff, 4 bytes
 * single-cycle instruction on M4 with DSP ext.
 * ═══════════════════════════════════════════════ */
#ifdef __ARM_FEATURE_SIMD32
#warning "USAD8 ENABLED"
#else
#warning "USAD8 NOT ENABLED"
#endif
static inline uint32_t sad4(uint32_t a, uint32_t b)
{
    uint32_t r;
    __asm volatile("usad8 %0,%1,%2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}

/* ═══════════════════════════════════════════════
 * KALMAN FILTER  (float, F303 has FPU → free)
 * ═══════════════════════════════════════════════ */
static KF kfx, kfy;

/* ── EIS crop state — persists across calls (like Kalman) ── */
static float eis_crop_x = (float)EIS_MARGIN_X;
static float eis_crop_y = (float)EIS_MARGIN_Y;

static void kf_init(KF *k, float Q, float R)
{
    k->x = 0.0f;
    k->P = 1.0f;
    k->Q = Q;
    k->R = R;
}

static float kf_update(KF *k, float z)
{
    /* Predict */
    float P_ = k->P + k->Q;
    /* Update */
    float K  = P_ / (P_ + k->R);
    k->x    += K * (z - k->x);
    k->P     = (1.0f - K) * P_;
    return k->x;
}

/* ═══════════════════════════════════════════════
 * BLOCK VARIANCE  (integer, no division trick)
 * ═══════════════════════════════════════════════ */
static uint32_t variance(
    const uint8_t f[OF_H][OF_W],
    int bx, int by)
{
    uint32_t sum = 0, sum2 = 0;

    for (int i = 0; i < BLOCK_SIZE; i++)
        for (int j = 0; j < BLOCK_SIZE; j++) {
            uint32_t p = f[by+i][bx+j];
            sum  += p;
            sum2 += p * p;
        }

    /* N = BLOCK_SIZE*BLOCK_SIZE = 64
     * var = (sum2 - sum²/64) / 64
     * avoid division: compare sum2*64 vs sum²  */
    const uint32_t N = BLOCK_SIZE * BLOCK_SIZE; /* 64 */
    return (sum2 - (sum * sum) / N) / N;
}

/* ═══════════════════════════════════════════════
 * BLOCK SAD  (USAD8, processes 4 pixels/cycle)
 * ═══════════════════════════════════════════════ */
static uint32_t block_sad(
    const uint8_t f1[OF_H][OF_W],
    const uint8_t f2[OF_H][OF_W],
    int bx, int by,
    int dx, int dy,
	uint32_t best_so_far)
{
    uint32_t s  = 0;
    int      x2 = bx + dx;
    int      y2 = by + dy;

    for (int i = 0; i < BLOCK_SIZE; i++) {
        const uint8_t *r1 = &f1[by+i][bx];
        const uint8_t *r2 = &f2[y2+i][x2];
        /* 8 pixels per row, 4 at a time → 2 USAD8 calls */
        s += sad4(*(uint32_t*)(r1+0), *(uint32_t*)(r2+0));
        s += sad4(*(uint32_t*)(r1+4), *(uint32_t*)(r2+4));

        /* Early termination */
                if (s >= best_so_far)
                    return s;
    }
    return s;
}

/* ═══════════════════════════════════════════════
 * SUBPIXEL REFINEMENT  (parabolic interpolation)
 * Given SAD values at dx-1, dx, dx+1 → float peak
 * Multiplied by 4 and returned as int16 (×4 scale)
 * ═══════════════════════════════════════════════ */
static int16_t subpixel(int32_t sm1, int32_t s0, int32_t sp1, int integer_pos)
{
    int32_t denom = 2 * (sm1 - 2*s0 + sp1);
    if (denom == 0) return (int16_t)(integer_pos * 4);
    /* peak = integer_pos - (sp1-sm1)/(2*(sm1-2*s0+sp1)) */
    /* ×4 fixed point */
    int32_t num   = (sm1 - sp1) * 4;
    int32_t frac  = num / denom;          /* in units of 0.25px */
    return (int16_t)(integer_pos * 4 + frac);
}

/* ═══════════════════════════════════════════════
 * HISTOGRAM FILTER
 * Build histogram of dx (or dy) values.
 * Return the weighted mean of the dominant bin±1.
 * Also return confidence = peak_count / total × 100
 * ═══════════════════════════════════════════════ */
static int16_t histogram_mean(
    const int8_t *vals,   /* dx or dy values          */
    const int16_t *subvals,/* subpixel (×4) version    */
    int            n,
    uint8_t       *confidence)
{
    int32_t hist[HIST_SIZE] = {0};

    for (int i = 0; i < n; i++) {
        int idx = (int)vals[i] + HIST_OFFSET;
        if (idx >= 0 && idx < HIST_SIZE)
            hist[idx]++;
    }

    /* Find peak */
    int peak = 0;
    for (int i = 1; i < HIST_SIZE; i++)
        if (hist[i] > hist[peak]) peak = i;

    /* Weighted mean of peak ± 1 using subpixel values */
    int32_t sum_sub = 0, cnt = 0;
    for (int i = 0; i < n; i++) {
        int idx = (int)vals[i] + HIST_OFFSET;
        if (idx >= peak-1 && idx <= peak+1) {
            sum_sub += subvals[i];
            cnt++;
        }
    }

    /* Confidence = fraction of blocks in peak bin */
    *confidence = (uint8_t)((cnt * 100) / n);

    if (cnt == 0) return 0;
    return (int16_t)(sum_sub / cnt);   /* subpixel ×4 */
}

/* ═══════════════════════════════════════════════
 * STATIC STORAGE  (avoid stack allocation)
 * ═══════════════════════════════════════════════ */
static int8_t  best_dx[MAX_BLOCKS];
static int8_t  best_dy[MAX_BLOCKS];
static int16_t sub_dx[MAX_BLOCKS];   /* subpixel ×4 */
static int16_t sub_dy[MAX_BLOCKS];

/* Precomputed SAD table for subpixel (3 positions) */
static uint32_t sad_at[3];   /* sad[dx-1], sad[dx], sad[dx+1] */

/* ═══════════════════════════════════════════════
 * PUBLIC: INIT
 * ═══════════════════════════════════════════════ */
void of_init(void)
{
    /*
     * Q = process noise  — how fast true velocity changes
     * R = measurement noise — how noisy SAD output is
     *
     * For indoor drone hover:
     *   Q=0.3 (slow velocity changes)
     *   R=3.0 (SAD is ~10× noisier than model)
     *
     * For faster motion or webcam:
     *   Increase Q to 1.0–2.0
     */
    kf_init(&kfx, 0.3f, 3.0f);
    kf_init(&kfy, 0.3f, 3.0f);
    of_reset_eis();
}

/* ═══════════════════════════════════════════════
 * PUBLIC: RESET EIS CROP POSITION
 * ═══════════════════════════════════════════════ */
void of_reset_eis(void)
{
    eis_crop_x = (float)EIS_MARGIN_X;
    eis_crop_y = (float)EIS_MARGIN_Y;
}

/* ═══════════════════════════════════════════════
 * PUBLIC: COMPUTE FLOW
 * ═══════════════════════════════════════════════ */
void of_compute(
    const uint8_t f1[OF_H][OF_W],
    const uint8_t f2[OF_H][OF_W],
    FlowResult *out)
{
    int vcnt = 0;   /* valid block counter */

    /* Safe region: block + search radius must stay inside frame */
    const int xs = SEARCH_RADIUS;
    const int ys = SEARCH_RADIUS;
    const int xe = OF_W - BLOCK_SIZE - SEARCH_RADIUS;
    const int ye = OF_H - BLOCK_SIZE - SEARCH_RADIUS;

    /* ─── Step 1 & 2: Variance + SAD search ─── */
    for (int by = ys; by < ye; by += BLOCK_SIZE) {
        for (int bx = xs; bx < xe; bx += BLOCK_SIZE) {

            /* Step 1: Variance check — skip flat blocks */
            if (variance(f1, bx, by) < VARIANCE_THRESH)
                continue;

            /* Step 2: Full SAD search over ±SEARCH_RADIUS */
            uint32_t min_sad = 0xFFFFFFFFUL;
            int      bx_best = 0, by_best = 0;

            for (int dy = -SEARCH_RADIUS; dy <= SEARCH_RADIUS; dy++) {
                for (int dx = -SEARCH_RADIUS; dx <= SEARCH_RADIUS; dx++) {
                    uint32_t s = block_sad(f1, f2, bx, by, dx, dy, min_sad);
                    if (s < min_sad) {
                        min_sad = s;
                        bx_best = dx;
                        by_best = dy;
                    }
                }
            }

            /* Step 3: Subpixel refinement (parabolic along x) */
            int16_t spx, spy;

            /* X direction */
            if (bx_best > -SEARCH_RADIUS && bx_best < SEARCH_RADIUS) {
                uint32_t sm = block_sad(f1,f2,bx,by, bx_best-1, by_best, 0xFFFFFFFFUL);
                uint32_t s0 = min_sad;
                uint32_t sp = block_sad(f1,f2,bx,by, bx_best+1, by_best, 0xFFFFFFFFUL);
                spx = subpixel((int32_t)sm,(int32_t)s0,(int32_t)sp, bx_best);
            } else {
                spx = (int16_t)(bx_best * 4);
            }

            /* Y direction */
            if (by_best > -SEARCH_RADIUS && by_best < SEARCH_RADIUS) {
                uint32_t sm = block_sad(f1,f2,bx,by, bx_best, by_best-1, 0xFFFFFFFFUL);
                uint32_t s0 = min_sad;
                uint32_t sp = block_sad(f1,f2,bx,by, bx_best, by_best+1, 0xFFFFFFFFUL);
                spy = subpixel((int32_t)sm,(int32_t)s0,(int32_t)sp, by_best);
            } else {
                spy = (int16_t)(by_best * 4);
            }

            /* Store */
            if (vcnt < MAX_BLOCKS) {
                best_dx[vcnt] = (int8_t)bx_best;
                best_dy[vcnt] = (int8_t)by_best;
                sub_dx[vcnt]  = spx;
                sub_dy[vcnt]  = spy;
                vcnt++;
            }
        }
    }

    out->valid_blocks = (uint8_t)(vcnt > 255 ? 255 : vcnt);

    if (vcnt < MIN_VALID_BLOCKS) {
        /* Not enough texture — output zero, don't update Kalman */
        out->vx         = 0;
        out->vy         = 0;
        out->confidence = 0;
        return;
    }

    /* ─── Step 4: Histogram outlier filter + subpixel mean ─── */
    uint8_t conf_x, conf_y;
    int16_t raw_vx = histogram_mean(best_dx, sub_dx, vcnt, &conf_x);
    int16_t raw_vy = histogram_mean(best_dy, sub_dy, vcnt, &conf_y);

    out->confidence = (uint8_t)((conf_x + conf_y) / 2);

    /* ─── Step 5: Kalman filter ─── */
    /* raw_vx is in ×4 subpixel units; Kalman works in same units */
    float svx = kf_update(&kfx, (float)raw_vx);
    float svy = kf_update(&kfy, (float)raw_vy);

    out->vx = (int16_t)svx;   /* subpixel ×4 */
    out->vy = (int16_t)svy;

    /* ─── Step 6: EIS — shift crop window to cancel motion ───
     * svx/svy are in subpixel ×4 units; convert to real pixels
     * before applying to the crop position. Subtracting the
     * flow cancels the detected camera motion (if the scene
     * moved +vx, shift the viewing window by -vx to compensate). */
    float real_vx = svx / 4.0f;
    float real_vy = svy / 4.0f;

    eis_crop_x -= real_vx;
    eis_crop_y -= real_vy;

    /* Clamp to valid range so the crop window never reads
     * outside the 96x96 source frame */
    if (eis_crop_x < 0.0f)
        eis_crop_x = 0.0f;
    if (eis_crop_x > (float)(OF_W - EIS_OUT_W))
        eis_crop_x = (float)(OF_W - EIS_OUT_W);

    if (eis_crop_y < 0.0f)
        eis_crop_y = 0.0f;
    if (eis_crop_y > (float)(OF_H - EIS_OUT_H))
        eis_crop_y = (float)(OF_H - EIS_OUT_H);

    out->eis_crop_x = (int16_t)(eis_crop_x + 0.5f);  /* round */
    out->eis_crop_y = (int16_t)(eis_crop_y + 0.5f);
}
