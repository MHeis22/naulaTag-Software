/*
 * E2206KS0E1 display wrapper.
 *
 * Panel convention: 0 = black (ink), 1 = white (no ink).
 * The COG driver sends framebuffer bytes to the panel verbatim — no inversion.
 * fb_clear(buf, 0xFF) therefore produces an all-white background.
 *
 * Screen layout (248 × 128 px):
 *   y=  8  scale 5  temperature   "±XX.X"          (35 px tall)
 *   y=  8  scale 2  voltage       "X.XXV"   (right-aligned, 14 px tall)
 *   y= 50  scale 3  "C"                            (21 px tall)
 *   y= 80  scale 4  humidity      "XX.X%"          (28 px tall)
 *   y=115  scale 2  "H"                            (14 px tall)
 *   y=131            horizontal separator line
 *   y=134 … 252      24-hour temperature trend graph (118 px tall)
 */

#include "epd_cog.h"
#include "display.h"
#include "epd_hal.h"
#include "font5x7.h"

#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

/* ── Display dimensions ─────────────────────────────────────────────────── */

#define FB_SIZE  EPD_FRAME_SIZE   /* 3968 bytes */
#define EPD_W    248
#define EPD_H    (FB_SIZE * 8 / EPD_W)   /* 128 */

/* ── Graph region ───────────────────────────────────────────────────────── */
#define GRAPH_X_L   2
#define GRAPH_X_R   (EPD_W - 3)          /* 245 */
#define GRAPH_Y_T   64                   /* Moved up from 134 */
#define GRAPH_Y_B   125                  /* Moved up from 252 */
#define GRAPH_W     (GRAPH_X_R - GRAPH_X_L)   /* 243 */
#define GRAPH_H     (GRAPH_Y_B - GRAPH_Y_T)   /* 61 */

/* ── Driver state ───────────────────────────────────────────────────────── */

static epd_ctx_t epd_ctx;
static bool      epd_ready;

static uint8_t fb[FB_SIZE];
static uint8_t fb_prev[FB_SIZE];

/* ── Low-level pixel primitives ─────────────────────────────────────────── */

static void fb_clear(uint8_t *buf, uint8_t fill)
{
    memset(buf, fill, FB_SIZE);
}

/* Set a single pixel black (0) on the framebuffer.
 * Out-of-bounds writes are silently dropped. */
static void draw_pixel(uint8_t *buf, int x, int y)
{
    if ((unsigned)x >= (unsigned)EPD_W || (unsigned)y >= (unsigned)EPD_H) {
        return;
    }
    int n = y * EPD_W + x;
    buf[n / 8] &= ~(1 << (7 - n % 8));   /* clear bit = 0 = black */
}

/* Bresenham line between (x0,y0) and (x1,y1). */
static void draw_line(uint8_t *buf, int x0, int y0, int x1, int y1)
{
    int dx  = x1 > x0 ? x1 - x0 : x0 - x1;
    int dy  = y1 > y0 ? y1 - y0 : y0 - y1;
    int sx  = x0 < x1 ? 1 : -1;
    int sy  = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        draw_pixel(buf, x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* Draw ASCII string at pixel (x, y) with integer scale. */
static void draw_string(uint8_t *buf, int x, int y, const char *s, int scale)
{
    int cx = x;
    while (*s) {
        int idx = font_idx(*s);
        font_draw_char(buf, EPD_W, cx, y, idx, scale, 1 /* black */);
        cx += (FONT5X7_WIDTH + 1) * scale;
        s++;
    }
}

/* ── Graph rendering ────────────────────────────────────────────────────── */

static void draw_graph(uint8_t *buf,
                       const int32_t *hist, uint16_t count, uint16_t head)
{
    if (count == 0) {
        return;
    }

    /* Oldest entry index in the circular buffer */
    uint16_t oldest = (uint16_t)((head + DISPLAY_HIST_SIZE - count)
                                 % DISPLAY_HIST_SIZE);

    /* Auto-scale: find min and max temperature in the history */
    int32_t t_min = hist[oldest];
    int32_t t_max = t_min;
    for (uint16_t i = 1; i < count; i++) {
        int32_t t = hist[(oldest + i) % DISPLAY_HIST_SIZE];
        if (t < t_min) t_min = t;
        if (t > t_max) t_max = t;
    }

    /* Avoid degenerate (flat line) case by adding ±0.5 °C padding */
    if (t_max == t_min) {
        t_min -= 500;
        t_max += 500;
    }
    int32_t t_range = t_max - t_min;

    /* Plot points and connect them with lines */
    int prev_x = -1;
    int prev_y = -1;
    for (uint16_t i = 0; i < count; i++) {
        int32_t t = hist[(oldest + i) % DISPLAY_HIST_SIZE];

        int x = GRAPH_X_L + (count > 1
                              ? (int32_t)i * GRAPH_W / (count - 1)
                              : 0);
        int y = GRAPH_Y_B - (int32_t)(t - t_min) * GRAPH_H / t_range;

        /* Clamp to graph area */
        if (y < GRAPH_Y_T) y = GRAPH_Y_T;
        if (y > GRAPH_Y_B) y = GRAPH_Y_B;

        if (prev_x >= 0) {
            draw_line(buf, prev_x, prev_y, x, y);
        } else {
            draw_pixel(buf, x, y);
        }
        prev_x = x;
        prev_y = y;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int display_init(void)
{
    epd_begin(&epd_ctx, EPD_PIN_BUSY, EPD_PIN_DC, EPD_PIN_CS);
    epd_ready = true;
    fb_clear(fb_prev, 0xFF); /* all-white previous frame */
    return 0;
}

void display_update(int32_t temp_mdeg, uint32_t humid_mpct,
                    uint32_t voltage_mv,
                    const int32_t *temp_hist, uint16_t hist_count,
                    uint16_t hist_head)
{
    if (!epd_ready) {
        return;
    }

    int8_t t_celsius = (int8_t)(temp_mdeg / 1000);
    epd_set_temperature(&epd_ctx, t_celsius);

    memcpy(fb_prev, fb, FB_SIZE);
    fb_clear(fb, 0xFF);

    /* ── Temperature string "±XX.X" ──────────────────────────────────────── */
    char temp_str[16];
    int  t_abs  = (int)(temp_mdeg < 0 ? -temp_mdeg : temp_mdeg);
    int  t_int  = t_abs / 1000;
    int  t_frac = (t_abs % 1000) / 100;

    if (temp_mdeg < 0) {
        snprintf(temp_str, sizeof(temp_str), "-%d.%d", t_int, t_frac);
    } else {
        snprintf(temp_str, sizeof(temp_str), "%d.%d", t_int, t_frac);
    }

    /* ── Humidity string "XX.X%" ─────────────────────────────────────────── */
    char     humid_str[16];
    unsigned h_int  = humid_mpct / 1000;
    unsigned h_frac = (humid_mpct % 1000) / 100;
    snprintf(humid_str, sizeof(humid_str), "%u.%u%%", h_int, h_frac);

    /* ── Voltage string "X.XXV" ──────────────────────────────────────────── */
    char volt_str[16]; // Changed from 8 to 16 to satisfy GCC
    uint32_t v_int  = voltage_mv / 1000;
    uint32_t v_frac = (voltage_mv % 1000) / 10;   /* two decimal places */
    snprintf(volt_str, sizeof(volt_str), "%u.%02uV", v_int, v_frac);

/* ── Render text ─────────────────────────────────────────────────────── */
    draw_string(fb, 8,   4, temp_str,  4);  /* Scaled down to 4, moved up */
    draw_string(fb, 8,  36, humid_str, 3);  /* Scaled down to 3, moved up */
    
    /* Voltage: right-aligned at top-right */
    int volt_x = EPD_W - 8 - font_string_width(volt_str, 2);
    draw_string(fb, volt_x, 4, volt_str, 2);

    /* Separator line above graph */
    draw_line(fb, 0, 60, EPD_W - 1, 60);    /* Moved up to 60 */

    /* ── Trend graph ─────────────────────────────────────────────────────── */
    draw_graph(fb, temp_hist, hist_count, hist_head);

    /* ── Update mode selection ──────────────────────────────────────────── */
    static uint32_t update_count;
    bool use_fast = true;

    if (t_celsius < 0 || t_celsius > 50) {
        use_fast = false;
    }

    /* Force a full normal update every ~24 h (144 × 10 min) to clear ghosting.
     * The very first update (count == 0) is also forced normal. */
    if (update_count % 144 == 0) {
        use_fast = false;
    }

    if (use_fast) {
        epd_update_fast(&epd_ctx, fb_prev, fb, FB_SIZE);
    } else {
        epd_update_normal(&epd_ctx, fb, FB_SIZE);
    }

    update_count++;
}
