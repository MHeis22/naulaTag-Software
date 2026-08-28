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
#include <errno.h>
#include <string.h>
#include <stdio.h>

#define FB_SIZE  EPD_FRAME_SIZE
#define EPD_W    248
#define EPD_H    128

#define GRAPH_X_L   2
#define GRAPH_X_R   245
#define GRAPH_Y_T   64
#define GRAPH_Y_B   125
#define GRAPH_W     243
#define GRAPH_H     61

static epd_ctx_t epd_ctx;
static bool      epd_ready;
/* Set whenever the panel contents stop being knowable — at startup, and after
 * any failed update — so the next update repaints the whole screen. */
static bool      force_full_refresh = true;

static uint8_t fb[FB_SIZE];
static uint8_t fb_prev[FB_SIZE];

static void fb_clear(uint8_t *buf, uint8_t fill) { memset(buf, fill, FB_SIZE); }

static void draw_pixel(uint8_t *buf, int x, int y) {
    if ((unsigned)x >= (unsigned)EPD_W || (unsigned)y >= (unsigned)EPD_H) return;
    int n = y * EPD_W + x;
    buf[n / 8] &= ~(1 << (7 - n % 8));
}

static void draw_line(uint8_t *buf, int x0, int y0, int x1, int y1) {
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

static void draw_string(uint8_t *buf, int x, int y, const char *s, int scale) {
    int cx = x;
    while (*s) {
        int idx = font_idx(*s);
        font_draw_char(buf, EPD_W, cx, y, idx, scale, 1);
        cx += (FONT5X7_WIDTH + 1) * scale;
        s++;
    }
}

static void draw_graph(uint8_t *buf, const int32_t *hist, uint16_t count, uint16_t head) {
    if (count == 0) return;

    uint16_t oldest = (uint16_t)((head + DISPLAY_HIST_SIZE - count) % DISPLAY_HIST_SIZE);

    int32_t t_min = hist[oldest];
    int32_t t_max = t_min;
    for (uint16_t i = 1; i < count; i++) {
        int32_t t = hist[(oldest + i) % DISPLAY_HIST_SIZE];
        if (t < t_min) t_min = t;
        if (t > t_max) t_max = t;
    }

    if (t_max == t_min) {
        t_min -= 500;
        t_max += 500;
    }
    int32_t t_range = t_max - t_min;

    int prev_x = -1, prev_y = -1;
    for (uint16_t i = 0; i < count; i++) {
        int32_t t = hist[(oldest + i) % DISPLAY_HIST_SIZE];

        int x = GRAPH_X_L + (count > 1 ? (int32_t)i * GRAPH_W / (count - 1) : 0);
        int y = GRAPH_Y_B - (int32_t)(t - t_min) * GRAPH_H / t_range;

        if (y < GRAPH_Y_T) y = GRAPH_Y_T;
        if (y > GRAPH_Y_B) y = GRAPH_Y_B;

        if (prev_x >= 0) draw_line(buf, prev_x, prev_y, x, y);
        else draw_pixel(buf, x, y);

        prev_x = x;
        prev_y = y;
    }
}

int display_init(void) {
    int rc = epd_begin(&epd_ctx, EPD_PIN_BUSY, EPD_PIN_DC, EPD_PIN_CS,
                       EPD_PIN_RESET);
    epd_ready = (rc == 0);
    if (rc) {
        return rc;
    }
    fb_clear(fb_prev, 0xFF);
    return 0;
}

int display_update(int32_t temp_mdeg, uint32_t humid_mpct,
                   uint32_t voltage_mv,
                   const int32_t *temp_hist, uint16_t hist_count,
                   uint16_t hist_head)
{
    if (!epd_ready) return -ENODEV;

    int8_t t_celsius = (int8_t)(temp_mdeg / 1000);
    epd_set_temperature(&epd_ctx, t_celsius);

    memcpy(fb_prev, fb, FB_SIZE);
    fb_clear(fb, 0xFF);

    char temp_str[16];
    int  t_abs  = (int)(temp_mdeg < 0 ? -temp_mdeg : temp_mdeg);
    snprintf(temp_str, sizeof(temp_str), "%s%d.%d", temp_mdeg < 0 ? "-" : "", t_abs / 1000, (t_abs % 1000) / 100);

    char     humid_str[16];
    snprintf(humid_str, sizeof(humid_str), "%u.%u%%", humid_mpct / 1000, (humid_mpct % 1000) / 100);

    /* Fix: Safe and correct voltage decimal rounding */
    char volt_str[16]; 
    uint32_t v_rounded = (voltage_mv + 5) / 10;
    snprintf(volt_str, sizeof(volt_str), "%u.%02uV", v_rounded / 100, v_rounded % 100);

    draw_string(fb, 8,   4, temp_str,  4);
    draw_string(fb, 8,  36, humid_str, 3);
    
    int volt_x = EPD_W - 8 - font_string_width(volt_str, 2);
    draw_string(fb, volt_x, 4, volt_str, 2);

    draw_line(fb, 0, 60, EPD_W - 1, 60);

    draw_graph(fb, temp_hist, hist_count, hist_head);

    static uint32_t update_count;
    /* A fast update differentiates against fb_prev, so it is only valid while
     * the panel really holds that frame.  After a failed update it does not. */
    bool use_fast = !force_full_refresh &&
                    (t_celsius >= 0 && t_celsius <= 50) &&
                    (update_count % 144 != 0);

    int rc;
    if (use_fast) rc = epd_update_fast(&epd_ctx, fb_prev, fb, FB_SIZE);
    else          rc = epd_update_normal(&epd_ctx, fb, FB_SIZE);

    force_full_refresh = (rc != 0);
    if (rc) {
        return rc;
    }

    update_count++;
    return 0;
}

void display_power_off(void) {
    /* Unconditional: the caller drops the load switch straight after this, and
     * any pin left driven then back-feeds the unpowered panel through its input
     * ESD clamps.  epd_begin() configures those pins as outputs before it can
     * fail, so a failed display_init() — epd_ready == false — is exactly the
     * case that must still be cleaned up. */
    if (epd_ready) {
        epd_sleep(&epd_ctx);
    } else {
        epd_hal_spi_end();
        epd_hal_pins_sleep();
    }
}