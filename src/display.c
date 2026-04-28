/*
 * E2206KS0E1 display wrapper — C port of display.cpp.
 *
 * Pin IDs (must match epd_hal.c):
 *   EPD_PIN_BUSY = 0   EPD_PIN_DC = 1   EPD_PIN_CS = 2
 *
 * panelReset is NOT_CONNECTED — the load switch power-cycles the panel
 * before epd_begin() is called, achieving the same effect as a hard reset.
 */

#include "epd_cog.h"
#include "display.h"
#include "epd_hal.h"
#include "font5x7.h"

#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

/* ── Display dimensions ─────────────────────────────────────────────────── */

#define FB_SIZE  EPD_FRAME_SIZE          /* 7936 bytes */
#define EPD_W    248
#define EPD_H    (FB_SIZE * 8 / EPD_W)  /* 256 */

/* ── Driver state ───────────────────────────────────────────────────────── */

static epd_ctx_t epd_ctx;
static bool      epd_ready;

static uint8_t fb[FB_SIZE];
static uint8_t fb_prev[FB_SIZE];

/* ── Internal helpers ───────────────────────────────────────────────────── */

static void fb_clear(uint8_t *buf, uint8_t fill)
{
    memset(buf, fill, FB_SIZE);
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

/* ── Public API ─────────────────────────────────────────────────────────── */

int display_init(void)
{
    epd_begin(&epd_ctx, EPD_PIN_BUSY, EPD_PIN_DC, EPD_PIN_CS);
    epd_ready = true;
    fb_clear(fb_prev, 0xFF); /* all-white previous frame */
    return 0;
}

void display_update(int32_t temp_mdeg, uint32_t humid_mpct)
{
    if (!epd_ready) {
        return;
    }

    int8_t t_celsius = (int8_t)(temp_mdeg / 1000);

    epd_set_temperature(&epd_ctx, t_celsius);

    memcpy(fb_prev, fb, FB_SIZE);
    fb_clear(fb, 0xFF);

    /* Format temperature as "±XX.X" using integer arithmetic. */
    char temp_str[16];
    int  t_abs  = (int)(temp_mdeg < 0 ? -temp_mdeg : temp_mdeg);
    int  t_int  = t_abs / 1000;
    int  t_frac = (t_abs % 1000) / 100;

    if (temp_mdeg < 0) {
        snprintf(temp_str, sizeof(temp_str), "-%d.%d", t_int, t_frac);
    } else {
        snprintf(temp_str, sizeof(temp_str), "%d.%d", t_int, t_frac);
    }

    /* Format humidity as "XX.X%" using integer arithmetic. */
    char     humid_str[16];
    unsigned h_int  = humid_mpct / 1000;
    unsigned h_frac = (humid_mpct % 1000) / 100;

    snprintf(humid_str, sizeof(humid_str), "%u.%u%%", h_int, h_frac);

    /* Layout on 248×256 display:
     *   Row  16: temperature, scale 5 (35 px tall)
     *   Row  80: "C" label,   scale 3
     *   Row 140: humidity,    scale 4
     *   Row 196: "H" label,   scale 2
     */
    draw_string(fb, 8,  16, temp_str,  5);
    draw_string(fb, 8,  80, "C",       3);
    draw_string(fb, 8, 140, humid_str, 4);
    draw_string(fb, 8, 196, "H",       2);

    /* ── Update mode selection ──────────────────────────────────────────── */
    static uint32_t update_count;
    bool use_fast = true;

    /* Fast update is only valid between 0 °C and 50 °C. */
    if (t_celsius < 0 || t_celsius > 50) {
        use_fast = false;
    }

    /* Force a full normal update every ~24 h (144 × 10 min) to clear
     * ghosting.  The very first update (count == 0) is also forced normal,
     * which lays down the initial image cleanly. */
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
