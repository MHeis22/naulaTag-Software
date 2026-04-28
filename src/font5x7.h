/*
 * 5×7 pixel bitmap font — digits 0-9, '.', '-', '%', '°', 'C', 'H', space.
 * Each character is 5 columns × 7 rows, packed as 5 bytes (one byte per column,
 * bit 0 = top row).  Bit set = black pixel.
 */
#pragma once
#include <stdint.h>

#define FONT5X7_WIDTH  5
#define FONT5X7_HEIGHT 7

/* Character index helper */
#define FONT_IDX_SPACE   0
#define FONT_IDX_MINUS   1
#define FONT_IDX_DOT     2
#define FONT_IDX_PERCENT 3
#define FONT_IDX_DEGREE  4
#define FONT_IDX_C       5
#define FONT_IDX_H       6
#define FONT_IDX_0      10
#define FONT_IDX_1      11
#define FONT_IDX_2      12
#define FONT_IDX_3      13
#define FONT_IDX_4      14
#define FONT_IDX_5      15
#define FONT_IDX_6      16
#define FONT_IDX_7      17
#define FONT_IDX_8      18
#define FONT_IDX_9      19

/* 20 glyphs: space minus dot % ° C H 0..9 */
static const uint8_t font5x7[20][FONT5X7_WIDTH] = {
    /* 0 space */   {0x00, 0x00, 0x00, 0x00, 0x00},
    /* 1 minus */   {0x08, 0x08, 0x08, 0x08, 0x08},
    /* 2 dot   */   {0x00, 0x00, 0x60, 0x60, 0x00},
    /* 3 %     */   {0x46, 0x26, 0x10, 0x48, 0x64},
    /* 4 °     */   {0x06, 0x09, 0x09, 0x06, 0x00},
    /* 5 C     */   {0x3E, 0x41, 0x41, 0x41, 0x22},
    /* 6 H     */   {0x7F, 0x08, 0x08, 0x08, 0x7F},
    /* 7 (unused)*/ {0x00, 0x00, 0x00, 0x00, 0x00},
    /* 8 (unused)*/ {0x00, 0x00, 0x00, 0x00, 0x00},
    /* 9 (unused)*/ {0x00, 0x00, 0x00, 0x00, 0x00},
    /* 10 0    */   {0x3E, 0x51, 0x49, 0x45, 0x3E},
    /* 11 1    */   {0x00, 0x42, 0x7F, 0x40, 0x00},
    /* 12 2    */   {0x42, 0x61, 0x51, 0x49, 0x46},
    /* 13 3    */   {0x21, 0x41, 0x45, 0x4B, 0x31},
    /* 14 4    */   {0x18, 0x14, 0x12, 0x7F, 0x10},
    /* 15 5    */   {0x27, 0x45, 0x45, 0x45, 0x39},
    /* 16 6    */   {0x3C, 0x4A, 0x49, 0x49, 0x30},
    /* 17 7    */   {0x01, 0x71, 0x09, 0x05, 0x03},
    /* 18 8    */   {0x36, 0x49, 0x49, 0x49, 0x36},
    /* 19 9    */   {0x06, 0x49, 0x49, 0x29, 0x1E},
};

/* Map ASCII character to font index, returns FONT_IDX_SPACE for unknowns */
static inline int font_idx(char c)
{
    if (c >= '0' && c <= '9') return FONT_IDX_0 + (c - '0');
    switch (c) {
    case ' ': return FONT_IDX_SPACE;
    case '-': return FONT_IDX_MINUS;
    case '.': return FONT_IDX_DOT;
    case '%': return FONT_IDX_PERCENT;
    case 0xC2: /* UTF-8 degree ° is 0xC2 0xB0 — handle via caller */
    case 0xB0: return FONT_IDX_DEGREE;
    case 'C': return FONT_IDX_C;
    case 'H': return FONT_IDX_H;
    default:  return FONT_IDX_SPACE;
    }
}

/*
 * Draw a single scaled character into a 1bpp framebuffer.
 *
 * fb         : framebuffer, row-major, 1bpp (bit 7 of byte = leftmost pixel)
 * fb_width   : display width in pixels
 * x, y       : top-left corner in pixels
 * idx        : font index (use font_idx() or FONT_IDX_* constants)
 * scale      : pixel magnification (1 = normal, 4 = 4× larger)
 * black      : 1 = draw black, 0 = draw white
 */
static inline void font_draw_char(uint8_t *fb, int fb_width,
                                  int x, int y, int idx, int scale, int black)
{
    for (int col = 0; col < FONT5X7_WIDTH; col++) {
        uint8_t col_data = font5x7[idx][col];
        for (int row = 0; row < FONT5X7_HEIGHT; row++) {
            int bit = (col_data >> row) & 1;
            if (bit == 0) continue; /* background — leave white */
            for (int sy = 0; sy < scale; sy++) {
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx;
                    int py = y + row * scale + sy;
                    int byte_idx = (py * fb_width + px) / 8;
                    int bit_idx  = 7 - ((py * fb_width + px) % 8);
                    if (black)
                        fb[byte_idx] |=  (1 << bit_idx);
                    else
                        fb[byte_idx] &= ~(1 << bit_idx);
                }
            }
        }
    }
}

/* Returns pixel width of a string at given scale (including inter-char gap) */
static inline int font_string_width(const char *s, int scale)
{
    int len = 0;
    while (*s++) len++;
    return len * (FONT5X7_WIDTH + 1) * scale;
}
