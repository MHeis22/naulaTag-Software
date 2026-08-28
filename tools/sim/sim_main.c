/*
 * naulaTAG display simulator.
 *
 * Compiles the real src/display.c against stubbed panel hardware, runs it over
 * a set of sensor scenarios, and writes each resulting framebuffer out as a BMP
 * so the screen layout can be checked without the board.
 *
 * Usage:
 *   sim                     render every scenario into tools/sim/out/
 *   sim <name>              render one scenario
 *   sim <name> --ascii      also dump it to the terminal
 *   sim ... --scale N       pixel magnification (default 4)
 *   sim ... --out DIR       output directory
 *
 *   sim custom --temp 23.5 --humid 45 --volt 3.02
 */

#include <stdint.h>   /* display.h expects the fixed-width types to be in scope */

#include "display.h"
#include "sim_capture.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EPD_W 248
#define EPD_H 128

/*
 * E2206KS0E1 physical geometry, from the panel datasheet:
 *   viewing area  46.5 mm x 24.0 mm   -> 248/46.5 = 128/24 = 5.333 px/mm (135 dpi)
 *   module        57.75 mm x 29.5 mm  -> the surround is the plastic bezel
 * Used to render the module at true size and to report element heights in mm.
 */
#define VIEW_MM_W    46.5
#define VIEW_MM_H    24.0
#define MODULE_MM_W  57.75
#define MODULE_MM_H  29.5
#define PX_PER_MM    (EPD_W / VIEW_MM_W)

/* Bezel width per side, in panel pixels, so the module renders to scale. */
#define BEZEL_X ((int)(((MODULE_MM_W - VIEW_MM_W) / 2.0) * PX_PER_MM + 0.5))
#define BEZEL_Y ((int)(((MODULE_MM_H - VIEW_MM_H) / 2.0) * PX_PER_MM + 0.5))

/*
 * Two-colour panel (background colour: black, white), so one bit per pixel is
 * the whole story.  E-ink is never pure black on pure white, though — these
 * are closer to what the K-film actually looks like.
 */
#define INK_R   0x1C
#define INK_G   0x1C
#define INK_B   0x1A
#define PAPER_R 0xD9
#define PAPER_G 0xD8
#define PAPER_B 0xD0
#define BEZEL_R 0xE8
#define BEZEL_G 0xE8
#define BEZEL_B 0xE6

/* ── history generators ──────────────────────────────────────────────────── */

static int32_t hist[DISPLAY_HIST_SIZE];

/* 24 h of readings drifting around `base` with a diurnal swing of `swing`. */
static uint16_t hist_sine(int32_t base_mdeg, int32_t swing_mdeg, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        double phase = 2.0 * 3.14159265 * (double)i / (double)count;
        hist[i] = base_mdeg + (int32_t)(swing_mdeg * sin(phase));
    }
    return count;
}

static uint16_t hist_flat(int32_t v, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) hist[i] = v;
    return count;
}

/* Monotonic ramp — makes off-by-one errors at the graph edges obvious. */
static uint16_t hist_ramp(int32_t from, int32_t to, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
        hist[i] = from + (int32_t)((int64_t)(to - from) * i / (count > 1 ? count - 1 : 1));
    return count;
}

/* ── scenarios ───────────────────────────────────────────────────────────── */

typedef struct {
    const char *name;
    const char *what;      /* why this case is worth looking at */
    int32_t     temp_mdeg;
    uint32_t    humid_mpct;
    uint32_t    volt_mv;
    uint16_t    (*build_hist)(void);
} scenario_t;

static uint16_t h_room(void)   { return hist_sine(23500, 2500, DISPLAY_HIST_SIZE); }
static uint16_t h_cold(void)   { return hist_sine(-12300, 4000, DISPLAY_HIST_SIZE); }
static uint16_t h_hot(void)    { return hist_ramp(28000, 41700, DISPLAY_HIST_SIZE); }
static uint16_t h_none(void)   { return 0; }
static uint16_t h_one(void)    { return hist_flat(23500, 1); }
static uint16_t h_few(void)    { return hist_ramp(21000, 24000, 5); }
static uint16_t h_flat(void)   { return hist_flat(23500, DISPLAY_HIST_SIZE); }
static uint16_t h_spike(void)  { hist_flat(23500, DISPLAY_HIST_SIZE);
                                 hist[70] = 60000; hist[71] = -20000;
                                 return DISPLAY_HIST_SIZE; }

static const scenario_t scenarios[] = {
    { "room",  "typical indoor reading",              23500,  45000, 3024, h_room  },
    { "cold",  "negative temperature, minus sign",   -12300,  88800, 2710, h_cold  },
    { "hot",   "widest digits, low humidity",         41700,   9900, 3300, h_hot   },
    { "boot",  "first update, no history yet",        23500,  45000, 3024, h_none  },
    { "one",   "single history sample",               23500,  45000, 3024, h_one   },
    { "few",   "five samples, sparse graph",          22000,  50000, 3024, h_few   },
    { "flat",  "constant history, degenerate range",  23500,  45000, 3024, h_flat  },
    { "spike", "out-of-range excursions, clamping",   23500,  45000, 3024, h_spike },
    { "wide",  "widest strings the format can make", -100000, 100000, 3999, h_room },
    { "low",   "depleted battery",                    19100,  62400, 2201, h_room  },
};

/* ── output ──────────────────────────────────────────────────────────────── */

static int px_is_ink(const uint8_t *fb, int x, int y)
{
    int n = y * EPD_W + x;
    /* Panel convention: bit clear = ink. */
    return (fb[n / 8] & (1 << (7 - n % 8))) == 0;
}

/*
 * Minimal PNG writer.  Deflate "stored" blocks only — no zlib dependency, and
 * at this size the file is still small enough not to care.
 */

static uint32_t crc32_of(const uint8_t *p, size_t n, uint32_t crc)
{
    static uint32_t tab[256];
    if (!tab[1])
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            tab[i] = c;
        }
    crc ^= 0xFFFFFFFFu;
    while (n--) crc = tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF; p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;  p[3] = v & 0xFF;
}

static void png_chunk(FILE *f, const char *type, const uint8_t *data, size_t len)
{
    uint8_t hdr[4];
    put_be32(hdr, (uint32_t)len);
    fwrite(hdr, 1, 4, f);
    fwrite(type, 1, 4, f);
    if (len) fwrite(data, 1, len, f);

    uint32_t crc = crc32_of((const uint8_t *)type, 4, 0);
    if (len) crc = crc32_of(data, len, crc);
    put_be32(hdr, crc);
    fwrite(hdr, 1, 4, f);
}

static int write_png(const char *path, const uint8_t *fb, int scale)
{
    const int w = EPD_W * scale + 2 * BEZEL_X * scale;
    const int h = EPD_H * scale + 2 * BEZEL_Y * scale;
    const size_t raw_len = (size_t)h * (1 + (size_t)w * 3);   /* +1 filter byte */

    uint8_t *raw = malloc(raw_len);
    if (!raw) return -1;

    uint8_t *o = raw;
    for (int iy = 0; iy < h; iy++) {
        *o++ = 0;                                   /* filter: none */
        for (int ix = 0; ix < w; ix++) {
            int sx = ix - BEZEL_X * scale, sy = iy - BEZEL_Y * scale;
            if (sx < 0 || sy < 0 || sx >= EPD_W * scale || sy >= EPD_H * scale) {
                *o++ = BEZEL_R; *o++ = BEZEL_G; *o++ = BEZEL_B;
                continue;
            }
            int ink = px_is_ink(fb, sx / scale, sy / scale);
            *o++ = ink ? INK_R : PAPER_R;
            *o++ = ink ? INK_G : PAPER_G;
            *o++ = ink ? INK_B : PAPER_B;
        }
    }

    /* zlib stream: 0x78 0x01, then stored deflate blocks, then adler32. */
    size_t nblocks = (raw_len + 65534) / 65535;
    size_t z_len = 2 + nblocks * 5 + raw_len + 4;
    uint8_t *z = malloc(z_len);
    if (!z) { free(raw); return -1; }

    uint8_t *zo = z;
    *zo++ = 0x78; *zo++ = 0x01;
    for (size_t off = 0; off < raw_len; off += 65535) {
        size_t n = raw_len - off < 65535 ? raw_len - off : 65535;
        *zo++ = (off + n >= raw_len) ? 1 : 0;       /* BFINAL */
        *zo++ = n & 0xFF;         *zo++ = (n >> 8) & 0xFF;
        *zo++ = ~n & 0xFF;        *zo++ = (~n >> 8) & 0xFF;
        memcpy(zo, raw + off, n);
        zo += n;
    }
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < raw_len; i++) { a = (a + raw[i]) % 65521; b = (b + a) % 65521; }
    put_be32(zo, (b << 16) | a);
    zo += 4;

    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); free(raw); free(z); return -1; }

    static const uint8_t sig[8] = {0x89,'P','N','G','\r','\n',0x1A,'\n'};
    fwrite(sig, 1, 8, f);

    uint8_t ihdr[13] = {0};
    put_be32(ihdr, (uint32_t)w);
    put_be32(ihdr + 4, (uint32_t)h);
    ihdr[8] = 8;    /* 8 bits per channel */
    ihdr[9] = 2;    /* truecolour */
    png_chunk(f, "IHDR", ihdr, sizeof(ihdr));

    /* pHYs: tag the true panel density so the image prints at life size. */
    uint8_t phys[9] = {0};
    uint32_t ppm = (uint32_t)(PX_PER_MM * 1000.0 * scale + 0.5);
    put_be32(phys, ppm);
    put_be32(phys + 4, ppm);
    phys[8] = 1;                                    /* unit: metre */
    png_chunk(f, "pHYs", phys, sizeof(phys));

    png_chunk(f, "IDAT", z, (size_t)(zo - z));
    png_chunk(f, "IEND", NULL, 0);

    fclose(f);
    free(raw);
    free(z);
    return 0;
}

/* Half-block preview: one glyph per 2×2 pixels, so 248×128 fits a terminal. */
static void write_ascii(const uint8_t *fb)
{
    for (int y = 0; y < EPD_H; y += 2) {
        for (int x = 0; x < EPD_W; x += 2) {
            int top = px_is_ink(fb, x, y) || px_is_ink(fb, x + 1, y);
            int bot = px_is_ink(fb, x, y + 1) || px_is_ink(fb, x + 1, y + 1);
            fputs(top && bot ? "\xE2\x96\x88"       /* full  */
                  : top     ? "\xE2\x96\x80"       /* upper */
                  : bot     ? "\xE2\x96\x84"       /* lower */
                            : " ", stdout);
        }
        putchar('\n');
    }
}

/* ── driver ──────────────────────────────────────────────────────────────── */

static int render(const scenario_t *s, const char *outdir, int scale, int ascii)
{
    uint16_t count = s->build_hist();

    display_init();
    display_update(s->temp_mdeg, s->humid_mpct, s->volt_mv,
                   count ? hist : NULL, count, count % DISPLAY_HIST_SIZE);

    char path[512];
    snprintf(path, sizeof(path), "%s/%s.png", outdir, s->name);
    if (write_png(path, sim_frame, scale) != 0) return -1;

    int ink = 0;
    for (int y = 0; y < EPD_H; y++)
        for (int x = 0; x < EPD_W; x++) ink += px_is_ink(sim_frame, x, y);

    /* Panel is rated -15..+60 C; outside that the waveform is not guaranteed. */
    const char *warn = (sim_temp_reg < -15 || sim_temp_reg > 60)
                       ? "  [!] outside panel operating range" : "";

    printf("%-6s %-36s %s  (%s, temp reg %d C, %d%% ink)%s\n",
           s->name, s->what, path,
           sim_update_mode == EPD_UPDATE_FAST ? "fast" : "normal",
           sim_temp_reg, ink * 100 / (EPD_W * EPD_H), warn);

    if (ascii) write_ascii(sim_frame);
    return 0;
}

int main(int argc, char **argv)
{
    const char *only   = NULL;
    const char *outdir = "tools/sim/out";
    int scale = 4, ascii = 0;
    double c_temp = 23.5, c_humid = 45.0, c_volt = 3.02;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--ascii"))              ascii  = 1;
        else if (!strcmp(argv[i], "--scale") && i + 1 < argc) scale  = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--out")   && i + 1 < argc) outdir = argv[++i];
        else if (!strcmp(argv[i], "--temp")  && i + 1 < argc) c_temp  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--humid") && i + 1 < argc) c_humid = atof(argv[++i]);
        else if (!strcmp(argv[i], "--volt")  && i + 1 < argc) c_volt  = atof(argv[++i]);
        else if (argv[i][0] != '-')                        only   = argv[i];
        else { fprintf(stderr, "unknown option: %s\n", argv[i]); return 2; }
    }
    if (scale < 1) scale = 1;

    /* Text scales mirror the draw_string() calls in display.c. */
    printf("panel  %dx%d px, %.1fx%.1f mm viewing area, %.2f px/mm (%d dpi)\n",
           EPD_W, EPD_H, VIEW_MM_W, VIEW_MM_H, PX_PER_MM,
           (int)(PX_PER_MM * 25.4 + 0.5));
    printf("text   temperature %.1f mm tall, humidity %.1f mm, voltage %.1f mm\n",
           7 * 4 / PX_PER_MM, 7 * 3 / PX_PER_MM, 7 * 2 / PX_PER_MM);
    printf("images %dx magnified, tagged for life-size printing\n\n", scale);

    if (only && !strcmp(only, "custom")) {
        scenario_t s = { "custom", "command-line values",
                         (int32_t)(c_temp * 1000.0), (uint32_t)(c_humid * 1000.0),
                         (uint32_t)(c_volt * 1000.0), h_room };
        return render(&s, outdir, scale, ascii) == 0 ? 0 : 1;
    }

    int matched = 0;
    for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
        if (only && strcmp(only, scenarios[i].name)) continue;
        matched = 1;
        if (render(&scenarios[i], outdir, scale, ascii) != 0) return 1;
    }

    if (!matched) {
        fprintf(stderr, "no such scenario: %s\navailable:", only);
        for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++)
            fprintf(stderr, " %s", scenarios[i].name);
        fprintf(stderr, " custom\n");
        return 2;
    }
    return 0;
}
