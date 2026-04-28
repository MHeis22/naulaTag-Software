/*
 * COG (Controller On Glass) driver for the E2206KS0E1 2.06" K-film e-ink
 * display (eScreen_EPD_206_KS_0E).
 *
 * C port of:
 *   external/PDLS_Common/src/hV_Board.cpp
 *   external/Pervasive_Wide_Small/src/Pervasive_Wide_Small.cpp
 *
 * Only the 206-KS-0E code paths are retained; multi-panel and other screen
 * variants have been removed.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Framebuffer size for the 2.06" screen: 248 × 128 / 8 = 3968 bytes. */
#define EPD_FRAME_SIZE  3968U

#define EPD_UPDATE_NORMAL  1
#define EPD_UPDATE_FAST    2

typedef struct {
    uint8_t pin_busy;
    uint8_t pin_dc;
    uint8_t pin_cs;
    uint8_t otp[2];      /* PSR0 and PSR1 read from panel OTP */
    bool    otp_valid;
    int8_t  temperature; /* °C — used for waveform calibration */
    uint8_t fsm;         /* bit 0: GPIO pins have been configured */
} epd_ctx_t;

/*
 * Initialise GPIO pins and read the panel OTP.
 * Must be called while the load switch (power rail) is ON.
 */
void epd_begin(epd_ctx_t *ctx,
               uint8_t pin_busy, uint8_t pin_dc, uint8_t pin_cs);

/* Update the temperature register (°C).  Call before each update. */
void epd_set_temperature(epd_ctx_t *ctx, int8_t celsius);

/* Full normal refresh — best quality, ~2 s. */
void epd_update_normal(epd_ctx_t *ctx,
                       const uint8_t *frame, uint32_t size);

/*
 * Fast differential refresh — prev is the previously displayed frame,
 * next is the new frame.  ~0.5 s.  Only valid between 0 °C and 50 °C.
 */
void epd_update_fast(epd_ctx_t *ctx,
                     const uint8_t *prev, const uint8_t *next,
                     uint32_t size);
