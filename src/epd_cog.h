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

#define EPD_FRAME_SIZE  3968U

#define EPD_UPDATE_NORMAL  1
#define EPD_UPDATE_FAST    2

typedef struct {
    uint8_t pin_busy;
    uint8_t pin_dc;
    uint8_t pin_cs;
    uint8_t pin_reset;
    uint8_t otp[2];      /* PSR0 and PSR1 read from panel OTP */
    bool    otp_valid;
    int8_t  temperature; /* °C — used for waveform calibration */
    uint8_t fsm;         /* bit 0: GPIO pins have been configured */
} epd_ctx_t;

/*
 * All three of these return 0 on success, or a negative errno:
 *   -ETIMEDOUT  the panel never released BUSY
 *   -EIO        the OTP read did not return the expected marker
 * A failed update leaves the panel contents undefined, so the caller must force
 * a full refresh next time rather than trusting its previous-frame buffer.
 */
int epd_begin(epd_ctx_t *ctx,
              uint8_t pin_busy, uint8_t pin_dc, uint8_t pin_cs,
              uint8_t pin_reset);

void epd_set_temperature(epd_ctx_t *ctx, int8_t celsius);

int epd_update_normal(epd_ctx_t *ctx,
                      const uint8_t *frame, uint32_t size);

int epd_update_fast(epd_ctx_t *ctx,
                    const uint8_t *prev, const uint8_t *next,
                    uint32_t size);

/* Places pins into disconnected state and prepares context for next power-on */
void epd_sleep(epd_ctx_t *ctx);