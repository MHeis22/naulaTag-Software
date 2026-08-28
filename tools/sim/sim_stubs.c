/*
 * Panel-side stubs for the host simulator.
 *
 * display.c is compiled verbatim; these replace epd_cog.c / epd_hal.c so the
 * COG command sequence and bit-banged SPI are skipped and the framebuffer the
 * driver would have shifted out is captured instead.
 */

#include "epd_cog.h"
#include "sim_capture.h"

#include <string.h>

uint8_t sim_frame[EPD_FRAME_SIZE];
uint8_t sim_prev[EPD_FRAME_SIZE];
int     sim_update_mode;
int8_t  sim_temp_reg;

int epd_begin(epd_ctx_t *ctx, uint8_t pin_busy, uint8_t pin_dc, uint8_t pin_cs,
              uint8_t pin_reset)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->pin_busy  = pin_busy;
    ctx->pin_dc    = pin_dc;
    ctx->pin_cs    = pin_cs;
    ctx->pin_reset = pin_reset;
    /* The panel OTP read is a hardware transaction; pretend it succeeded. */
    ctx->otp_valid = true;
    return 0;
}

void epd_set_temperature(epd_ctx_t *ctx, int8_t celsius)
{
    ctx->temperature = celsius;
    sim_temp_reg     = celsius;
}

int epd_update_normal(epd_ctx_t *ctx, const uint8_t *frame, uint32_t size)
{
    (void)ctx;
    memcpy(sim_frame, frame, size);
    sim_update_mode = EPD_UPDATE_NORMAL;
    return 0;
}

int epd_update_fast(epd_ctx_t *ctx, const uint8_t *prev, const uint8_t *next,
                    uint32_t size)
{
    (void)ctx;
    memcpy(sim_prev, prev, size);
    memcpy(sim_frame, next, size);
    sim_update_mode = EPD_UPDATE_FAST;
    return 0;
}

void epd_sleep(epd_ctx_t *ctx) { (void)ctx; }
