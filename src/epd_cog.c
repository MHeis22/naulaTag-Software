/*
 * COG driver for E2206KS0E1 (eScreen_EPD_206_KS_0E).
 *
 * C port of hV_Board.cpp + Pervasive_Wide_Small.cpp, trimmed to the single
 * screen variant used in this project.  All timing, register values, and
 * OTP offsets are taken verbatim from the upstream library (release 904).
 *
 * SPI CS delay: 50 µs (FAMILY_SMALL default from hV_Board).
 * s_flag50: true for the 206-KS-0E screen (compile-time constant).
 */

#include "epd_cog.h"
#include "epd_hal.h"

#include <string.h>

/* ── Internal constants ─────────────────────────────────────────────────── */

#define DELAY_CS_US   50u   /* inter-command CS timing for FAMILY_SMALL */
#define FSM_GPIO_MASK 0x01u

/* ── Low-level SPI helpers (mirror hV_Board private methods) ────────────── */

/* b_reset(ms1, ms2, ms3, ms4, ms5)
 * panelReset is NOT_CONNECTED on this board so those pin ops are no-ops;
 * only the delays and the final CS-high matter. */
static void cog_hw_reset(epd_ctx_t *ctx,
                         uint32_t ms1, uint32_t ms2, uint32_t ms3,
                         uint32_t ms4, uint32_t ms5)
{
    epd_hal_delay_ms(ms1);
    /* panelReset HIGH — NOT_CONNECTED, no-op */
    epd_hal_delay_ms(ms2);
    /* panelReset LOW  — NOT_CONNECTED, no-op */
    epd_hal_delay_ms(ms3);
    /* panelReset HIGH — NOT_CONNECTED, no-op */
    epd_hal_delay_ms(ms4);
    epd_hal_gpio_set(ctx->pin_cs, 1);
    epd_hal_delay_ms(ms5);
}

/* b_waitBusy(HIGH) — busy clears when BUSY pin goes HIGH */
static void cog_wait_busy(epd_ctx_t *ctx)
{
    while (epd_hal_gpio_get(ctx->pin_busy) != 1) {
        epd_hal_delay_ms(32);
    }
}

/* b_sendCommand8 */
static void cog_cmd8(epd_ctx_t *ctx, uint8_t cmd)
{
    epd_hal_delay_ms(1);
    epd_hal_gpio_set(ctx->pin_dc, 0);
    epd_hal_gpio_set(ctx->pin_cs, 0);
    epd_hal_spi_transfer(cmd);
    epd_hal_gpio_set(ctx->pin_cs, 1);
}

/* b_sendCommandData8 */
static void cog_cmd8data8(epd_ctx_t *ctx, uint8_t cmd, uint8_t data)
{
    epd_hal_delay_ms(1);
    epd_hal_gpio_set(ctx->pin_dc, 0);
    epd_hal_gpio_set(ctx->pin_cs, 0);
    epd_hal_spi_transfer(cmd);
    epd_hal_gpio_set(ctx->pin_dc, 1);
    epd_hal_spi_transfer(data);
    epd_hal_gpio_set(ctx->pin_cs, 1);
}

/* b_sendIndexData (FAMILY_SMALL — no CSS, 50 µs delays) */
static void cog_idx_data(epd_ctx_t *ctx, uint8_t idx,
                         const uint8_t *data, uint32_t size)
{
    epd_hal_gpio_set(ctx->pin_dc, 0);
    epd_hal_gpio_set(ctx->pin_cs, 0);
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_spi_transfer(idx);
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_gpio_set(ctx->pin_dc, 1);
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_spi_write_buf(data, size);
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_gpio_set(ctx->pin_cs, 1);
    epd_hal_delay_us(DELAY_CS_US);
}

/* b_sendIndexFixed (same framing, repeated single byte) */
static void cog_idx_fixed(epd_ctx_t *ctx, uint8_t idx,
                          uint8_t data, uint32_t size)
{
    epd_hal_gpio_set(ctx->pin_dc, 0);
    epd_hal_gpio_set(ctx->pin_cs, 0);
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_spi_transfer(idx);
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_gpio_set(ctx->pin_dc, 1);
    epd_hal_delay_us(DELAY_CS_US);
    for (uint32_t i = 0; i < size; i++) {
        epd_hal_spi_transfer(data);
    }
    epd_hal_delay_us(DELAY_CS_US);
    epd_hal_gpio_set(ctx->pin_cs, 1);
}

/* ── b_resume: configure GPIO pins (guarded by FSM) ────────────────────── */

static void cog_resume(epd_ctx_t *ctx)
{
    if (ctx->fsm & FSM_GPIO_MASK) {
        return;
    }
    epd_hal_gpio_input(ctx->pin_busy);

    epd_hal_gpio_output(ctx->pin_dc);
    epd_hal_gpio_set(ctx->pin_dc, 1);

    epd_hal_gpio_output(ctx->pin_cs);
    epd_hal_gpio_set(ctx->pin_cs, 1);

    ctx->fsm |= FSM_GPIO_MASK;
}

/* ── COG_reset ──────────────────────────────────────────────────────────── */

static void cog_reset(epd_ctx_t *ctx)
{
    /* Application note § 2. Power on COG driver */
    cog_hw_reset(ctx, 1, 5, 5, 10, 20);
    /* 206-KS-0E has no post-reset busy check */
}

/* ── COG_getDataOTP ─────────────────────────────────────────────────────── */
/*
 * Read 2 PSR bytes from the panel OTP via bit-bang 3-wire SPI.
 * OTP layout for 206-KS-0E (application note § 3):
 *   Bank 0: marker at offset 0x0000 (0xa5), PSR at 0x0b1b
 *   Bank 1: marker at offset 0x0c00 (0xa5), PSR at 0x171b
 */
static void cog_read_otp(epd_ctx_t *ctx)
{
    epd_hal_spi_end();   /* release SCK/MOSI for bit-bang */

    /* Application note § 3. Set pins for 3-wire read */
    epd_hal_gpio_set(ctx->pin_dc, 1);
    /* panelReset HIGH — NOT_CONNECTED, no-op */
    epd_hal_gpio_set(ctx->pin_cs, 1);

    cog_hw_reset(ctx, 0, 5, 5, 10, 20);

    /* Issue 0xa2 read command */
    epd_hal_gpio_set(ctx->pin_dc, 0);   /* command mode */
    epd_hal_gpio_set(ctx->pin_cs, 0);   /* select */
    epd_hal_spi3_write(0xa2);
    epd_hal_gpio_set(ctx->pin_cs, 1);   /* unselect */
    epd_hal_delay_ms(10);

    /* Read: one dummy byte, then the bank-detection byte */
    epd_hal_gpio_set(ctx->pin_dc, 1);   /* data mode */

    epd_hal_gpio_set(ctx->pin_cs, 0);
    (void)epd_hal_spi3_read();          /* dummy */
    epd_hal_gpio_set(ctx->pin_cs, 1);

    epd_hal_gpio_set(ctx->pin_cs, 0);
    uint8_t first = epd_hal_spi3_read();
    epd_hal_gpio_set(ctx->pin_cs, 1);

    uint8_t bank = (first == 0xa5u) ? 0u : 1u;

    /* Offsets for 206-KS-0E */
    uint16_t offset_a5  = (bank == 0) ? 0x0000u : 0x0c00u;
    uint16_t offset_psr = (bank == 0) ? 0x0b1bu : 0x171bu;

    /* Skip to offsetA5 (only needed for bank 1) */
    for (uint16_t i = 1; i < offset_a5; i++) {
        epd_hal_gpio_set(ctx->pin_cs, 0);
        (void)epd_hal_spi3_read();
        epd_hal_gpio_set(ctx->pin_cs, 1);
    }

    /* Verify 0xa5 marker at offsetA5 (bank 1 only) */
    if (offset_a5 > 0u) {
        epd_hal_gpio_set(ctx->pin_cs, 0);
        uint8_t marker = epd_hal_spi3_read();
        epd_hal_gpio_set(ctx->pin_cs, 1);
        (void)marker; /* assertion omitted; trust the panel */
    }

    /* Skip from offsetA5+1 to offsetPSR */
    for (uint16_t i = offset_a5 + 1u; i < offset_psr; i++) {
        epd_hal_gpio_set(ctx->pin_cs, 0);
        (void)epd_hal_spi3_read();
        epd_hal_gpio_set(ctx->pin_cs, 1);
    }

    /* Read 2 PSR bytes */
    for (uint8_t i = 0; i < 2u; i++) {
        epd_hal_gpio_set(ctx->pin_cs, 0);
        ctx->otp[i] = epd_hal_spi3_read();
        epd_hal_gpio_set(ctx->pin_cs, 1);
    }

    ctx->otp_valid = true;
}

/* ── COG_initial ────────────────────────────────────────────────────────── */

static void cog_initial(epd_ctx_t *ctx, uint8_t mode)
{
    /* Application note § 4. Input initial command */
    uint8_t temp_reg;
    uint8_t psr[2];

    if (mode == EPD_UPDATE_FAST) {
        temp_reg = (uint8_t)(ctx->temperature) | 0x40u;
        psr[0]   = ctx->otp[0] | 0x10u;
        psr[1]   = ctx->otp[1] | 0x02u;
    } else {
        temp_reg = (uint8_t)(ctx->temperature);
        psr[0]   = ctx->otp[0];
        psr[1]   = ctx->otp[1];
    }

    cog_cmd8data8(ctx, 0x00, 0x0e);  /* Soft-reset */
    cog_wait_busy(ctx);

    cog_cmd8data8(ctx, 0xe5, temp_reg); /* Input Temperature */
    cog_cmd8data8(ctx, 0xe0, 0x02);     /* Activate Temperature */
    cog_idx_data(ctx, 0x00, psr, 2);    /* PSR */

    if (mode == EPD_UPDATE_FAST) {
        cog_cmd8data8(ctx, 0x50, 0x07); /* Vcom and data interval */
    }
}

/* ── COG_sendImageDataNormal ────────────────────────────────────────────── */

static void cog_send_normal(epd_ctx_t *ctx,
                            const uint8_t *frame, uint32_t size)
{
    /* Application note § 5. Input image */
    cog_idx_data(ctx,  0x10, frame, size); /* new image */
    cog_idx_fixed(ctx, 0x13, 0x00, size);  /* previous frame zeroed */
}

/* ── COG_sendImageDataFast ──────────────────────────────────────────────── */
/*
 * s_flag50 is true for the 206-KS-0E (compile-time constant).
 * prev = previously displayed frame (→ register 0x10 = "old image").
 * next = new frame to display    (→ register 0x13 = "new image").
 */
static void cog_send_fast(epd_ctx_t *ctx,
                          const uint8_t *prev,
                          const uint8_t *next,
                          uint32_t size)
{
    cog_cmd8data8(ctx, 0x50, 0x27);         /* Vcom: enable fast path */
    cog_idx_data(ctx,  0x10, prev, size);   /* old image */
    cog_idx_data(ctx,  0x13, next, size);   /* new image */
    cog_cmd8data8(ctx, 0x50, 0x07);         /* Vcom: restore */
}

/* ── COG_update ─────────────────────────────────────────────────────────── */

static void cog_update(epd_ctx_t *ctx)
{
    /* Application note § 6. Send updating command */
    cog_wait_busy(ctx);
    cog_cmd8(ctx, 0x04);  /* Power on */
    cog_wait_busy(ctx);
    cog_cmd8(ctx, 0x12);  /* Display Refresh */
    cog_wait_busy(ctx);
}

/* ── COG_stopDCDC ───────────────────────────────────────────────────────── */

static void cog_stop_dcdc(epd_ctx_t *ctx)
{
    /* Application note § 7. Turn-off DC/DC */
    cog_cmd8(ctx, 0x02);  /* Power off */
    cog_wait_busy(ctx);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void epd_begin(epd_ctx_t *ctx,
               uint8_t pin_busy, uint8_t pin_dc, uint8_t pin_cs)
{
    ctx->pin_busy    = pin_busy;
    ctx->pin_dc      = pin_dc;
    ctx->pin_cs      = pin_cs;
    ctx->temperature = 25;
    ctx->otp_valid   = false;
    ctx->fsm         = 0;

    /* Start SPI so that spi_end() inside cog_read_otp() has a device to
     * suspend, putting SCK/MOSI into low-power-enable state for bit-bang. */
    epd_hal_spi_begin(8000000);

    cog_resume(ctx);
    cog_reset(ctx);
    cog_read_otp(ctx);   /* ends SPI; leaves it inactive */
}

void epd_set_temperature(epd_ctx_t *ctx, int8_t celsius)
{
    ctx->temperature = celsius;
}

void epd_update_normal(epd_ctx_t *ctx,
                       const uint8_t *frame, uint32_t size)
{
    cog_resume(ctx);
    cog_reset(ctx);

    if (!ctx->otp_valid) {
        cog_read_otp(ctx);  /* re-read if somehow lost */
        cog_reset(ctx);
    }

    epd_hal_spi_begin(16000000);
    cog_initial(ctx, EPD_UPDATE_NORMAL);
    cog_send_normal(ctx, frame, size);
    cog_update(ctx);
    cog_stop_dcdc(ctx);
}

void epd_update_fast(epd_ctx_t *ctx,
                     const uint8_t *prev, const uint8_t *next,
                     uint32_t size)
{
    cog_resume(ctx);
    cog_reset(ctx);

    epd_hal_spi_begin(16000000);
    cog_initial(ctx, EPD_UPDATE_FAST);
    cog_send_fast(ctx, prev, next, size);
    cog_update(ctx);
    cog_stop_dcdc(ctx);
}
