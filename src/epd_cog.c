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

#include <errno.h>
#include <string.h>

/* ── Internal constants ─────────────────────────────────────────────────── */

#define DELAY_CS_US   50u   /* inter-command CS timing for FAMILY_SMALL */
#define FSM_GPIO_MASK 0x01u

/*
 * Upstream's b_waitBusy() polls forever.  That is tolerable on a tethered dev
 * board and not on an unattended battery device, so the wait is bounded here.
 * A full refresh at the panel's -15 C limit is seconds, not tens of seconds,
 * so this only trips when the panel is genuinely not responding.
 */
#define BUSY_POLL_MS     32u
#define BUSY_TIMEOUT_MS  20000u

/* ── Low-level SPI helpers (mirror hV_Board private methods) ────────────── */

/* b_reset(ms1, ms2, ms3, ms4, ms5)
 * RST_N is active-low, so raw 1 releases reset and raw 0 asserts it — the same
 * sense as upstream's GPIO_set / GPIO_clear on panelReset. */
static void cog_hw_reset(epd_ctx_t *ctx,
                         uint32_t ms1, uint32_t ms2, uint32_t ms3,
                         uint32_t ms4, uint32_t ms5)
{
    epd_hal_delay_ms(ms1);                      /* wait for power stabilisation */
    epd_hal_gpio_set(ctx->pin_reset, 1);
    epd_hal_delay_ms(ms2);
    epd_hal_gpio_set(ctx->pin_reset, 0);
    epd_hal_delay_ms(ms3);
    epd_hal_gpio_set(ctx->pin_reset, 1);
    epd_hal_delay_ms(ms4);
    epd_hal_gpio_set(ctx->pin_cs, 1);
    epd_hal_delay_ms(ms5);
}

/* b_waitBusy(HIGH) — busy clears when BUSY pin goes HIGH.
 * Returns 0 when the panel released BUSY, -ETIMEDOUT if it never did. */
static int cog_wait_busy(epd_ctx_t *ctx)
{
    for (uint32_t waited = 0; waited < BUSY_TIMEOUT_MS; waited += BUSY_POLL_MS) {
        if (epd_hal_gpio_get(ctx->pin_busy) == 1) {
            return 0;
        }
        epd_hal_delay_ms(BUSY_POLL_MS);
    }
    return (epd_hal_gpio_get(ctx->pin_busy) == 1) ? 0 : -ETIMEDOUT;
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

    epd_hal_gpio_output(ctx->pin_reset);
    epd_hal_gpio_set(ctx->pin_reset, 1);

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
static int cog_read_otp(epd_ctx_t *ctx)
{
    ctx->otp_valid = false;

    epd_hal_spi_end();   /* release SCK/MOSI for bit-bang */

    /* Application note § 3. Set pins for 3-wire read */
    epd_hal_gpio_set(ctx->pin_dc, 1);
    epd_hal_gpio_set(ctx->pin_reset, 1);
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

    /* Verify 0xa5 marker at offsetA5 (bank 1 only; for bank 0 the marker is
     * `first`, already checked above by the bank selection itself).
     * A mismatch means the OTP read is garbage — the PSR bytes would drive the
     * wrong waveform, so fail out rather than display something unpredictable. */
    if (offset_a5 > 0u) {
        epd_hal_gpio_set(ctx->pin_cs, 0);
        uint8_t marker = epd_hal_spi3_read();
        epd_hal_gpio_set(ctx->pin_cs, 1);
        if (marker != 0xa5u) {
            return -EIO;
        }
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
    return 0;
}

/* ── COG_initial ────────────────────────────────────────────────────────── */

static int cog_initial(epd_ctx_t *ctx, uint8_t mode)
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
    int rc = cog_wait_busy(ctx);
    if (rc) {
        return rc;
    }

    cog_cmd8data8(ctx, 0xe5, temp_reg); /* Input Temperature */
    cog_cmd8data8(ctx, 0xe0, 0x02);     /* Activate Temperature */
    cog_idx_data(ctx, 0x00, psr, 2);    /* PSR */

    if (mode == EPD_UPDATE_FAST) {
        cog_cmd8data8(ctx, 0x50, 0x07); /* Vcom and data interval */
    }
    return 0;
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

static int cog_update(epd_ctx_t *ctx)
{
    /* Application note § 6. Send updating command */
    int rc = cog_wait_busy(ctx);
    if (rc) {
        return rc;
    }

    cog_cmd8(ctx, 0x04);  /* Power on */
    rc = cog_wait_busy(ctx);
    if (rc) {
        return rc;
    }

    cog_cmd8(ctx, 0x12);  /* Display Refresh */
    return cog_wait_busy(ctx);
}

/* ── COG_stopDCDC ───────────────────────────────────────────────────────── */

static int cog_stop_dcdc(epd_ctx_t *ctx)
{
    /* Application note § 7. Turn-off DC/DC */
    cog_cmd8(ctx, 0x02);  /* Power off */
    return cog_wait_busy(ctx);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int epd_begin(epd_ctx_t *ctx,
              uint8_t pin_busy, uint8_t pin_dc, uint8_t pin_cs,
              uint8_t pin_reset)
{
    ctx->pin_busy    = pin_busy;
    ctx->pin_dc      = pin_dc;
    ctx->pin_cs      = pin_cs;
    ctx->pin_reset   = pin_reset;
    ctx->temperature = 25;
    ctx->otp_valid   = false;
    ctx->fsm         = 0;

    /* Start SPI so that spi_end() inside cog_read_otp() has a device to
     * suspend, putting SCK/MOSI into low-power-enable state for bit-bang. */
    epd_hal_spi_begin(8000000);

    cog_resume(ctx);
    cog_reset(ctx);
    return cog_read_otp(ctx);   /* ends SPI; leaves it inactive */
}

void epd_set_temperature(epd_ctx_t *ctx, int8_t celsius)
{
    ctx->temperature = celsius;
}

/*
 * On failure these return early with the DC/DC potentially still on.  That is
 * deliberate: the caller drops the load switch immediately afterwards, which is
 * a more reliable way out than issuing further commands to a panel that has
 * already stopped answering.
 */
int epd_update_normal(epd_ctx_t *ctx,
                      const uint8_t *frame, uint32_t size)
{
    cog_resume(ctx);
    cog_reset(ctx);

    if (!ctx->otp_valid) {
        int rc = cog_read_otp(ctx);  /* re-read if somehow lost */
        if (rc) {
            return rc;
        }
        cog_reset(ctx);
    }

    epd_hal_spi_begin(16000000);

    int rc = cog_initial(ctx, EPD_UPDATE_NORMAL);
    if (rc) {
        return rc;
    }
    cog_send_normal(ctx, frame, size);
    rc = cog_update(ctx);
    if (rc) {
        return rc;
    }
    return cog_stop_dcdc(ctx);
}

int epd_update_fast(epd_ctx_t *ctx,
                    const uint8_t *prev, const uint8_t *next,
                    uint32_t size)
{
    cog_resume(ctx);
    cog_reset(ctx);

    epd_hal_spi_begin(16000000);

    int rc = cog_initial(ctx, EPD_UPDATE_FAST);
    if (rc) {
        return rc;
    }
    cog_send_fast(ctx, prev, next, size);
    rc = cog_update(ctx);
    if (rc) {
        return rc;
    }
    return cog_stop_dcdc(ctx);
}

void epd_sleep(epd_ctx_t *ctx)
{
    /* Release SPI first: suspending the device re-applies the spi30 "sleep"
     * pinctrl state to SCK/MOSI, which would otherwise land after the GPIO
     * disconnect below. */
    epd_hal_spi_end();

    /* Float every panel-facing pin so nothing back-feeds the display once the
     * load switch opens. */
    epd_hal_pins_sleep();
    
    /* Clear the GPIO configured bit so cog_resume() re-initializes them on next update */
    ctx->fsm &= ~FSM_GPIO_MASK;
}