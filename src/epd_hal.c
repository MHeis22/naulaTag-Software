/*
 * Zephyr implementation of the EPD hardware abstraction layer.
 * Replaces src/hal/hal_impl.cpp and external hV_HAL_Peripherals.cpp.
 */

#include "epd_hal.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/pm/device.h>

/* ── Pin table ──────────────────────────────────────────────────────────── */

struct pin_entry {
    const struct device *port;
    gpio_pin_t           pin;
};

#define N_PINS 5
static struct pin_entry pin_table[N_PINS];
static bool pin_table_ready;

static void pin_table_init(void)
{
    if (pin_table_ready) {
        return;
    }
    const struct device *g1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    const struct device *g2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));

    pin_table[0] = (struct pin_entry){g1, 3};  /* BUSY  */
    pin_table[1] = (struct pin_entry){g2, 0};  /* DC    */
    pin_table[2] = (struct pin_entry){g2, 5};  /* CS    */
    pin_table[3] = (struct pin_entry){g2, 1};  /* SCK   */
    pin_table[4] = (struct pin_entry){g2, 2};  /* MOSI  */

    pin_table_ready = true;
}

/* ── GPIO ───────────────────────────────────────────────────────────────── */

void epd_hal_gpio_output(uint8_t pin)
{
    if (pin == EPD_NOT_CONNECTED || pin >= N_PINS) {
        return;
    }
    pin_table_init();
    gpio_pin_configure(pin_table[pin].port, pin_table[pin].pin,
                       GPIO_OUTPUT_INACTIVE);
}

void epd_hal_gpio_input(uint8_t pin)
{
    if (pin == EPD_NOT_CONNECTED || pin >= N_PINS) {
        return;
    }
    pin_table_init();
    gpio_pin_configure(pin_table[pin].port, pin_table[pin].pin, GPIO_INPUT);
}

void epd_hal_gpio_set(uint8_t pin, int val)
{
    if (pin == EPD_NOT_CONNECTED || pin >= N_PINS) {
        return;
    }
    pin_table_init();
    gpio_pin_set_raw(pin_table[pin].port, pin_table[pin].pin, val ? 1 : 0);
}

int epd_hal_gpio_get(uint8_t pin)
{
    if (pin == EPD_NOT_CONNECTED || pin >= N_PINS) {
        return 1; /* assume non-busy */
    }
    pin_table_init();
    return gpio_pin_get_raw(pin_table[pin].port, pin_table[pin].pin);
}

/* ── Hardware SPI ───────────────────────────────────────────────────────── */

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi30));

static struct spi_config spi_cfg = {
    .frequency = 8000000,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .slave     = 0,
    .cs        = {},  /* CS managed by software */
};

static bool spi_active;

void epd_hal_spi_begin(uint32_t hz)
{
    if (!spi_active) {
        spi_cfg.frequency = hz;
        pm_device_action_run(spi_dev, PM_DEVICE_ACTION_RESUME);
        spi_active = true;
    }
}

void epd_hal_spi_end(void)
{
    if (spi_active) {
        pm_device_action_run(spi_dev, PM_DEVICE_ACTION_SUSPEND);
        spi_active = false;
    }
}

uint8_t epd_hal_spi_transfer(uint8_t byte)
{
    uint8_t rx = 0;
    struct spi_buf    tx_buf = {.buf = &byte, .len = 1};
    struct spi_buf    rx_buf = {.buf = &rx,   .len = 1};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
    spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    return rx;
}

void epd_hal_spi_write_buf(const uint8_t *buf, size_t len)
{
    struct spi_buf     tx_buf = {.buf = (void *)buf, .len = len};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    spi_write(spi_dev, &spi_cfg, &tx_set);
}

/* ── Bit-bang 3-wire SPI (OTP read path) ───────────────────────────────── */
/*
 * After epd_hal_spi_end() the SPI peripheral releases SCK/MOSI to the
 * low-power-enable pinctrl state.  epd_hal_gpio_output/input then reclaim
 * them as plain GPIO for the bit-bang sequence.
 */

void epd_hal_spi3_write(uint8_t val)
{
    epd_hal_gpio_output(EPD_PIN_SCK);
    epd_hal_gpio_output(EPD_PIN_MOSI);
    for (int i = 7; i >= 0; i--) {
        epd_hal_gpio_set(EPD_PIN_MOSI, !!(val & (1 << i)));
        epd_hal_delay_us(1);
        epd_hal_gpio_set(EPD_PIN_SCK, 1);
        epd_hal_delay_us(1);
        epd_hal_gpio_set(EPD_PIN_SCK, 0);
        epd_hal_delay_us(1);
    }
}

uint8_t epd_hal_spi3_read(void)
{
    uint8_t val = 0;
    epd_hal_gpio_output(EPD_PIN_SCK);
    epd_hal_gpio_input(EPD_PIN_MOSI);
    for (int i = 7; i >= 0; i--) {
        epd_hal_gpio_set(EPD_PIN_SCK, 1);
        epd_hal_delay_us(1);
        if (epd_hal_gpio_get(EPD_PIN_MOSI)) {
            val |= (uint8_t)(1u << i);
        }
        epd_hal_gpio_set(EPD_PIN_SCK, 0);
        epd_hal_delay_us(1);
    }
    return val;
}

/* ── Timing ─────────────────────────────────────────────────────────────── */

void epd_hal_delay_ms(uint32_t ms)
{
    k_sleep(K_MSEC(ms));
}

void epd_hal_delay_us(uint32_t us)
{
    k_busy_wait(us);
}
