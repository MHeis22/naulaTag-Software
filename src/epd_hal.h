/*
 * Hardware abstraction layer for the E2206KS0E1 e-ink display.
 *
 * Abstract pin IDs — must match the table in epd_hal.c:
 *   0 = BUSY   gpio1 pin 3   input
 *   1 = DC     gpio2 pin 0   output
 *   2 = CS     gpio2 pin 5   output, active-low
 *   3 = SCK    gpio2 pin 1   bit-bang SPI3 clock
 *   4 = MOSI   gpio2 pin 2   bit-bang SPI3 data (bidirectional)
 *   5 = RESET  gpio0 pin 0   output, active-low (RST_N)
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define EPD_PIN_BUSY    0
#define EPD_PIN_DC      1
#define EPD_PIN_CS      2
#define EPD_PIN_SCK     3
#define EPD_PIN_MOSI    4
#define EPD_PIN_RESET   5
#define EPD_NOT_CONNECTED 0xFF

void    epd_hal_gpio_output(uint8_t pin);
void    epd_hal_gpio_input(uint8_t pin);
void    epd_hal_gpio_set(uint8_t pin, int val);
int     epd_hal_gpio_get(uint8_t pin);

void    epd_hal_spi_begin(uint32_t hz);
void    epd_hal_spi_end(void);
uint8_t epd_hal_spi_transfer(uint8_t byte);
void    epd_hal_spi_write_buf(const uint8_t *buf, size_t len);

/* Bit-bang 3-wire SPI on EPD_PIN_SCK / EPD_PIN_MOSI (OTP read path). */
void    epd_hal_spi3_write(uint8_t byte);
uint8_t epd_hal_spi3_read(void);

void    epd_hal_delay_ms(uint32_t ms);
void    epd_hal_delay_us(uint32_t us);

void    epd_hal_pins_sleep(void);