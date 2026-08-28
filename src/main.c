/*
 * naulaTAG — IoT temperature/humidity/light meter
 *
 * Hardware: nRF54L15-QFAA-R7
 * SRButton  (P0.01) — immediate measurement
 * BLEButton (P1.07) — interactive BLE window
 * HDC3022 (I2C 0x45) — temperature + humidity
 * OPT3005 (I2C 0x44) — ambient light, single-shot (INT on P1.06 unused)
 * E2206KS0E1 (SPI)   — 2.06" e-ink display
 * LoadSwitch (P1.04) — display power rail
 *
 * Power strategy:
 * CPU sleeps indefinitely (k_sleep(K_FOREVER)) and the idle thread drops the
 * core into System ON IDLE via WFI, with RAM and the GRTC retained.  Note that
 * CONFIG_PM is deliberately off: this SoC defines no DT power-states in NCS
 * 3.3, so the PM subsystem would have nothing deeper to select, and the 24 h
 * history plus the 10 min GRTC wakeup both need RAM and the timer alive.
 * Wakeup sources: GPIO interrupt (buttons), GRTC timer (periodic measurement).
 * Display is powered only during screen updates and skipped entirely when
 * ambient light is below DARK_THRESH_LOWER_LUX.
 * BLE beacons a short BTHome burst after each measurement; the button opens a
 * connectable window.  See ble.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/watchdog.h>

#include <stdlib.h>

#include "ble.h"
#include "display.h"
#include "storage.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ── Measurement interval ─────────────────────────────────────────────── */
#define MEASURE_INTERVAL_S  600

/* Button debounce.  Each contact bounce re-arms the delay, so the handler runs
 * once, this long after the last edge.  Without it a bounce arriving while the
 * handler is already running re-queues it — which on the BLE button toggles
 * advertising on and straight back off within a single press. */
#define BUTTON_DEBOUNCE_MS  50

/* Watchdog window.  Fed once per measurement rather than from a periodic tick:
 * a dedicated feeder would have to wake the CPU far more often than the 10 min
 * measurement interval, which is exactly the cost this design avoids.  The
 * margin over MEASURE_INTERVAL_S covers a slow display refresh plus the panel
 * BUSY timeout. */
#define WDT_TIMEOUT_S       (MEASURE_INTERVAL_S + 300)

/* Ambient light hysteresis thresholds (lux).  Hysteresis keeps the display from
 * refreshing every other cycle when the room sits right at the boundary. */
#define DARK_THRESH_LOWER_LUX  3U
#define DARK_THRESH_UPPER_LUX  7U

/* ── OPT3005 conversion control ───────────────────────────────────────── */
/*
 * The Zephyr opt300x driver puts the sensor in CONTINUOUS conversion at init and
 * exposes no way to change that — no PM hooks, no attribute for the mode.  Left
 * alone it converts 24/7 for 1.8 uA in the dark and 3.7 uA in bright light,
 * against 0.3 uA shut down.  On a CR2 that is one of the largest standby terms
 * in the design, and the reading is only used once every 10 minutes.
 *
 * So the mode is driven directly over I2C here.  Config register 0x01, 16-bit
 * big-endian:
 *   [15:12] RN = 0xC  automatic full-scale ranging
 *   [11]    CT = 1    800 ms conversion (better low-light resolution than the
 *                     100 ms setting; the CPU sleeps through it, so it is free)
 *   [10:9]  M         00 shutdown, 01 single-shot, 11 continuous
 *   [4]     L  = 1    latched window comparator (register reset default)
 *
 * Single-shot returns the device to shutdown by itself once the conversion
 * completes, so only the boot-time write is needed to undo the driver's
 * continuous mode; after that each cycle just kicks one conversion.
 */
#define OPT3005_I2C_ADDR        0x44
#define OPT3005_REG_CONFIG      0x01
#define OPT3005_CFG_SHUTDOWN    0xC810
#define OPT3005_CFG_SINGLE_SHOT 0xCA10
/* 800 ms conversion plus margin for the sensor's internal timing. */
#define OPT3005_CONV_WAIT_MS    900

static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c21));

static int opt3005_write_config(uint16_t cfg)
{
    /* Register pointer then the value, MSB first. */
    uint8_t buf[3] = {
        OPT3005_REG_CONFIG,
        (uint8_t)(cfg >> 8),
        (uint8_t)(cfg & 0xFF),
    };

    return i2c_write(i2c_bus, buf, sizeof(buf), OPT3005_I2C_ADDR);
}

/* ── Device handles ───────────────────────────────────────────────────── */
static const struct gpio_dt_spec sr_btn =
    GPIO_DT_SPEC_GET(DT_ALIAS(sr_button), gpios);
static const struct gpio_dt_spec ble_btn =
    GPIO_DT_SPEC_GET(DT_ALIAS(ble_button), gpios);
static const struct gpio_dt_spec load_sw =
    GPIO_DT_SPEC_GET(DT_ALIAS(epd_load_sw), gpios);

static const struct device *hdc = DEVICE_DT_GET(DT_NODELABEL(hdc3022));
static const struct device *opt = DEVICE_DT_GET(DT_NODELABEL(opt3005));
static const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt31));

/* Negative until the watchdog is installed and running. */
static int wdt_channel = -1;

/* ── Work items ────────────────────────────────────────────────────────── */
/*
 * A measurement holds its thread for the whole panel refresh — seconds, and up
 * to the BUSY timeout when the panel misbehaves.  It therefore runs on its own
 * queue, so a button press does not sit behind it.  The priority matches the
 * system workqueue's, keeping preemption behaviour identical; only the queueing
 * changes.
 */
#define MEASURE_WQ_STACK_SIZE  2048

K_THREAD_STACK_DEFINE(measure_wq_stack, MEASURE_WQ_STACK_SIZE);
static struct k_work_q measure_wq;

static struct k_work_delayable measure_work;     /* on measure_wq            */
static struct k_work_delayable ble_button_work;  /* on the system workqueue  */

/* ── Timers ────────────────────────────────────────────────────────────── */
static struct k_timer measure_timer;

/* ── ADC (battery voltage via internal VDD channel) ───────────────────── */
static const struct adc_dt_spec adc_vdd =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* ── Measurement state ─────────────────────────────────────────────────── */
static int32_t  last_temp_mdeg  = 0;
static uint32_t last_humid_mpct = 0;
static uint32_t last_lux        = 0;   /* integer lux */
static uint32_t last_voltage_mv = 0;

/* ── 24-hour temperature history (circular buffer) ─────────────────────── */
#define HISTORY_SIZE  DISPLAY_HIST_SIZE   /* 144 × 10 min = 24 h */

static int32_t  temp_hist[HISTORY_SIZE];
static uint16_t hist_head  = 0;   /* next write position    */
static uint16_t hist_count = 0;   /* valid entries, 0..144  */

/* ── Measurement ──────────────────────────────────────────────────────── */
static void do_measure(struct k_work *w)
{
    (void)w;
    int rc;

    /* 0. Feed the watchdog.  Reaching here proves the workqueue is still
     * scheduling; if anything below wedges it, this never runs again and the
     * SoC resets.  Fed at the top so the early return for darkness is covered
     * too. */
    if (wdt_channel >= 0) {
        (void)wdt_feed(wdt, wdt_channel);
    }

    /* 1. Kick the OPT3005 conversion first, so its 800 ms runs concurrently
     * with the reads below instead of serialising after them. */
    bool opt_started = (opt3005_write_config(OPT3005_CFG_SINGLE_SHOT) == 0);
    int64_t opt_kicked_at = k_uptime_get();

    if (!opt_started) {
        LOG_WRN("OPT3005 single-shot trigger failed");
    }

    /* 2. Read temperature and humidity. */
    struct sensor_value temp, humid;

    rc = sensor_sample_fetch(hdc);
    if (rc) {
        LOG_ERR("HDC fetch failed: %d", rc);
    } else {
        sensor_channel_get(hdc, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(hdc, SENSOR_CHAN_HUMIDITY,     &humid);
        last_temp_mdeg  = temp.val1  * 1000 + temp.val2  / 1000;
        last_humid_mpct = humid.val1 * 1000 + humid.val2 / 1000;
    }

    /* 3. Read battery voltage via internal SAADC VDD channel. */
    (void)pm_device_action_run(adc_vdd.dev, PM_DEVICE_ACTION_RESUME);
    {
        int16_t adc_raw = 0;
        struct adc_sequence adc_seq = {
            .buffer      = &adc_raw,
            .buffer_size = sizeof(adc_raw),
        };
        adc_sequence_init_dt(&adc_vdd, &adc_seq);
        rc = adc_read(adc_vdd.dev, &adc_seq);
        if (rc != 0) {
            LOG_WRN("ADC read failed: %d", rc);
        } else if (adc_raw < 0) {
            LOG_WRN("ADC returned negative sample: %d", adc_raw);
        } else {
            /* Derive the scaling from the DT channel rather than hardcoding it:
             * the nRF54L internal reference is 900 mV, not the 600 mV of the
             * nRF52 series, so gain 1/6 gives a 5.4 V full scale. */
            int32_t mv = adc_raw;
            rc = adc_raw_to_millivolts_dt(&adc_vdd, &mv);
            if (rc == 0) {
                last_voltage_mv = (uint32_t)mv;
            } else {
                LOG_WRN("ADC conversion failed: %d", rc);
            }
        }
    }
    /* Suspend explicitly to stop SAADC from leaking ~1mA */
    (void)pm_device_action_run(adc_vdd.dev, PM_DEVICE_ACTION_SUSPEND);

    /* 4. Collect the light reading.  Sleep out whatever is left of the
     * conversion — the CPU idles through it, and the sensor puts itself back
     * into shutdown once the conversion completes. */
    if (opt_started) {
        int64_t elapsed = k_uptime_get() - opt_kicked_at;

        if (elapsed < OPT3005_CONV_WAIT_MS) {
            k_sleep(K_MSEC(OPT3005_CONV_WAIT_MS - elapsed));
        }

        struct sensor_value lux_val = {0};

        rc = sensor_sample_fetch(opt);
        if (rc) {
            LOG_WRN("OPT fetch failed: %d", rc);
        } else {
            sensor_channel_get(opt, SENSOR_CHAN_LIGHT, &lux_val);
            last_lux = (uint32_t)lux_val.val1;
        }
    }

    /* 5. Store temperature in the circular history buffer. */
    temp_hist[hist_head] = last_temp_mdeg;
    hist_head = (hist_head + 1) % HISTORY_SIZE;
    if (hist_count < HISTORY_SIZE) {
        hist_count++;
    }

    (void)storage_update_minmax(last_temp_mdeg);

    /* Persist the history on a slow cadence — hourly at the default interval.
     * Every cycle would be 144 writes/day for data whose value is mostly in the
     * last hour of it. */
    static uint32_t measure_count;
    if ((measure_count % STORAGE_HISTORY_SAVE_EVERY) == 0) {
        (void)storage_save_history(temp_hist, hist_count, hist_head);
    }
    measure_count++;

    LOG_INF("T=%d.%03d°C  RH=%d.%03d%%  L=%u lux  VDD=%u mV",
            (int)(last_temp_mdeg / 1000), (int)(abs(last_temp_mdeg) % 1000),
            (int)(last_humid_mpct / 1000), (int)(last_humid_mpct % 1000),
            last_lux, last_voltage_mv);

    /* 6. Publish over BLE: refreshes the BTHome payload and the GATT values,
     * then beacons a short burst. */
    ble_publish(last_temp_mdeg, last_humid_mpct, last_lux, last_voltage_mv);

    /* 7. Night mode: skip the display entirely if it is too dark to read.
     * Hysteresis is applied against whether the display is currently blanked, so
     * the panel does not flap when the room sits at the threshold. */
    static bool display_blanked;

    if (display_blanked) {
        if (last_lux < DARK_THRESH_UPPER_LUX) {
            LOG_DBG("still dark (%u lux) — display update skipped", last_lux);
            return;
        }
        display_blanked = false;
    } else if (last_lux < DARK_THRESH_LOWER_LUX) {
        LOG_INF("dark (%u lux) — display update skipped", last_lux);
        display_blanked = true;
        return;
    }

    /* 8. Power on display */
    gpio_pin_set_dt(&load_sw, 1);
    k_sleep(K_MSEC(10));

    /* 9. Update display (init on first call).
     * Both steps are retried on the next cycle rather than latched off, so a
     * panel that misses one update recovers by itself. */
    static bool display_initialized = false;
    if (!display_initialized) {
        rc = display_init();
        if (rc) {
            LOG_ERR("display init failed: %d — retrying next cycle", rc);
        } else {
            display_initialized = true;
        }
    }

    if (display_initialized) {
        rc = display_update(last_temp_mdeg, last_humid_mpct,
                            last_voltage_mv,
                            temp_hist, hist_count, hist_head);
        if (rc) {
            LOG_ERR("display update failed: %d — reinitialising next cycle", rc);
            display_initialized = false;
        }
    }

    /* 10. Float every panel-facing pin BEFORE opening the load switch.
     * This is panel protection, not just a power optimisation: a pin left
     * driven at 3 V into an unpowered panel pushes current through the
     * driver's input ESD clamps into the dead VDD rail.  R2-R5 (22 R) do not
     * prevent that, and RST_N has no series resistor at all.  Keep the order. */
    display_power_off();
    gpio_pin_set_dt(&load_sw, 0);
}

/* ── BLE button ───────────────────────────────────────────────────────── */
static void do_ble_button(struct k_work *w)
{
    (void)w;
    ble_start_interactive();
}

/* ── Timer callbacks (run in ISR — only submit work) ─────────────────── */
static void measure_timer_cb(struct k_timer *t)
{
    (void)t;
    /* schedule, not reschedule: never push out a debounce already counting
     * down for a button press — one run serves both. */
    k_work_schedule_for_queue(&measure_wq, &measure_work, K_NO_WAIT);
}

/* ── GPIO interrupt callbacks ─────────────────────────────────────────── */
static struct gpio_callback sr_cb_data;
static struct gpio_callback ble_cb_data;

static void sr_button_isr(const struct device *dev,
                          struct gpio_callback *cb, uint32_t pins)
{
    (void)dev; (void)cb; (void)pins;
    k_work_reschedule_for_queue(&measure_wq, &measure_work,
                                K_MSEC(BUTTON_DEBOUNCE_MS));
}

static void ble_button_isr(const struct device *dev,
                           struct gpio_callback *cb, uint32_t pins)
{
    (void)dev; (void)cb; (void)pins;
    k_work_reschedule(&ble_button_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}

/* ── Main ─────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Load switch first, before anything else can take time.  IC1's ON pin has
     * no external pull-down, so from SoC reset until this line runs it floats
     * and the display rail is in an undefined state — which matters most after
     * a watchdog reset taken mid-refresh.  This shortens that window to the
     * Zephyr init sequence; only a ~100 k pull-down on the LoadSwitch net
     * closes it completely. */
    gpio_pin_configure_dt(&load_sw, GPIO_OUTPUT_INACTIVE);

    LOG_INF("naulaTAG starting");

    /* Undo the opt300x driver's continuous-conversion init.  Every reading from
     * here on is an explicit single shot. */
    if (device_is_ready(i2c_bus)) {
        if (opt3005_write_config(OPT3005_CFG_SHUTDOWN) != 0) {
            LOG_WRN("OPT3005 shutdown write failed — sensor may stay continuous");
        }
    } else {
        LOG_ERR("I2C bus not ready");
    }

    if (device_is_ready(adc_vdd.dev)) {
        (void)pm_device_action_run(adc_vdd.dev, PM_DEVICE_ACTION_SUSPEND);
    }

    /* Persistent history and all-time extremes. */
    if (storage_init() == 0) {
        uint16_t count, head;

        if (storage_load_history(temp_hist, &count, &head) == 0) {
            hist_count = count;
            hist_head = head;
        }
    } else {
        LOG_WRN("storage unavailable — history will not persist");
    }

    /* Buttons: input with interrupt on falling edge (active-low) */
    gpio_pin_configure_dt(&sr_btn,  GPIO_INPUT);
    gpio_pin_configure_dt(&ble_btn, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&sr_btn,  GPIO_INT_EDGE_FALLING);
    gpio_pin_interrupt_configure_dt(&ble_btn, GPIO_INT_EDGE_FALLING);

    gpio_init_callback(&sr_cb_data,  sr_button_isr,  BIT(sr_btn.pin));
    gpio_init_callback(&ble_cb_data, ble_button_isr, BIT(ble_btn.pin));

    gpio_add_callback(sr_btn.port,  &sr_cb_data);
    gpio_add_callback(ble_btn.port, &ble_cb_data);

    /* Work items.  The measurement queue must exist before any timer or
     * interrupt can schedule onto it. */
    k_work_queue_start(&measure_wq, measure_wq_stack,
                       K_THREAD_STACK_SIZEOF(measure_wq_stack),
                       CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);

    k_work_init_delayable(&measure_work,    do_measure);
    k_work_init_delayable(&ble_button_work, do_ble_button);

    /* Periodic measurement timer */
    k_timer_init(&measure_timer, measure_timer_cb, NULL);
    k_timer_start(&measure_timer, K_SECONDS(1), K_SECONDS(MEASURE_INTERVAL_S));

    /* Watchdog: install before anything can block, but after the timers exist
     * so the first feed is only a second away.  Pauses under a debugger. */
    if (!device_is_ready(wdt)) {
        LOG_ERR("watchdog not ready — running unprotected");
    } else {
        struct wdt_timeout_cfg wdt_cfg = {
            .window   = { .min = 0U, .max = WDT_TIMEOUT_S * 1000U },
            .callback = NULL,
            .flags    = WDT_FLAG_RESET_SOC,
        };

        wdt_channel = wdt_install_timeout(wdt, &wdt_cfg);
        if (wdt_channel < 0) {
            LOG_ERR("wdt_install_timeout failed: %d", wdt_channel);
        } else if (wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) != 0) {
            LOG_ERR("wdt_setup failed — running unprotected");
            wdt_channel = -1;
        } else {
            LOG_INF("watchdog armed, %d s", WDT_TIMEOUT_S);
        }
    }

    (void)ble_init();

    /* A failure here leaves the battery reading at 0 mV for the whole session,
     * so it must be visible rather than silent. */
    if (!adc_is_ready_dt(&adc_vdd)) {
        LOG_ERR("ADC device not ready — battery voltage unavailable");
    } else {
        int rc = adc_channel_setup_dt(&adc_vdd);

        if (rc) {
            LOG_ERR("ADC channel setup failed: %d", rc);
        }
    }

    /* Sleep forever — woken by GPIO interrupts or timers. */
    k_sleep(K_FOREVER);
    return 0;
}
