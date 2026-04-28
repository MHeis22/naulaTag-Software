/*
 * naulaTAG — IoT temperature/humidity/light meter
 *
 * Hardware: nRF54L15-QFAA-R7
 *   SRButton  (P0.01) — immediate measurement
 *   BLEButton (P1.07) — toggle BLE advertising
 *   HDC3022 (I2C 0x45) — temperature + humidity
 *   OPT3005 (I2C 0x44) — ambient light, INT on P1.06
 *   E2206KS0E1 (SPI)   — 2.06" e-ink display
 *   LoadSwitch (P1.04) — display power rail
 *
 * Power strategy:
 *   CPU sleeps indefinitely (k_sleep(K_FOREVER)).  Zephyr PM enters the
 *   deepest available idle state.  Wakeup sources: GPIO interrupt (buttons,
 *   OPT3005 light-threshold crossing), GRTC timer (periodic measurement).
 *   Display is powered only during screen updates and skipped entirely when
 *   ambient light is below DARK_THRESHOLD_LUX (e.g. inside a box overnight).
 *   BLE is off by default; payload carries live sensor readings.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "display.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ── Measurement interval ─────────────────────────────────────────────── */
#define MEASURE_INTERVAL_S  600

/* BLE advertising window after button press (seconds) */
#define BLE_ADV_TIMEOUT_S   30

/* Ambient light below which display updates are skipped (lux).
 * 5 lux ≈ deep twilight; anything darker means no one can read the display. */
#define DARK_THRESHOLD_LUX  5U

/* ── Device handles ───────────────────────────────────────────────────── */
static const struct gpio_dt_spec sr_btn =
    GPIO_DT_SPEC_GET(DT_ALIAS(sr_button), gpios);
static const struct gpio_dt_spec ble_btn =
    GPIO_DT_SPEC_GET(DT_ALIAS(ble_button), gpios);
static const struct gpio_dt_spec load_sw =
    GPIO_DT_SPEC_GET(DT_ALIAS(epd_load_sw), gpios);

static const struct device *hdc = DEVICE_DT_GET(DT_NODELABEL(hdc3022));
static const struct device *opt = DEVICE_DT_GET(DT_NODELABEL(opt3005));

/* ── Work items ────────────────────────────────────────────────────────── */
static struct k_work measure_work;
static struct k_work ble_toggle_work;

/* ── Timers ────────────────────────────────────────────────────────────── */
static struct k_timer measure_timer;
static struct k_timer ble_off_timer;

/* ── State ─────────────────────────────────────────────────────────────── */
static bool ble_adv_active = false;

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

/* ── BLE manufacturer data ─────────────────────────────────────────────── */
/*
 * 8-byte payload, all little-endian:
 *   [0-1]  Company ID  — 0xFFFF (test/undefined; replace with registered ID)
 *   [2-3]  Temperature — int16_t in units of 0.1 °C
 *   [4-5]  Humidity    — uint16_t in units of 0.1 %RH
 *   [6-7]  Light       — uint16_t in lux (capped at 65535)
 *
 * A BLE central reading 0xFFFF / company 0xFFFF / T=235 / H=450 / L=120
 * decodes as: 23.5 °C, 45.0 %RH, 120 lux.
 */
#define MFR_DATA_LEN  8

static uint8_t mfr_data[MFR_DATA_LEN];

static void refresh_mfr_data(void)
{
    int16_t  t = (int16_t)(last_temp_mdeg  / 100);
    uint16_t h = (uint16_t)(last_humid_mpct / 100);
    uint16_t l = (uint16_t)(last_lux > 0xFFFFU ? 0xFFFFU : last_lux);

    mfr_data[0] = 0xFF;
    mfr_data[1] = 0xFF;
    mfr_data[2] = (uint8_t)((uint16_t)t);
    mfr_data[3] = (uint8_t)((uint16_t)t >> 8);
    mfr_data[4] = (uint8_t)(h);
    mfr_data[5] = (uint8_t)(h >> 8);
    mfr_data[6] = (uint8_t)(l);
    mfr_data[7] = (uint8_t)(l >> 8);
}

/* ── BLE advertisement data ────────────────────────────────────────────── */
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfr_data, MFR_DATA_LEN),
};

/* ── OPT3005 event-driven threshold interrupt ──────────────────────────── */
/*
 * After each measurement we re-arm the OPT3005 for the opposite edge:
 *   dark  → arm high-limit at DARK_THRESHOLD_LUX  (wake when lights turn on)
 *   bright → arm low-limit  at DARK_THRESHOLD_LUX  (wake when lights turn off)
 *
 * This gives instant wakeups on both transitions with no continuous firing.
 */
static void opt_trigger_handler(const struct device *dev,
                                 const struct sensor_trigger *trig)
{
    (void)dev; (void)trig;
    k_work_submit(&measure_work);
}

static void arm_opt3005_trigger(uint32_t lux)
{
    struct sensor_value thresh;
    const struct sensor_trigger trig = {
        .type = SENSOR_TRIG_THRESHOLD,
        .chan = SENSOR_CHAN_LIGHT,
    };

    if (lux < DARK_THRESHOLD_LUX) {
        /* Dark: fire when lux exceeds threshold (lights turned on) */
        thresh.val1 = (int32_t)DARK_THRESHOLD_LUX;
        thresh.val2 = 0;
        sensor_attr_set(opt, SENSOR_CHAN_LIGHT, SENSOR_ATTR_UPPER_THRESH, &thresh);
        thresh.val1 = 0;
        thresh.val2 = 0;
        sensor_attr_set(opt, SENSOR_CHAN_LIGHT, SENSOR_ATTR_LOWER_THRESH, &thresh);
    } else {
        /* Bright: fire when lux drops below threshold (lights turned off) */
        thresh.val1 = (int32_t)DARK_THRESHOLD_LUX;
        thresh.val2 = 0;
        sensor_attr_set(opt, SENSOR_CHAN_LIGHT, SENSOR_ATTR_LOWER_THRESH, &thresh);
        thresh.val1 = 100000;  /* beyond any real indoor lux */
        thresh.val2 = 0;
        sensor_attr_set(opt, SENSOR_CHAN_LIGHT, SENSOR_ATTR_UPPER_THRESH, &thresh);
    }

    sensor_trigger_set(opt, &trig, opt_trigger_handler);
}

/* ── Measurement ──────────────────────────────────────────────────────── */
static void do_measure(struct k_work *w)
{
    (void)w;

    /* 1. Read temperature and humidity */
    struct sensor_value temp, humid;
    int rc = sensor_sample_fetch(hdc);
    if (rc) {
        LOG_ERR("HDC fetch failed: %d", rc);
        return;
    }
    sensor_channel_get(hdc, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(hdc, SENSOR_CHAN_HUMIDITY,     &humid);

    last_temp_mdeg  = temp.val1  * 1000 + temp.val2  / 1000;
    last_humid_mpct = humid.val1 * 1000 + humid.val2 / 1000;

    /* 2. Read ambient light */
    struct sensor_value lux_val = {0};
    rc = sensor_sample_fetch(opt);
    if (rc) {
        LOG_WRN("OPT fetch failed: %d", rc);
    } else {
        sensor_channel_get(opt, SENSOR_CHAN_LIGHT, &lux_val);
        last_lux = (uint32_t)lux_val.val1;
    }

    /* 3. Read battery voltage via internal SAADC VDD channel.
     *    Gain=1/6, ref=600 mV → full-scale 3600 mV, 12-bit resolution. */
    {
        int16_t adc_raw = 0;
        struct adc_sequence adc_seq = {
            .buffer      = &adc_raw,
            .buffer_size = sizeof(adc_raw),
        };
        adc_sequence_init_dt(&adc_vdd, &adc_seq);
        rc = adc_read(adc_vdd.dev, &adc_seq);
        if (rc == 0 && adc_raw >= 0) {
            last_voltage_mv = (uint32_t)((int32_t)adc_raw * 3600 / 4096);
        } else {
            LOG_WRN("ADC read failed: %d", rc);
        }
    }

    /* 4. Store temperature in the circular history buffer */
    temp_hist[hist_head] = last_temp_mdeg;
    hist_head = (hist_head + 1) % HISTORY_SIZE;
    if (hist_count < HISTORY_SIZE) {
        hist_count++;
    }

    LOG_INF("T=%d.%03d°C  RH=%d.%03d%%  L=%u lux  VDD=%u mV",
            temp.val1,  temp.val2  / 1000,
            humid.val1, humid.val2 / 1000,
            last_lux, last_voltage_mv);

    /* 5. Update BLE payload; push to stack if advertising is active */
    refresh_mfr_data();
    if (ble_adv_active) {
        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
    }

    /* 6. Re-arm OPT3005 threshold interrupt for the current light state */
    arm_opt3005_trigger(last_lux);

    /* 7. Night mode: skip display entirely if too dark to see */
    if (last_lux < DARK_THRESHOLD_LUX) {
        LOG_INF("Dark — display update skipped");
        return;
    }

    /* 8. Power on display */
    gpio_pin_set_dt(&load_sw, 1);
    k_sleep(K_MSEC(10));

    /* 9. Update display (init on first call) */
    static bool display_initialized = false;
    if (!display_initialized) {
        rc = display_init();
        if (rc == 0) display_initialized = true;
        else         LOG_ERR("display init failed: %d", rc);
    }

    if (display_initialized) {
        display_update(last_temp_mdeg, last_humid_mpct,
                       last_voltage_mv,
                       temp_hist, hist_count, hist_head);
    }

    /* 10. Power off display */
    gpio_pin_set_dt(&load_sw, 0);
}

/* ── BLE toggle ───────────────────────────────────────────────────────── */
static void do_ble_toggle(struct k_work *w)
{
    (void)w;

    if (ble_adv_active) {
        bt_le_adv_stop();
        k_timer_stop(&ble_off_timer);
        ble_adv_active = false;
        LOG_INF("BLE advertising stopped");
    } else {
        /* Stamp current readings into the payload before advertising starts */
        refresh_mfr_data();
        int rc = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                                 ad, ARRAY_SIZE(ad), NULL, 0);
        if (rc == 0) {
            ble_adv_active = true;
            k_timer_start(&ble_off_timer,
                          K_SECONDS(BLE_ADV_TIMEOUT_S), K_NO_WAIT);
            LOG_INF("BLE adv started (%d s) — T=%d.%01d°C RH=%d.%01d%% L=%u lux",
                    BLE_ADV_TIMEOUT_S,
                    (int)(last_temp_mdeg  / 1000),
                    (int)((last_temp_mdeg  % 1000) / 100),
                    (int)(last_humid_mpct / 1000),
                    (int)((last_humid_mpct % 1000) / 100),
                    last_lux);
        } else {
            LOG_ERR("BLE adv start failed: %d", rc);
        }
    }
}

/* ── Timer callbacks (run in ISR — only submit work) ─────────────────── */
static void measure_timer_cb(struct k_timer *t)
{
    (void)t;
    k_work_submit(&measure_work);
}

static void ble_off_timer_cb(struct k_timer *t)
{
    (void)t;
    k_work_submit(&ble_toggle_work);
}

/* ── GPIO interrupt callbacks ─────────────────────────────────────────── */
static struct gpio_callback sr_cb_data;
static struct gpio_callback ble_cb_data;

static void sr_button_isr(const struct device *dev,
                          struct gpio_callback *cb, uint32_t pins)
{
    (void)dev; (void)cb; (void)pins;
    k_work_submit(&measure_work);
}

static void ble_button_isr(const struct device *dev,
                           struct gpio_callback *cb, uint32_t pins)
{
    (void)dev; (void)cb; (void)pins;
    k_work_submit(&ble_toggle_work);
}

/* ── BLE connection callbacks (informational only) ────────────────────── */
static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) LOG_ERR("BLE connect error %u", err);
    else     LOG_INF("BLE connected");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected (reason %u)", reason);
    if (ble_adv_active) {
        bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
    }
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

/* ── Main ─────────────────────────────────────────────────────────────── */
int main(void)
{
    LOG_INF("naulaTAG starting");

    /* Load switch: output, initially off */
    gpio_pin_configure_dt(&load_sw, GPIO_OUTPUT_INACTIVE);

    /* Buttons: input with interrupt on falling edge (active-low) */
    gpio_pin_configure_dt(&sr_btn,  GPIO_INPUT);
    gpio_pin_configure_dt(&ble_btn, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&sr_btn,  GPIO_INT_EDGE_FALLING);
    gpio_pin_interrupt_configure_dt(&ble_btn, GPIO_INT_EDGE_FALLING);

    gpio_init_callback(&sr_cb_data,  sr_button_isr,  BIT(sr_btn.pin));
    gpio_init_callback(&ble_cb_data, ble_button_isr, BIT(ble_btn.pin));

    gpio_add_callback(sr_btn.port,  &sr_cb_data);
    gpio_add_callback(ble_btn.port, &ble_cb_data);

    /* Work items */
    k_work_init(&measure_work,    do_measure);
    k_work_init(&ble_toggle_work, do_ble_toggle);

    /* Periodic measurement timer */
    k_timer_init(&measure_timer, measure_timer_cb, NULL);
    k_timer_start(&measure_timer,
                  K_SECONDS(1),                   /* first fire after 1s  */
                  K_SECONDS(MEASURE_INTERVAL_S));  /* then every interval  */

    /* BLE auto-off timer (one-shot, restarted each time BLE is enabled) */
    k_timer_init(&ble_off_timer, ble_off_timer_cb, NULL);

    /* Initialise Bluetooth stack (radio stays off until advertising starts) */
    int rc = bt_enable(NULL);
    if (rc) LOG_ERR("bt_enable failed: %d", rc);

    /* Initialise ADC channel for battery voltage measurement */
    if (!adc_is_ready_dt(&adc_vdd)) {
        LOG_WRN("ADC not ready — voltage readings disabled");
    } else {
        rc = adc_channel_setup_dt(&adc_vdd);
        if (rc) LOG_ERR("ADC channel setup failed: %d", rc);
    }

    /* Arm OPT3005 light-threshold interrupt (assume dark at boot) */
    if (device_is_ready(opt)) {
        arm_opt3005_trigger(0);
    } else {
        LOG_WRN("OPT3005 not ready — light interrupt disabled");
    }

    /* Sleep forever — woken by GPIO interrupts or timers.
     * Zephyr PM will select the deepest idle state the hardware supports. */
    k_sleep(K_FOREVER);
    return 0;
}
