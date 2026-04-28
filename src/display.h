/*
 * C interface to the e-ink display (E2206KS0E1, 2.06").
 * Call display_init() once, then display_update() each time the screen
 * needs refreshing.  The load switch must be asserted externally before
 * calling and de-asserted after it returns.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialise the display driver and read OTP from the panel.
 * Must be called while the load switch is ON (display powered).
 * Returns 0 on success, negative errno on failure.
 */
int display_init(void);

/*
 * Render all sensor data and push to the display.
 *
 *  temp_mdeg  : temperature in milli-degrees Celsius (e.g. 23500 = 23.5 °C)
 *  humid_mpct : relative humidity in milli-percent  (e.g. 45000 = 45.0 %)
 *  voltage_mv : supply voltage in millivolts         (e.g. 3024  = 3.024 V)
 *  temp_hist  : circular buffer of past temp_mdeg readings (may be NULL if
 *               hist_count == 0)
 *  hist_count : number of valid entries in temp_hist (0 .. DISPLAY_HIST_SIZE)
 *  hist_head  : next-write index in temp_hist; oldest entry is at
 *               (hist_head - hist_count + DISPLAY_HIST_SIZE) % DISPLAY_HIST_SIZE
 *
 * The display driver automatically sets the temperature register so the panel
 * waveform matches the ambient temperature for best contrast.
 */
#define DISPLAY_HIST_SIZE  144

void display_update(int32_t temp_mdeg, uint32_t humid_mpct,
                    uint32_t voltage_mv,
                    const int32_t *temp_hist, uint16_t hist_count,
                    uint16_t hist_head);

#ifdef __cplusplus
}
#endif
