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
 * Render temperature and relative humidity and push to the display.
 *
 *  temp_mdeg  : temperature in milli-degrees Celsius (e.g. 23500 = 23.5 °C)
 *  humid_mpct : relative humidity in milli-percent  (e.g. 45000 = 45.0 %)
 *
 * The display driver will automatically set the temperature register so the
 * panel waveform matches the ambient temperature for best contrast.
 */
void display_update(int32_t temp_mdeg, uint32_t humid_mpct);

#ifdef __cplusplus
}
#endif
