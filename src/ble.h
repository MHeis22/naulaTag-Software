/*
 * Bluetooth for naulaTAG.
 *
 * Two roles, because they serve different consumers:
 *
 *   BEACON (default) — a short non-connectable burst of BTHome v2 service data
 *     after every measurement.  Home Assistant's BTHome integration picks this
 *     up with no custom code, so the device logs itself to a hub without anyone
 *     touching it.  Bursting rather than beaconing continuously matters: the
 *     data only changes every 10 minutes, so advertising continuously would
 *     spend current re-sending values nobody has not already seen.  A few
 *     packets per measurement costs well under 0.1 uA averaged.
 *
 *   INTERACTIVE (on button press) — 30 s of fast connectable advertising
 *     carrying the device name, with the Environmental Sensing and Battery
 *     services readable over the connection.  This is the "point a phone at it"
 *     path, and it is the only mode that is connectable.
 *
 * The two are mutually exclusive; a button press during a beacon burst takes
 * over, and the burst is not restored (the next measurement starts another).
 */

#ifndef NAULATAG_BLE_H
#define NAULATAG_BLE_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Enable the controller and register the GATT services. */
int ble_init(void);

/**
 * @brief Publish a new measurement.
 *
 * Refreshes the BTHome payload and the GATT characteristic values, then starts
 * a beacon burst.  If an interactive session is in progress the advertisement
 * data is updated in place instead, so a connected phone sees live values.
 *
 * @param temp_mdeg   Temperature in milli-degrees C.
 * @param humid_mpct  Relative humidity in milli-percent.
 * @param lux         Illuminance in lux.
 * @param vdd_mv      Supply voltage in millivolts.
 */
void ble_publish(int32_t temp_mdeg, uint32_t humid_mpct, uint32_t lux, uint32_t vdd_mv);

/** @brief Start (or restart) a 30 s interactive advertising window. */
void ble_start_interactive(void);

#endif /* NAULATAG_BLE_H */
