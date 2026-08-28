/*
 * Persistent storage for naulaTAG.
 *
 * Backed by ZMS (Zephyr Memory Storage) on the nRF54L15's RRAM.  ZMS is used
 * rather than NVS because it is designed for non-erasable media like RRAM and
 * MRAM; NVS would emulate flash erases by writing 0xFF patterns and burn write
 * cycles for nothing.
 *
 * Two records live here:
 *
 *   - The 24 h temperature history, so the graph survives a watchdog reset or a
 *     battery change instead of restarting empty.  Saved on a slow cadence (see
 *     STORAGE_HISTORY_SAVE_EVERY) because it is bulky and only worth what the
 *     last hour of it is.
 *
 *   - All-time min/max temperature.  Written only when a new extreme actually
 *     occurs, which is naturally rare, so this costs almost no write cycles.
 */

#ifndef NAULATAG_STORAGE_H
#define NAULATAG_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "display.h"

/* Persist the history every Nth measurement.  At MEASURE_INTERVAL_S = 600 this
 * is hourly: 24 writes/day, which with ZMS wear levelling across the 9 sectors
 * of the 36 KB storage partition is far inside the RRAM endurance budget. */
#define STORAGE_HISTORY_SAVE_EVERY  6

/** @brief Mount the ZMS filesystem.  Safe to call once, at boot. */
int storage_init(void);

/**
 * @brief Restore the temperature history written by storage_save_history().
 *
 * @param hist   Destination buffer of DISPLAY_HIST_SIZE entries, milli-degrees C.
 * @param count  Receives the number of valid entries.
 * @param head   Receives the next write position.
 *
 * @retval 0        History restored.
 * @retval -ENOENT  Nothing stored yet; outputs are left untouched.
 * @retval <0       Storage error.
 *
 * @note There is no wall clock across a power cycle, so restored samples carry
 *       no age.  After a watchdog reset that is irrelevant; after a battery
 *       change the graph is stale until it scrolls out over the next 24 h.
 */
int storage_load_history(int32_t *hist, uint16_t *count, uint16_t *head);

/** @brief Persist the temperature history.  Values are milli-degrees C. */
int storage_save_history(const int32_t *hist, uint16_t count, uint16_t head);

/**
 * @brief Fold a new reading into the all-time min/max, persisting on change.
 *
 * @return true if this reading set a new extreme (and was written to storage).
 */
bool storage_update_minmax(int32_t temp_mdeg);

/**
 * @brief Read back the all-time extremes.
 *
 * @retval 0        Both outputs valid.
 * @retval -ENOENT  No extremes recorded yet.
 */
int storage_get_minmax(int32_t *min_mdeg, int32_t *max_mdeg);

#endif /* NAULATAG_STORAGE_H */
