/*
 * ZMS-backed persistence.  See storage.h for the rationale.
 */

#include "storage.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/zms.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

#define STORAGE_PARTITION  storage_partition

/* Record IDs.  Never renumber these — a changed ID orphans the stored data. */
#define ID_HISTORY  1
#define ID_MINMAX   2

/* Bumped only if a record layout changes, so a stale record is rejected rather
 * than misinterpreted after a firmware update. */
#define HISTORY_VERSION  1

/*
 * The in-RAM history is int32 milli-degrees because that is what display.c
 * plots.  On the way to storage it is narrowed to int16 centi-degrees: the
 * display only ever shows one decimal, so this loses nothing visible and halves
 * the bytes written (288 instead of 576).
 */
struct history_record {
	uint8_t  version;
	uint8_t  reserved;
	uint16_t count;
	uint16_t head;
	uint16_t entries;                       /* guards against a resized buffer */
	int16_t  samples[DISPLAY_HIST_SIZE];    /* centi-degrees C */
};

struct minmax_record {
	uint8_t version;
	uint8_t reserved[3];
	int32_t min_mdeg;
	int32_t max_mdeg;
};

static struct zms_fs fs;
static bool mounted;

/* Cached so a read is free and so update_minmax() can decide whether a write is
 * actually needed.  valid == false means "nothing recorded yet". */
static struct minmax_record minmax_cache;
static bool minmax_valid;

int storage_init(void)
{
	struct flash_pages_info page;
	int rc;

	if (mounted) {
		return 0;
	}

	fs.flash_device = FIXED_PARTITION_DEVICE(STORAGE_PARTITION);
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("storage flash device not ready");
		return -ENODEV;
	}

	fs.offset = FIXED_PARTITION_OFFSET(STORAGE_PARTITION);

	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &page);
	if (rc) {
		LOG_ERR("flash_get_page_info_by_offs failed: %d", rc);
		return rc;
	}

	fs.sector_size = page.size;
	fs.sector_count = FIXED_PARTITION_SIZE(STORAGE_PARTITION) / page.size;

	if (fs.sector_count < 2) {
		/* ZMS needs at least two sectors to garbage-collect into. */
		LOG_ERR("storage partition too small: %u sector(s)", fs.sector_count);
		return -EINVAL;
	}

	rc = zms_mount(&fs);
	if (rc) {
		LOG_ERR("zms_mount failed: %d", rc);
		return rc;
	}

	mounted = true;
	LOG_INF("storage mounted: %u sectors x %u B", fs.sector_count, fs.sector_size);

	/* Prime the min/max cache so later updates only write on a real change. */
	ssize_t n = zms_read(&fs, ID_MINMAX, &minmax_cache, sizeof(minmax_cache));

	if (n == sizeof(minmax_cache) && minmax_cache.version == HISTORY_VERSION) {
		minmax_valid = true;
		LOG_INF("all-time min %d.%03d C / max %d.%03d C",
			minmax_cache.min_mdeg / 1000, abs(minmax_cache.min_mdeg) % 1000,
			minmax_cache.max_mdeg / 1000, abs(minmax_cache.max_mdeg) % 1000);
	} else if (n >= 0) {
		/* Present but the wrong size or version — treat as absent so the next
		 * reading rewrites it cleanly. */
		LOG_WRN("min/max record unusable (%d B), starting fresh", (int)n);
	}

	return 0;
}

int storage_load_history(int32_t *hist, uint16_t *count, uint16_t *head)
{
	struct history_record rec;
	ssize_t n;

	if (!mounted) {
		return -ENODEV;
	}

	n = zms_read(&fs, ID_HISTORY, &rec, sizeof(rec));
	if (n == -ENOENT) {
		return -ENOENT;
	}
	if (n < 0) {
		LOG_ERR("history read failed: %d", (int)n);
		return (int)n;
	}

	if (n != sizeof(rec) || rec.version != HISTORY_VERSION ||
	    rec.entries != DISPLAY_HIST_SIZE ||
	    rec.count > DISPLAY_HIST_SIZE || rec.head >= DISPLAY_HIST_SIZE) {
		LOG_WRN("history record unusable, discarding");
		return -ENOENT;
	}

	for (uint16_t i = 0; i < DISPLAY_HIST_SIZE; i++) {
		hist[i] = (int32_t)rec.samples[i] * 10;
	}
	*count = rec.count;
	*head = rec.head;

	LOG_INF("history restored: %u sample(s)", rec.count);
	return 0;
}

int storage_save_history(const int32_t *hist, uint16_t count, uint16_t head)
{
	struct history_record rec;
	ssize_t n;

	if (!mounted) {
		return -ENODEV;
	}

	rec.version = HISTORY_VERSION;
	rec.reserved = 0;
	rec.count = count;
	rec.head = head;
	rec.entries = DISPLAY_HIST_SIZE;

	for (uint16_t i = 0; i < DISPLAY_HIST_SIZE; i++) {
		int32_t centi = hist[i] / 10;

		/* Clamp rather than wrap: a bogus sample should not corrupt the graph. */
		if (centi > INT16_MAX) {
			centi = INT16_MAX;
		} else if (centi < INT16_MIN) {
			centi = INT16_MIN;
		}
		rec.samples[i] = (int16_t)centi;
	}

	n = zms_write(&fs, ID_HISTORY, &rec, sizeof(rec));
	if (n < 0) {
		LOG_ERR("history write failed: %d", (int)n);
		return (int)n;
	}

	LOG_DBG("history saved (%u samples)", count);
	return 0;
}

bool storage_update_minmax(int32_t temp_mdeg)
{
	bool changed = false;
	ssize_t n;

	if (!mounted) {
		return false;
	}

	if (!minmax_valid) {
		minmax_cache.version = HISTORY_VERSION;
		memset(minmax_cache.reserved, 0, sizeof(minmax_cache.reserved));
		minmax_cache.min_mdeg = temp_mdeg;
		minmax_cache.max_mdeg = temp_mdeg;
		minmax_valid = true;
		changed = true;
	} else {
		if (temp_mdeg < minmax_cache.min_mdeg) {
			minmax_cache.min_mdeg = temp_mdeg;
			changed = true;
		}
		if (temp_mdeg > minmax_cache.max_mdeg) {
			minmax_cache.max_mdeg = temp_mdeg;
			changed = true;
		}
	}

	if (!changed) {
		return false;
	}

	n = zms_write(&fs, ID_MINMAX, &minmax_cache, sizeof(minmax_cache));
	if (n < 0) {
		LOG_ERR("min/max write failed: %d", (int)n);
		return false;
	}

	LOG_INF("new extreme: min %d.%03d C / max %d.%03d C",
		minmax_cache.min_mdeg / 1000, abs(minmax_cache.min_mdeg) % 1000,
		minmax_cache.max_mdeg / 1000, abs(minmax_cache.max_mdeg) % 1000);
	return true;
}

int storage_get_minmax(int32_t *min_mdeg, int32_t *max_mdeg)
{
	if (!minmax_valid) {
		return -ENOENT;
	}

	*min_mdeg = minmax_cache.min_mdeg;
	*max_mdeg = minmax_cache.max_mdeg;
	return 0;
}
