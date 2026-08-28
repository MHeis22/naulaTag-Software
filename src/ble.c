/*
 * BTHome v2 beacon + Environmental Sensing / Battery GATT.  See ble.h.
 */

#include "ble.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <string.h>

LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

/* ── Timing ─────────────────────────────────────────────────────────────── */

/* Long enough that a hub almost certainly catches one packet, short enough that
 * the radio is off for >99% of the measurement interval. */
#define BEACON_BURST_MS       3000
#define INTERACTIVE_MS       30000

/* ── BTHome v2 ──────────────────────────────────────────────────────────── */
/*
 * Service data under UUID 0xFCD2.  Layout is a device-information byte followed
 * by (object id, little-endian value) pairs which MUST be in ascending object-id
 * order — receivers rely on that to skip object ids they do not know.
 *
 * Object ids, types and factors per the BTHome v2 spec:
 *   0x00 packet id    uint8    x1
 *   0x01 battery      uint8    x1      %
 *   0x02 temperature  sint16   x0.01   deg C
 *   0x03 humidity     uint16   x0.01   %
 *   0x05 illuminance  uint24   x0.01   lux
 *   0x0C voltage      uint16   x0.001  V
 */
#define BTHOME_UUID16          0xFCD2
/* bit0 encryption=0, bit2 trigger=0, bits5-7 version=2 */
#define BTHOME_DEVICE_INFO     0x40

#define BTHOME_ID_PACKET       0x00
#define BTHOME_ID_BATTERY      0x01
#define BTHOME_ID_TEMPERATURE  0x02
#define BTHOME_ID_HUMIDITY     0x03
#define BTHOME_ID_ILLUMINANCE  0x05
#define BTHOME_ID_VOLTAGE      0x0C

/* 1 + 2 + 2 + 3 + 3 + 4 + 3.  With the 2-byte UUID and the AD header this is 22
 * of the 31 legacy advertising bytes, which is why the beacon carries no device
 * name — it would need 10 more and BTHome does not need it (hubs key off the
 * address).  The name rides in the interactive advertisement instead. */
#define BTHOME_PAYLOAD_LEN     18

static uint8_t bthome_payload[BTHOME_PAYLOAD_LEN];
static uint8_t bthome_packet_id;

/* Illuminance is uint24 in centi-lux, so this is the largest value that fits. */
#define ILLUMINANCE_CENTI_MAX  0xFFFFFFU

static void bthome_build(int32_t temp_mdeg, uint32_t humid_mpct,
			 uint32_t lux, uint32_t vdd_mv, uint8_t battery_pct)
{
	/* Round to the stored resolution rather than truncating, so 23.999 C reads
	 * as 24.00 rather than 23.99. */
	int32_t t_centi = (temp_mdeg >= 0) ? (temp_mdeg + 5) / 10 : (temp_mdeg - 5) / 10;
	uint32_t h_centi = (humid_mpct + 5) / 10;
	uint32_t l_centi = lux;
	uint8_t *p = bthome_payload;

	if (t_centi > INT16_MAX) {
		t_centi = INT16_MAX;
	} else if (t_centi < INT16_MIN) {
		t_centi = INT16_MIN;
	}
	if (h_centi > UINT16_MAX) {
		h_centi = UINT16_MAX;
	}
	/* lux -> centi-lux can overflow uint24 above ~167 klx; clamp instead. */
	if (l_centi > ILLUMINANCE_CENTI_MAX / 100U) {
		l_centi = ILLUMINANCE_CENTI_MAX;
	} else {
		l_centi *= 100U;
	}
	if (vdd_mv > UINT16_MAX) {
		vdd_mv = UINT16_MAX;
	}

	*p++ = BTHOME_DEVICE_INFO;

	/* One id per measurement, repeated across the burst, so a hub collapses the
	 * duplicate packets into a single update. */
	*p++ = BTHOME_ID_PACKET;
	*p++ = bthome_packet_id++;

	*p++ = BTHOME_ID_BATTERY;
	*p++ = battery_pct;

	*p++ = BTHOME_ID_TEMPERATURE;
	sys_put_le16((uint16_t)(int16_t)t_centi, p);
	p += 2;

	*p++ = BTHOME_ID_HUMIDITY;
	sys_put_le16((uint16_t)h_centi, p);
	p += 2;

	*p++ = BTHOME_ID_ILLUMINANCE;
	*p++ = (uint8_t)(l_centi & 0xFF);
	*p++ = (uint8_t)((l_centi >> 8) & 0xFF);
	*p++ = (uint8_t)((l_centi >> 16) & 0xFF);

	*p++ = BTHOME_ID_VOLTAGE;
	sys_put_le16((uint16_t)vdd_mv, p);
	p += 2;

	__ASSERT(p == bthome_payload + BTHOME_PAYLOAD_LEN,
		 "BTHome payload length mismatch");
}

/* ── Advertising data ───────────────────────────────────────────────────── */

/*
 * BT_DATA_BYTES cannot express a runtime buffer, so the service-data element is
 * built by hand: 2 bytes of little-endian UUID followed by the payload.
 */
static uint8_t svc_data[2 + BTHOME_PAYLOAD_LEN];

/* Guards the beacon against a future field being added past the legacy limit:
 * the element costs a length byte and a type byte on top of the buffer. */
BUILD_ASSERT(2 + sizeof(svc_data) <= 31,
	     "BTHome service data element exceeds the 31-byte legacy adv limit");

static struct bt_data ad_beacon_rt[] = {
	{ .type = BT_DATA_SVC_DATA16,
	  .data_len = sizeof(svc_data),
	  .data = svc_data },
};

/*
 * Legacy advertising allows 31 bytes, and flags (3) + name (10) + BTHome service
 * data (22) is 35.  Since interactive mode is connectable it is also scannable,
 * so the service data goes in the scan response: a phone sees the name in the
 * advertisement, and nothing is lost.
 */
static const struct bt_data ad_interactive[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_data sd_interactive[] = {
	{ .type = BT_DATA_SVC_DATA16,
	  .data_len = sizeof(svc_data),
	  .data = svc_data },
};

/*
 * USE_IDENTITY on both: hubs identify a BTHome device by its Bluetooth address,
 * so a rotating private address would make it reappear as a new device.  With
 * CONFIG_BT_PRIVACY off this is already the behaviour, but stating it means a
 * later privacy build cannot silently break the integration.
 */
#define ADV_BEACON_PARAM                                                       \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY,                            \
			BT_GAP_ADV_SLOW_INT_MIN, BT_GAP_ADV_SLOW_INT_MAX, NULL)

#define ADV_INTERACTIVE_PARAM                                                  \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,       \
			BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

/* ── State ──────────────────────────────────────────────────────────────── */

enum adv_state {
	ADV_OFF,
	ADV_BEACON,
	ADV_INTERACTIVE,
};

static enum adv_state adv_state;
static bool ready;
static uint8_t conn_count;

static void adv_stop_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(adv_stop_work, adv_stop_work_fn);

/* Guards the advertising state against the measurement thread and the button
 * work racing each other. */
static K_MUTEX_DEFINE(adv_lock);

/* ── Cached sensor values for GATT reads ────────────────────────────────── */

static int16_t  ess_temp_centi;   /* 0.01 deg C, sint16  */
static uint16_t ess_humid_centi;  /* 0.01 %,     uint16  */
static uint32_t ess_illum_centi;  /* 0.01 lux,   uint24  */

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint8_t le[2];

	sys_put_le16(*(const uint16_t *)attr->user_data, le);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, le, sizeof(le));
}

static ssize_t read_u24(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint32_t v = *(const uint32_t *)attr->user_data;
	uint8_t le[3] = {
		(uint8_t)(v & 0xFF),
		(uint8_t)((v >> 8) & 0xFF),
		(uint8_t)((v >> 16) & 0xFF),
	};

	return bt_gatt_attr_read(conn, attr, buf, len, offset, le, sizeof(le));
}

/* Illuminance (0x2AFB) has no BT_UUID_* macro in this SDK. */
#define BT_UUID_ILLUMINANCE  BT_UUID_DECLARE_16(0x2AFB)

BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &ess_temp_centi),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &ess_humid_centi),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_ILLUMINANCE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u24, NULL, &ess_illum_centi),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ── Battery estimate ───────────────────────────────────────────────────── */
/*
 * A CR2 (Li-MnO2) holds ~2.9-3.0 V for most of its life then falls off a cliff,
 * so no linear map from open-circuit voltage is honest.  This table is shaped to
 * that curve: nearly flat at the top, steep at the bottom.  It is still only an
 * estimate — measuring the sag under the display-refresh load would track the
 * cell's rising internal resistance and give a far better state of charge.
 */
static uint8_t battery_percent(uint32_t mv)
{
	static const struct { uint16_t mv; uint8_t pct; } curve[] = {
		{ 3050, 100 }, { 3000, 90 }, { 2950, 75 }, { 2900, 60 },
		{ 2850, 45 },  { 2800, 30 }, { 2700, 15 }, { 2600, 5 },
		{ 2400, 0 },
	};

	if (mv >= curve[0].mv) {
		return 100;
	}
	for (size_t i = 1; i < ARRAY_SIZE(curve); i++) {
		if (mv >= curve[i].mv) {
			/* Linear inside the bracket we landed in. */
			uint32_t span_mv = curve[i - 1].mv - curve[i].mv;
			uint32_t span_pct = curve[i - 1].pct - curve[i].pct;

			return (uint8_t)(curve[i].pct +
					 ((mv - curve[i].mv) * span_pct + span_mv / 2) / span_mv);
		}
	}
	return 0;
}

/* ── Advertising control ────────────────────────────────────────────────── */

/* Caller must hold adv_lock. */
static int adv_restart(enum adv_state want)
{
	const struct bt_data *ad;
	size_t ad_len;
	int rc;

	if (adv_state != ADV_OFF) {
		rc = bt_le_adv_stop();
		if (rc) {
			LOG_WRN("bt_le_adv_stop: %d", rc);
		}
		adv_state = ADV_OFF;
	}

	if (want == ADV_OFF) {
		return 0;
	}

	if (want == ADV_INTERACTIVE) {
		ad = ad_interactive;
		ad_len = ARRAY_SIZE(ad_interactive);
		rc = bt_le_adv_start(ADV_INTERACTIVE_PARAM, ad, ad_len,
				     sd_interactive, ARRAY_SIZE(sd_interactive));
	} else {
		ad = ad_beacon_rt;
		ad_len = ARRAY_SIZE(ad_beacon_rt);
		rc = bt_le_adv_start(ADV_BEACON_PARAM, ad, ad_len, NULL, 0);
	}

	if (rc) {
		LOG_ERR("bt_le_adv_start(%d): %d", (int)want, rc);
		return rc;
	}

	adv_state = want;
	return 0;
}

static void adv_stop_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&adv_lock, K_FOREVER);

	/* A live connection outlives its advertising window — dropping the link
	 * because a timer expired would be hostile. */
	if (conn_count > 0) {
		k_work_reschedule(&adv_stop_work, K_MSEC(INTERACTIVE_MS));
		k_mutex_unlock(&adv_lock);
		return;
	}

	(void)adv_restart(ADV_OFF);
	k_mutex_unlock(&adv_lock);
}

/* ── Connection callbacks ───────────────────────────────────────────────── */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("BLE connect error %u", err);
		return;
	}

	k_mutex_lock(&adv_lock, K_FOREVER);
	conn_count++;
	/* The controller stops advertising on connect. */
	adv_state = ADV_OFF;
	k_mutex_unlock(&adv_lock);

	LOG_INF("BLE connected");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE disconnected (reason %u)", reason);

	k_mutex_lock(&adv_lock, K_FOREVER);
	if (conn_count > 0) {
		conn_count--;
	}
	/* Give the user another window to reconnect, then fall silent. */
	(void)adv_restart(ADV_INTERACTIVE);
	k_work_reschedule(&adv_stop_work, K_MSEC(INTERACTIVE_MS));
	k_mutex_unlock(&adv_lock);
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

/* ── Public API ─────────────────────────────────────────────────────────── */

int ble_init(void)
{
	int rc;

	/* Publish something valid before the radio comes up, so a hub that catches
	 * the very first burst does not read zeros. */
	svc_data[0] = (uint8_t)(BTHOME_UUID16 & 0xFF);
	svc_data[1] = (uint8_t)(BTHOME_UUID16 >> 8);
	bthome_build(0, 0, 0, 0, 0);
	memcpy(&svc_data[2], bthome_payload, sizeof(bthome_payload));

	rc = bt_enable(NULL);
	if (rc) {
		LOG_ERR("bt_enable failed: %d", rc);
		return rc;
	}

	ready = true;
	LOG_INF("BLE ready (BTHome v2 beacon + ESS/BAS)");
	return 0;
}

void ble_publish(int32_t temp_mdeg, uint32_t humid_mpct, uint32_t lux, uint32_t vdd_mv)
{
	uint8_t pct = battery_percent(vdd_mv);
	uint32_t l_centi;

	if (!ready) {
		return;
	}

	/* GATT cache (read back by a connected client). */
	ess_temp_centi = (int16_t)((temp_mdeg >= 0) ? (temp_mdeg + 5) / 10
						   : (temp_mdeg - 5) / 10);
	ess_humid_centi = (uint16_t)((humid_mpct + 5) / 10);
	l_centi = (lux > ILLUMINANCE_CENTI_MAX / 100U) ? ILLUMINANCE_CENTI_MAX : lux * 100U;
	ess_illum_centi = l_centi;

	(void)bt_bas_set_battery_level(pct);

	bthome_build(temp_mdeg, humid_mpct, lux, vdd_mv, pct);
	memcpy(&svc_data[2], bthome_payload, sizeof(bthome_payload));

	k_mutex_lock(&adv_lock, K_FOREVER);

	if (adv_state == ADV_INTERACTIVE || conn_count > 0) {
		/* Leave the interactive window alone; just refresh what it carries so a
		 * phone sitting on the connection sees live values. */
		if (adv_state == ADV_INTERACTIVE) {
			int rc = bt_le_adv_update_data(ad_interactive,
						       ARRAY_SIZE(ad_interactive),
						       sd_interactive,
						       ARRAY_SIZE(sd_interactive));
			if (rc) {
				LOG_WRN("adv update failed: %d", rc);
			}
		}
	} else {
		if (adv_restart(ADV_BEACON) == 0) {
			k_work_reschedule(&adv_stop_work, K_MSEC(BEACON_BURST_MS));
		}
	}

	k_mutex_unlock(&adv_lock);
}

void ble_start_interactive(void)
{
	if (!ready) {
		return;
	}

	k_mutex_lock(&adv_lock, K_FOREVER);
	if (adv_restart(ADV_INTERACTIVE) == 0) {
		k_work_reschedule(&adv_stop_work, K_MSEC(INTERACTIVE_MS));
		LOG_INF("interactive advertising, %d s", INTERACTIVE_MS / 1000);
	}
	k_mutex_unlock(&adv_lock);
}

