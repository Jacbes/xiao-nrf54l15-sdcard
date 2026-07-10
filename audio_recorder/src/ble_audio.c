/*
 * BLE Audio Streaming — peripheral with fixed passkey pairing,
 * custom GATT audio + battery service.
 *
 * Custom service UUID:  12345678-1234-5678-1234-56789abcdef0
 * Audio characteristic: 12345678-1234-5678-1234-56789abcdef1 (notify)
 * Battery characteristic: 12345678-1234-5678-1234-56789abcdef2 (read+notify)
 *                        value = uint16 little-endian battery voltage, mV
 *
 * Pairing uses a fixed passkey (BLE_FIXED_PASSKEY = 123456). The device
 * has DisplayOnly IO capability (it "knows" the passkey and shows it via
 * the app_passkey callback), so the phone side prompts the user to enter
 * the passkey.
 */

#include "ble_audio.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>

/* ---- UUIDs ---- */
#define BT_UUID_AUDIO_SVC_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_AUDIO_DATA_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_AUDIO_BATT_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)
#define BT_UUID_AUDIO_PASS_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)

static const struct bt_uuid_128 audio_svc_uuid  = BT_UUID_INIT_128(BT_UUID_AUDIO_SVC_VAL);
static const struct bt_uuid_128 audio_data_uuid = BT_UUID_INIT_128(BT_UUID_AUDIO_DATA_VAL);
static const struct bt_uuid_128 audio_batt_uuid = BT_UUID_INIT_128(BT_UUID_AUDIO_BATT_VAL);
static const struct bt_uuid_128 audio_pass_uuid = BT_UUID_INIT_128(BT_UUID_AUDIO_PASS_VAL);

/* ---- State ---- */
static struct bt_conn *current_conn;
static bool ntf_enabled;
static volatile bool app_authed;  /* true after correct password write */

static uint16_t batt_mv_le;     /* battery voltage, little-endian, mV */
static bool batt_ntf_enabled;

/* Forward declaration: BT_GATT_SERVICE_DEFINE() (below) instantiates
 * `audio_svc` as a bt_gatt_service_static. The CCC callbacks need to
 * reference it to resolve the value attribute for notifications. */
extern const struct bt_gatt_service_static audio_svc;

/* Recording status reported by the audio "read" characteristic.
 * main.c updates this via ble_audio_set_recording(). */
static volatile uint8_t rec_status_byte;

void ble_audio_set_recording(bool recording)
{
	rec_status_byte = recording ? 1 : 0;
}

/* ---- GATT callbacks ---- */
static void audio_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ntf_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("BLE audio notify: %s\n", ntf_enabled ? "ON" : "OFF");
}

static ssize_t read_audio(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	/* Return a status byte: 1 = recording, 0 = idle */
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 (void *)&rec_status_byte, 1);
}

static void batt_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool was = batt_ntf_enabled;
	batt_ntf_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("BLE battery notify: %s\n", batt_ntf_enabled ? "ON" : "OFF");

	/* As soon as the peer subscribes, push the current value so the
	 * phone shows the battery state immediately after connecting. */
	if (batt_ntf_enabled && !was && current_conn) {
		bt_gatt_notify(current_conn, &audio_svc.attrs[4],
			       &batt_mv_le, sizeof(batt_mv_le));
	}
}

static ssize_t read_batt(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &batt_mv_le, sizeof(batt_mv_le));
}

/* ---- Password ---- */
static void pass_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	/* No notifications from password char; CCC exists only for
	 * future use or to match the Bluetooth spec layout. */
}

static ssize_t read_pass(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t status = app_authed ? 1 : 0;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &status, 1);
}

static ssize_t write_pass(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset,
			  uint8_t flags)
{
	const char correct[] = "123456";

	if (len != sizeof(correct) - 1 || memcmp(buf, correct, len) != 0) {
		printk("BLE password: wrong (len=%u)\n", len);
		return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
	}

	app_authed = true;
	printk("BLE password: OK, audio unlocked\n");
	return len;
}

/* ---- GATT service definition ----
 * Attribute layout (indices into audio_svc.attrs[]):
 *   [0] primary service
 *   [1] audio data characteristic declaration
 *   [2] audio data value            <- ble_audio_send() targets [1]->value
 *   [3] audio data CCC
 *   [4] battery characteristic declaration
 *   [5] battery value               <- battery notify targets [4]->value
 *   [6] battery CCC
 *   [7] password characteristic declaration
 *   [8] password value
 *   [9] password CCC
 */
BT_GATT_SERVICE_DEFINE(audio_svc,
	BT_GATT_PRIMARY_SERVICE(&audio_svc_uuid),
	BT_GATT_CHARACTERISTIC(&audio_data_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_audio, NULL, NULL),
	BT_GATT_CCC(audio_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&audio_batt_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_batt, NULL, NULL),
	BT_GATT_CCC(batt_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&audio_pass_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_pass, write_pass, NULL),
	BT_GATT_CCC(pass_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ---- Advertising data ---- */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_AUDIO_SVC_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ---- Connection callbacks ---- */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("BLE conn fail 0x%02x\n", err);
		return;
	}
	printk("BLE connected\n");
	if (current_conn) {
		bt_conn_unref(current_conn);
	}
	current_conn = bt_conn_ref(conn);
	/* Request authenticated pairing (passkey 123456). */
	bt_conn_set_security(conn, BT_SECURITY_L2);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("BLE disconnected (0x%02x)\n", reason);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	ntf_enabled = false;
	batt_ntf_enabled = false;
	app_authed = false;
	/* Auto-restart advertising */
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	if (err) {
		printk("BLE security fail %d\n", err);
		return;
	}
	printk("BLE security level %d\n", level);

	/* Once the link is encrypted, push the latest battery reading so the
	 * phone shows the battery state right away (even before the periodic
	 * update). */
	if (level >= BT_SECURITY_L1 && batt_mv_le != 0) {
		bt_gatt_notify(conn, &audio_svc.attrs[4],
			       &batt_mv_le, sizeof(batt_mv_le));
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

/* ---- Bond management (fixed passkey 123456) ---- */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	printk("BLE pairing passkey: %06u (enter on phone)\n", passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	printk("BLE pairing cancelled\n");
}

#if defined(CONFIG_BT_APP_PASSKEY)
static uint32_t auth_app_passkey(struct bt_conn *conn)
{
	/* Always offer the fixed passkey so the phone prompts for it. */
	printk("BLE offering fixed passkey %06u\n", BLE_FIXED_PASSKEY);
	return BLE_FIXED_PASSKEY;
}
#endif

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("BLE pairing %s\n", bonded ? "complete (bonded)" : "done (not bonded)");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("BLE pairing failed (%d)\n", reason);
}

static struct bt_conn_auth_cb auth_cb = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
#if defined(CONFIG_BT_APP_PASSKEY)
	.app_passkey = auth_app_passkey,
#endif
};

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* ---- Public API ---- */

int ble_audio_init(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("BLE init fail %d\n", err);
		return err;
	}
	printk("BLE initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);

	printk("BLE fixed passkey: %06u\n", BLE_FIXED_PASSKEY);
	return 0;
}

int ble_audio_start_advertising(bool pairing_mode)
{
	int err;

	/* If pairing mode, delete all bonds first. */
	if (pairing_mode) {
		ble_audio_clear_bonds();
		printk("BLE: pairing mode (bonds cleared)\n");
	}

	bt_le_adv_stop();
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			      ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("BLE adv start fail %d\n", err);
		return err;
	}
	printk("BLE advertising started (%s)\n",
	       pairing_mode ? "pairing" : "normal");
	return 0;
}

int ble_audio_send(const uint8_t *data, uint16_t len)
{
	if (!current_conn || !ntf_enabled) {
		return -ENOTCONN;
	}
	if (!app_authed) {
		return -EACCES;
	}

	/* audio_svc.attrs[1] is the audio-data characteristic declaration;
	 * bt_gatt_notify resolves it to the value handle + CCC. */
	return bt_gatt_notify(current_conn, &audio_svc.attrs[1], data, len);
}

bool ble_audio_is_connected(void)
{
	return current_conn != NULL && ntf_enabled;
}

bool ble_audio_has_conn(void)
{
	return current_conn != NULL;
}

void ble_audio_clear_bonds(void)
{
	int err = bt_unpair(BT_ID_DEFAULT, NULL);
	if (err) {
		printk("BLE clear bonds fail %d\n", err);
	} else {
		printk("BLE bonds cleared\n");
	}
}

void ble_audio_set_battery_mv(uint16_t mv)
{
	uint16_t le = sys_cpu_to_le16(mv);

	if (le == batt_mv_le) {
		return;
	}
	batt_mv_le = le;

	/* Notify subscribers (and all connected peers if any). */
	if (batt_ntf_enabled && current_conn) {
		bt_gatt_notify(current_conn, &audio_svc.attrs[4],
			       &batt_mv_le, sizeof(batt_mv_le));
	}
}

uint16_t ble_audio_get_battery_mv(void)
{
	return sys_le16_to_cpu(batt_mv_le);
}

bool ble_audio_is_authed(void)
{
	return app_authed;
}
