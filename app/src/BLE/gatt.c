#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "pill_service.h"

// join the "ble" log module that BLE.c registered (REGISTER may only happen once)
LOG_MODULE_DECLARE(ble, LOG_LEVEL_DBG);

#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x32E5EE98, 0xDA77, 0x4CFD, 0x939F, 0x0E55EC5F11E0)

static const struct bt_uuid_128 pill_svc_uuid = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_VAL);

static const struct bt_uuid_128 pill_log_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x32E5EE98, 0xDA77, 0x4CFD, 0x939F, 0x0E55EC5F11E1));

static const struct bt_uuid_128 pill_template_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x32E5EE98, 0xDA77, 0x4CFD, 0x939F, 0x0E55EC5F11E2));

// Container pill count read
static const struct bt_uuid_128 pill_container_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x32E5EE98, 0xDA77, 0x4CFD, 0x939F, 0x0E55EC5F11E3));

// Storage. The Bluetooth callbacks run on the Bluetooth thread, while
// pill_service_get_template()/pill_service_log_dispense() will be called from
// the dispensing code's thread, so every access goes through this mutex.
K_MUTEX_DEFINE(pill_lock);
static struct log_entry latest_log;
static struct pill_template templates[PILL_CONTAINERS];
static struct pill_container containers[PILL_CONTAINERS];

// function declarations
static ssize_t read_log(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset);
static void log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t read_templates(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset);
static ssize_t write_template(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static ssize_t read_container(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset);
static void containers_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

// Attribute table. Index:  [0] service  [1] log declaration  [2] log value
//                          [3] log CCC  [4] template declaration  [5] template value
BT_GATT_SERVICE_DEFINE(pill_svc,
	BT_GATT_PRIMARY_SERVICE(&pill_svc_uuid.uuid),

	BT_GATT_CHARACTERISTIC(&pill_log_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_log, NULL, NULL),
	// lets the phone turn notifications for the log on and off
	BT_GATT_CCC(log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	// PERM_PREPARE_WRITE allows "long writes": a phone with a small MTU sends the
	// 34-byte template in pieces, and Zephyr reassembles them before write_template sees it
	BT_GATT_CHARACTERISTIC(&pill_template_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE | BT_GATT_PERM_PREPARE_WRITE,
			       read_templates, write_template, NULL),

    BT_GATT_CHARACTERISTIC(&pill_container_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_container, NULL, NULL),

    BT_GATT_CCC(containers_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#define LOG_VALUE_ATTR (&pill_svc.attrs[2])
#define CONTAINERS_VALUE_ATTR (&pill_svc.attrs[7])

// ---- dispense log ----

static ssize_t read_log(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	struct log_entry snapshot;

	// copy under the lock so the phone never reads a half-updated entry
	k_mutex_lock(&pill_lock, K_FOREVER);
	snapshot = latest_log;
	k_mutex_unlock(&pill_lock);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &snapshot, sizeof(snapshot));
}

static void log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("Dispense log notifications %s", (value == BT_GATT_CCC_NOTIFY) ? "on" : "off");
}

int pill_service_log_dispense(const struct log_entry *entry)
{
	if (entry->num_pills > PILL_CONTAINERS) {
		return -EINVAL;
	}

	k_mutex_lock(&pill_lock, K_FOREVER);
	latest_log = *entry;
	// zero the unused items so leftover memory never goes over the air
	memset(&latest_log.pills[entry->num_pills], 0,
	       sizeof(struct pill_map) * (PILL_CONTAINERS - entry->num_pills));
	struct log_entry snapshot = latest_log;
	k_mutex_unlock(&pill_lock);

	LOG_INF("Dispense logged at %u (%u pill types)", snapshot.timestamp, snapshot.num_pills);

	// NULL conn = send to every connected phone that turned notifications on
	int err = bt_gatt_notify(NULL, LOG_VALUE_ATTR, &snapshot, sizeof(snapshot));
	if (err == -ENOTCONN) {
		// nobody connected; the phone can still read the entry later
		return 0;
	}
	if (err) {
		LOG_WRN("Dispense log notify failed (err %d)", err);
	}
	return err;
}

// ---- pill templates ----

static ssize_t read_templates(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	struct pill_template snapshot[PILL_CONTAINERS];

	k_mutex_lock(&pill_lock, K_FOREVER);
	memcpy(snapshot, templates, sizeof(snapshot));
	k_mutex_unlock(&pill_lock);

	// containers that were never written are all zeros; label them so the phone can tell them apart
	for (uint8_t i = 0; i < PILL_CONTAINERS; i++) {
		snapshot[i].container = i;
	}

	// all containers at once (136 bytes); the phone reads it in pieces using offset
	return bt_gatt_attr_read(conn, attr, buf, len, offset, snapshot, sizeof(snapshot));
}

static ssize_t write_template(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct pill_template incoming;

	// a long write first asks "may I write here?" for each piece; say yes and
	// validate once Zephyr hands over the reassembled value
	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(incoming)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	// copy into a local first: never trust or modify the stored table until it's validated
	memcpy(&incoming, buf, sizeof(incoming));

	if (incoming.container >= PILL_CONTAINERS) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (incoming.pill_id == 0) {
		// clearing the container: store an all-zero entry
		uint8_t container = incoming.container;

		memset(&incoming, 0, sizeof(incoming));
		incoming.container = container;
	} else {
		// name must be non-empty and end with a terminator inside the field
		size_t name_len = strnlen(incoming.name, PILL_NAME_LEN);

		if (name_len == 0 || name_len == PILL_NAME_LEN) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
		if (incoming.dose == 0) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
		// zero everything after the name so equal names are equal byte-for-byte
		memset(&incoming.name[name_len], 0, PILL_NAME_LEN - name_len);
	}

	k_mutex_lock(&pill_lock, K_FOREVER);
	templates[incoming.container] = incoming;
	k_mutex_unlock(&pill_lock);

	if (incoming.pill_id == 0) {
		LOG_INF("Container %u cleared", incoming.container);
	} else {
		LOG_INF("Container %u: pill %u \"%s\", dose %u", incoming.container,
			incoming.pill_id, incoming.name, incoming.dose);
	}

	return len;
}

int pill_service_get_template(uint8_t container, struct pill_template *out)
{
	if (container >= PILL_CONTAINERS) {
		return -EINVAL;
	}

	k_mutex_lock(&pill_lock, K_FOREVER);
	*out = templates[container];
	k_mutex_unlock(&pill_lock);

	return (out->pill_id == 0) ? -ENOENT : 0;
}

//container 

static ssize_t read_container(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	struct pill_container snapshot[PILL_CONTAINERS];

	// copy under the lock so the phone never reads a half-updated entry
	k_mutex_lock(&pill_lock, K_FOREVER);
	memcpy(snapshot, containers, sizeof(snapshot));
	k_mutex_unlock(&pill_lock);

    // containers that were never written are all zeros; label them so the phone can tell them apart
	for (uint8_t i = 0; i < PILL_CONTAINERS; i++) {
		snapshot[i].container_num = i;
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &snapshot, sizeof(snapshot));
}

static void containers_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("Container read notification %s", (value == BT_GATT_CCC_NOTIFY) ? "on" : "off");
}


