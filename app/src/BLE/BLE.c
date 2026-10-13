#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include "BLE.h"

//function declration
static void ble_on_connected(struct bt_conn *conn, uint8_t err);
static void ble_on_disconnect(struct bt_conn *conn, uint8_t reason);
static void ble_on_recycled(void);

//data and module intialization
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = ble_on_connected,
	.disconnected = ble_on_disconnect,
    .recycled = ble_on_recycled,
};

LOG_MODULE_REGISTER(ble, LOG_LEVEL_DBG);

//functions implementation
static int start_advertising(void) {

    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }
    LOG_INF("Advertising as %s", CONFIG_BT_DEVICE_NAME);
    return 0;
}

int ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }
    LOG_INF("Bluetooth initialized");

    return start_advertising();
}

static void ble_on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Bluetooth connection failed (err %d)", err);
        return;
    }

    //Checks the address of the device thats currently connected
    const bt_addr_le_t *ble_addr = bt_conn_get_dst(conn);

    //Convert address to a printable string
    char buf[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(ble_addr, buf , ARRAY_SIZE(buf));


    LOG_DBG("Bluetooth connection successful! Currently connected at %s", buf);
}

static void ble_on_disconnect(struct bt_conn *conn, uint8_t reason) {

    LOG_DBG("Bluetooth connection disconnected (reason 0x%02x %s)", reason, bt_hci_err_to_str(reason));
}


static void ble_on_recycled(void) {
    start_advertising();
}
