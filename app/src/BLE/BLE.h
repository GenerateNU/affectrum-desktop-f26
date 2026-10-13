#ifndef BLE_H
#define BLE_H

// Turns on the Bluetooth stack and starts advertising so phones can see the device.
// Returns 0 on success, or a negative error code from Zephyr if something failed.
int ble_init(void);


#endif
