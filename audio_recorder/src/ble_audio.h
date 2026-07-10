/*
 * BLE Audio Streaming — peripheral with fixed passkey pairing,
 * custom GATT audio + battery service.
 */
#ifndef BLE_AUDIO_H
#define BLE_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

/* Fixed 6-digit passkey used for SMP pairing. The phone side must
 * enter this exact value when prompted by the OS pairing dialog. */
#define BLE_FIXED_PASSKEY 123456u

/* Initialize BLE stack, load bonds. */
int ble_audio_init(void);

/* Start advertising.  If pairing_mode is true, the device accepts new bonds.
 * Otherwise it only reconnects to the last bonded peer. */
int ble_audio_start_advertising(bool pairing_mode);

/* Send one opus packet via GATT notification.  Returns 0 on success,
 * -ENOTCONN if not connected, -EAGAIN if notifications are not enabled. */
int ble_audio_send(const uint8_t *data, uint16_t len);

/* True when a central is connected and audio notifications are enabled. */
bool ble_audio_is_connected(void);

/* True when any central is currently connected (regardless of CCC). */
bool ble_audio_has_conn(void);

/* Delete all stored bonds. */
void ble_audio_clear_bonds(void);

/* Update the recording-status byte returned by the audio read
 * characteristic (1 = recording, 0 = idle). */
void ble_audio_set_recording(bool recording);

/* Update the battery voltage exposed over GATT (millivolts).
 * Notifies connected peers that subscribed to battery notifications. */
void ble_audio_set_battery_mv(uint16_t mv);

/* Last reported battery voltage (millivolts). */
uint16_t ble_audio_get_battery_mv(void);

/* True after the central has written the correct password (123456). */
bool ble_audio_is_authed(void);

#endif /* BLE_AUDIO_H */
