/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_H__
#define __BLE_SVCS_H__

#include "AppDemux.h"
#include "ble_svcs_cmd.h"

typedef void (*nus_data_handler_t)(const APP_CMD_t data);

void ble_svcs_init(void);
void ble_svcs_event_handler(bsp_event_t event);
void ble_svcs_cmd(BLE_CMD_t ble_cmd, uint16_t data);
void ble_svcs_send_euler_angles(float& roll, float& pitch, float& yaw);
void ble_svcs_advertising_start(void);
#ifdef BLE_CONSOLE_AVAILABLE
void ble_svcs_register(nus_data_handler_t data_handler_fn);
void ble_svcs_send_client_notification(uint8_t *p_data, const size_t len);
bool ble_svcs_connected();
#endif

#endif
