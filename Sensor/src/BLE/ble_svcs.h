/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_H__
#define __BLE_SVCS_H__

    extern void ble_svcs_init(void);
    extern void ble_svcs_application_timers_start(void);
    extern void ble_svcs_advertising_start(void);
    extern void ble_svcs_event_handler(bsp_event_t event);
    extern void ble_svcs_timers_init(void);
    extern void ble_svcs_cmd(BLE_CMD_t ble_cmd, uint16_t data);
    extern void ble_svcs_send_euler_angles(float& roll, float& pitch, float& yaw);
    extern void ble_svcs_send_client_notification(uint8_t *p_data, const size_t len);
    extern bool ble_svcs_connected();

#endif
