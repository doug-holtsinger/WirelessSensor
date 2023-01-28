/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_H__
#define __BLE_SVCS_H__

    void ble_svcs_init(void);
    void ble_svcs_event_handler(bsp_event_t event);
    void ble_svcs_cmd(BLE_CMD_t ble_cmd, uint16_t data);
    void ble_svcs_send_euler_angles(float& roll, float& pitch, float& yaw);
    void ble_svcs_advertising_start(void);
#ifdef BLE_CONSOLE_AVAILABLE
    void ble_svcs_send_client_notification(uint8_t *p_data, const size_t len);
    bool ble_svcs_connected();
#endif

#endif
