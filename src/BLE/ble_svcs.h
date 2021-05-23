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


#endif
