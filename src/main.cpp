/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_uart.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "boards.h"

#include "TwoWire.h"
#include "LSM6DS3Sensor.h"
#include "LIS3MDLSensor.h"
#include "MahonyAHRS.h"

#include "nrfx_twi.h"

/* UART defines */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

/* print defines */
#define PRINTF_FLOAT_FORMAT " %c%ld.%02ld"
#define PRINTF_FLOAT_VALUE(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)


/* DEBUG defines */
// #define USE_BLE 1

LSM6DS3Sensor* AccGyr;
LIS3MDLSensor* Magneto;
TwoWire* dev_i2c;

static uint16_t heart_rate;

static bool calibrate_enable = false;
static bool calibrate_reset = false;
static bool show_pitch = true;
static bool show_yaw   = false;
static bool show_roll  = false;
unsigned int ideal_data[3] = { 0, 0, 0 };
static bool zero_data[3] = { false, false, false };

static int32_t accelerometer_uncal[3];
static int32_t accelerometer_cal[3];
#ifdef ACCEL_CALIBRATE
static int32_t accelerometer_min[3] = { 0, 0, 0 };
static int32_t accelerometer_max[3] = { 0, 0, 0 };
#endif

#define GYROSCOPE_MAX_ALLOWED  0x1000

static int32_t gyroscope_uncal[3];
// static uint8_t gyroscope_uncal_bytes[6];
static float gyroscope_cal[3];
static int32_t gyroscope_min[3] =  { 0, 0, 0 };
static int32_t gyroscope_max[3] =  { 0, 0, 0 };
static int32_t gyroscope_sensitivity = 17;

static int32_t magnetometer_uncal[3];
static int32_t magnetometer_cal[3];
static int32_t magnetometer_min[3] = { 0, 0, 0 };
static int32_t magnetometer_max[3] = { 0, 0, 0 };
static int xmit_sensor_data = 2;
static uint32_t show_input_ahrs = 0;
static float gx, gy, gz, ax, ay, az, mx, my, mz;


#define DEVICE_NAME                         "Nordic_HRM"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                    BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(1000)                   /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                1                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      1                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_rr_interval_timer_id);                              /**< RR interval timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool     m_rr_interval_enabled = false;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//DSH-FIXME
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    assert_info_t* ap = (assert_info_t *)info;
    error_info_t*  ep = (error_info_t *)info;

    ret_code_t err_code_orig = 0;
    uint16_t line_num = 0;
    uint8_t const * p_file = NULL;

    switch (id) 
    {
	case NRF_FAULT_ID_SDK_ERROR:
            err_code_orig = ep->err_code;
            line_num = ep->line_num & 0xFFFF;
            p_file = ep->p_file_name;
	    break;
	case NRF_FAULT_ID_SDK_ASSERT:
            err_code_orig = 0xEEEE; 
            line_num = ap->line_num & 0xFFFF;
            p_file = ap->p_file_name;
	    break;
	case NRF_FAULT_ID_SD_ASSERT:
            // NRF softdevice fault assert
            err_code_orig = 0xEEE0;
	    break;
	case NRF_FAULT_ID_APP_MEMACC:
            // NRF invalid memory access 
            err_code_orig = 0xEEE1;
	    break;
        default:
	    // Unknown error
            err_code_orig = 0xEEE2; 
	    break;
    }

    bsp_board_leds_off();
    bsp_board_led_on(BSP_BOARD_LED_1);
    bsp_board_led_off(BSP_BOARD_LED_2);
    bsp_board_led_off(BSP_BOARD_LED_3);

     NRF_LOG_ERROR("ERROR: id: 0x%x err_code: 0x%x File: %s Line: %d\n", id, err_code_orig, p_file, line_num);

    while (1) {}
}


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
#ifdef USE_BLE
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
#endif
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
#ifdef USE_BLE
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
#else
    sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

#endif
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

void cmd_get(uint32_t* cmd_cnt)
{
    uint8_t   rx_data;
    uint32_t err_code;

    err_code = app_uart_get(&rx_data);
    if (err_code == NRF_SUCCESS) {
        if (rx_data == 'm') 
        {
            // magnetometer
            xmit_sensor_data = 3;
            calibrate_enable = false;
        } else if (rx_data == 'g') 
        {
            // gyro
            xmit_sensor_data = 2;
            calibrate_enable = false;
        } else if (rx_data == 'a') 
        {
            // accelerometer
            xmit_sensor_data = 1;
            calibrate_enable = false;
        } else if (rx_data == 'q') 
        {
            xmit_sensor_data = 0;
            calibrate_enable = false;
        } else if (rx_data == 'i') 
        {
            show_input_ahrs = ( show_input_ahrs + 1 ) % 3;
        } else if (rx_data == 'c') {
            // calibrate command
            calibrate_enable = !calibrate_enable;
            if (calibrate_enable)
                printf("Calibrate Enable\r\n");
            else
                printf("Calibrate Disable\r\n");
        } else if (rx_data == 'e') {
            // reset calibration values
            calibrate_reset = true;
	    *cmd_cnt = 0;
            printf("Calibrate Reset\r\n");
        } else if (rx_data == 'y') {
            show_yaw = !show_yaw;
        } else if (rx_data == 'p') {
            show_pitch = !show_pitch;
        } else if (rx_data == 'r') {
            show_roll = !show_roll;
        } else if (rx_data == 'z') {
            if (xmit_sensor_data == 2) {
                zero_data[1] = !zero_data[1];
            if (zero_data[1]) 
                printf("Gyroscope Data Disabled\r\n");
            else
                printf("Gyroscope Data Enabled\r\n");
            } else if (xmit_sensor_data == 1) {
                zero_data[0] = !zero_data[0];
                if (zero_data[0]) 
                    printf("Accelerometer Data Disabled\r\n");
                else
                    printf("Accelerometer Data Enabled\r\n");
            } else if (xmit_sensor_data == 3) {
                zero_data[2] = !zero_data[2];
                if (zero_data[2]) 
                    printf("Magnetometer Data Disabled\r\n");
                else
                    printf("Magnetometer Data Enabled\r\n");
            }
        } else if (rx_data == 'd') {
            if (xmit_sensor_data == 2) {
                ideal_data[1] = ideal_data[1] ? 0 : 1;
            if (ideal_data[1]) 
                printf("Gyroscope Data Ideal\r\n");
            else
                printf("Gyroscope Data Real\r\n");
            } else if (xmit_sensor_data == 1) {
                ideal_data[0] = ideal_data[0] ? 0 : 1;
                if (ideal_data[0]) 
                    printf("Accelerometer Data Ideal\r\n");
                else
                    printf("Accelerometer Data Real\r\n");
            } else if (xmit_sensor_data == 3) {
                ideal_data[2] = ideal_data[2] ? 0 : 1;
                if (ideal_data[2]) 
                    printf("Magnetometer Data Ideal\r\n");
                else
                    printf("Magnetometer Data Real\r\n");
            }
        } else if (rx_data == 'o') {
            printf("Enter proportional gain (u) or down (d) : %lu\r\n", (uint32_t)(twoKp * 1000.0));
            rx_data = getchar();
            if (rx_data == 'u') 
                twoKp += 0.1f;
            else if (rx_data == 'd' && twoKp >= 0.1f)
                twoKp -= 0.1f;
            printf("twoKp : %lu\r\n", (uint32_t)(twoKp * 1000.0));
        } else if (rx_data == 'n') {
            printf("Enter integral gain (u) or down (d) : %lu\r\n", (uint32_t)(twoKi * 1000.0));
            rx_data = getchar();
            if (rx_data == 'u') 
                twoKi += 0.1f;
            else if (rx_data == 'd' && twoKi >= 0.1f)
                twoKi -= 0.1f;
            printf("twoKi : %lu\r\n", (uint32_t)(twoKi * 1000.0));
        } else if (rx_data == 's') {
                printf("Enter sampleFreq (u) or down (d) : %lu\r\n", (uint32_t)(sampleFreq));
        rx_data = getchar();
                if (rx_data == 'u') 
                    sampleFreq += 32.0f;
                else if (rx_data == 'd' && sampleFreq >= 32.0f)
                    sampleFreq -= 32.0f;
                printf("sampleFreq : %lu\r\n", (uint32_t)(sampleFreq));
        } else if (rx_data == 't') {
            printf("Enter gyro sensitivity (u) or down (d) : %ld\r\n", gyroscope_sensitivity);
            rx_data = getchar();
            if (rx_data == 'u') 
                gyroscope_sensitivity++; 
            else if (rx_data == 'd' && gyroscope_sensitivity >= 1)
                gyroscope_sensitivity--; 
            printf("gyroscope_sensitivity : %ld\r\n", gyroscope_sensitivity);
        } else {
            printf("Invalid command %c\r\n", rx_data);
        }
    }
}

void calibrate_data()
{
    //
    //  magnetometer calibration -- done while moving
    //
    if (xmit_sensor_data == 3 && calibrate_enable)
    {
        for (int i = 0 ; i < 3 ; i++) {
            if (magnetometer_min[i] == 0 && magnetometer_max[i] == 0) {
                magnetometer_min[i] = magnetometer_max[i] = magnetometer_uncal[i];
            } else if (magnetometer_uncal[i] < magnetometer_min[i]) {
                magnetometer_min[i] = magnetometer_uncal[i];
            } else if (magnetometer_uncal[i] > magnetometer_max[i]) {
                magnetometer_max[i] = magnetometer_uncal[i];
            }
        }
    }
    if (xmit_sensor_data == 3 && calibrate_reset)
    {
        calibrate_reset = false;
        for (int i = 0 ; i < 3 ; i++) {
            magnetometer_min[i] = magnetometer_max[i] = 0; 
            }
    }

    //
    // gyroscope calibration -- done while motionless
    //
    if (xmit_sensor_data == 2)
    {
        for (int i = 0 ; i < 3 ; i++) {
	    if (calibrate_enable)
            {
                if (gyroscope_min[i] == 0 && gyroscope_max[i] == 0) {
                    gyroscope_min[i] = gyroscope_max[i] = gyroscope_uncal[i];
                } else if (gyroscope_uncal[i] < gyroscope_min[i]) {
                    gyroscope_min[i] = gyroscope_uncal[i];
                } else if (gyroscope_uncal[i] > gyroscope_max[i]) {
                    gyroscope_max[i] = gyroscope_uncal[i];
                }
            } 
        }
    }
    if (xmit_sensor_data == 2 && calibrate_reset)
    {
        calibrate_reset = false;
        for (int i = 0 ; i < 3 ; i++) {
            gyroscope_min[i] = gyroscope_max[i] = 0; 
        }
    }

#ifdef ACCEL_CALIBRATE
    //
    // accelerometer calibration -- done while motionless 
    //
    if (xmit_sensor_data == 1 && calibrate_enable)
    {
        for (int i = 0 ; i < 3 ; i++) {
            if (accelerometer_min[i] == 0 && accelerometer_max[i] == 0) {
                accelerometer_min[i] = accelerometer_max[i] = accelerometer_uncal[i];
            } else if (accelerometer_uncal[i] < accelerometer_min[i]) {
                accelerometer_min[i] = accelerometer_uncal[i];
            } else if (accelerometer_uncal[i] > accelerometer_max[i]) {
                accelerometer_max[i] = accelerometer_uncal[i];
            }
        }
    }
    if (xmit_sensor_data == 1 && calibrate_reset)
    {
        calibrate_reset = false;
        for (int i = 0 ; i < 3 ; i++) {
            accelerometer_min[i] = accelerometer_max[i] = 0; 
        }
    }
#endif

    // adjust
    for (int i = 0 ; i < 3 ; i++) {
        magnetometer_cal[i] = magnetometer_uncal[i] - ((magnetometer_max[i] + magnetometer_min[i]) / 2);
#ifdef ACCEL_CALIBRATE
        accelerometer_cal[i] = accelerometer_uncal[i] - ((accelerometer_max[i] + accelerometer_min[i]) / 2);
#else
        accelerometer_cal[i] = accelerometer_uncal[i]; 
#endif
        gyroscope_cal[i] = ( gyroscope_uncal[i] - ((gyroscope_max[i] + gyroscope_min[i]) / 2) ) / (float)(1ULL << gyroscope_sensitivity) ;
    }

}

void AHRS() 
{
    if (xmit_sensor_data == 0)
    {
        gx = gyroscope_cal[0];
        gy = gyroscope_cal[1];
        gz = gyroscope_cal[2];
        ax = (float)accelerometer_cal[0];
        ay = (float)accelerometer_cal[1];
        az = (float)accelerometer_cal[2];
        mx = (float)magnetometer_cal[0];
        my = (float)magnetometer_cal[1];
        mz = (float)magnetometer_cal[2];
        if (zero_data[1])
        {
            gz = gy = gx = 0.0f;
        }
        if (zero_data[0])
        {
            az = ay = ax = 0.0f;
        }
        if (zero_data[2])
        {
            mz = my = mx = 0.0f;
        }
	if (ideal_data[0])
	{
            ax = 1000.0f; ay = 0.0f ; az = 0.0f;
	}
	if (ideal_data[1])
	{
            gx = 0.0f; gy = 0.0f ; gz = 0.0f;
	}
	if (ideal_data[2])
	{
            mx = 400.0f; my = 0.0f ; mz = 0.0f;
	}
        MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
        MahonyAHRSComputeAngles();
    }

}


void show_data(uint32_t cnt)
{

    if (xmit_sensor_data == 0) {
        if (show_input_ahrs == 1) {
            if (show_roll)
                   printf("gyro = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(gx), PRINTF_FLOAT_VALUE(gy), PRINTF_FLOAT_VALUE(gz) );
            if (show_pitch)
                  printf("acce = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(ax), PRINTF_FLOAT_VALUE(ay), PRINTF_FLOAT_VALUE(az) );
            if (show_yaw)
                   printf("magn = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(mx), PRINTF_FLOAT_VALUE(my), PRINTF_FLOAT_VALUE(mz) );
	} else if (show_input_ahrs == 2) {
           if (show_roll)
               printf("gyro normal = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(gxN), PRINTF_FLOAT_VALUE(gyN), PRINTF_FLOAT_VALUE(gzN) );
           if (show_pitch)
                printf("acce normal = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(axN), PRINTF_FLOAT_VALUE(ayN), PRINTF_FLOAT_VALUE(azN) );
           if (show_yaw)
                printf("magn normal = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(mxN), PRINTF_FLOAT_VALUE(myN), PRINTF_FLOAT_VALUE(mzN) );
        } else {
            if (show_roll)
                    printf("Roll  = " PRINTF_FLOAT_FORMAT " cnt = %lu\r\n", PRINTF_FLOAT_VALUE(roll), cnt);
            if (show_pitch)
                    printf("Pitch = " PRINTF_FLOAT_FORMAT " cnt = %lu\r\n", PRINTF_FLOAT_VALUE(pitch), cnt);
            if (show_yaw)
                    printf("Yaw   = " PRINTF_FLOAT_FORMAT " cnt = %lu\r\n", PRINTF_FLOAT_VALUE(yaw), cnt);
        }
        heart_rate = (int)q0;
    } else if (xmit_sensor_data == 1)
    {
        NRF_LOG_INFO("A = %04d %04d %04d  |  %04d %04d %04d ", 
            accelerometer_cal[0],
            accelerometer_cal[1],
            accelerometer_cal[2],
            accelerometer_uncal[0],
            accelerometer_uncal[1],
            accelerometer_uncal[2]
            );
        heart_rate = accelerometer_cal[0] ;
    } else if (xmit_sensor_data == 2)
    {
#if 1
        if (calibrate_enable)
        {
            NRF_LOG_INFO("G = max-min %08x %08x %08x  |  uncal %08x %08x %08x ", 
                gyroscope_max[0] - gyroscope_min[0],
                gyroscope_max[1] - gyroscope_min[1],
                gyroscope_max[2] - gyroscope_min[2],
                gyroscope_uncal[0],
                gyroscope_uncal[1],
                gyroscope_uncal[2]
            );
       } else {
            NRF_LOG_INFO("G cal X Y = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT ,
                PRINTF_FLOAT_VALUE(gyroscope_cal[0]),
                PRINTF_FLOAT_VALUE(gyroscope_cal[1])
            );
            NRF_LOG_INFO("G cal Z   = " PRINTF_FLOAT_FORMAT,
                PRINTF_FLOAT_VALUE(gyroscope_cal[2])
            );
       }
#else
        NRF_LOG_INFO("G = %01x %01x | %01x %01x | %01x %01x ", 
            gyroscope_uncal_bytes[0],
            gyroscope_uncal_bytes[1],
            gyroscope_uncal_bytes[2],
            gyroscope_uncal_bytes[3],
            gyroscope_uncal_bytes[4],
            gyroscope_uncal_bytes[5]
            );

#endif
        // heart_rate = gyroscope_cal[0] ;
    } else if (xmit_sensor_data == 3)
    {
        NRF_LOG_INFO("M = %04d %04d %04d  |  %04d %04d %04d ", 
            magnetometer_cal[0],
            magnetometer_cal[1],
            magnetometer_cal[2],
            magnetometer_uncal[0],
            magnetometer_uncal[1],
            magnetometer_uncal[2]
            );
        heart_rate = magnetometer_cal[0] ;
    }
}

void read_twi_sensor()
{
    LSM6DS3StatusTypeDef lsm_err_code = LSM6DS3_STATUS_OK;
    LIS3MDLStatusTypeDef lis_err_code = LIS3MDL_STATUS_OK;
    uint8_t reg_data = 0;
    static uint32_t cmd_get_cnt = 0;
    bool new_data_avail = false;
    static uint32_t new_data_cnt = 0;
    static uint32_t poll_cnt = 0;

    if ((cmd_get_cnt++ & 0xFF) == 0)
    {
        cmd_get_cnt++;
        cmd_get(&cmd_get_cnt);
    }

    //
    // Read LSM6DS3
    //
    lsm_err_code = AccGyr->ReadReg(LSM6DS3_ACC_GYRO_STATUS_REG, &reg_data);
    APP_ERROR_CHECK(lsm_err_code);

    // TBD -- can combine status of Accel and Gyro according to app note
    // Gyro status lags Accel status.
    if ((reg_data & LSM6DS3_ACC_GYRO_XLDA_MASK) == LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL) {
        new_data_avail = true;
        lsm_err_code = AccGyr->Get_X_Axes(accelerometer_uncal);
        APP_ERROR_CHECK(lsm_err_code);
    }
    if ((reg_data & LSM6DS3_ACC_GYRO_GDA_MASK) == LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL) {
        new_data_avail = true;
        lsm_err_code = AccGyr->Get_G_Axes(gyroscope_uncal);
        // lsm_err_code = AccGyr->Get_G_AxesRawBytes(gyroscope_uncal_bytes);
        // lsm_err_code = AccGyr->Get_AngularRate(gyroscope_uncal);
        APP_ERROR_CHECK(lsm_err_code);
    }

    //
    // Read LIS3MDL 
    //

    lis_err_code = Magneto->GetAxes(magnetometer_uncal);
    APP_ERROR_CHECK(lis_err_code);

    poll_cnt++;
    if (new_data_avail)
    {
	new_data_cnt++;
        calibrate_data();
        if (lsm_err_code != LSM6DS3_STATUS_OK || lis_err_code != LIS3MDL_STATUS_OK)
        {
            heart_rate = 0xAAAA;
            bsp_board_led_on(BSP_BOARD_LED_1);
        } else
        {
            AHRS();
            if ((cmd_get_cnt & 0xFF) == 0)
            {
                show_data(cmd_get_cnt);
		// printf("%ld %ld\r\n", new_data_cnt, poll_cnt);
	    }

        }
    }

}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void heart_rate_meas_timeout_handler(void * p_context)
{
#ifdef USE_BLE
    ret_code_t      err_code;
#endif

    UNUSED_PARAMETER(p_context);

    // heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
#ifdef USE_BLE
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
#endif

 
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
#ifdef USE_BLE
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
#endif
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, BLE_GATT_ATT_MTU_DEFAULT);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        NRF_LOG_INFO("%d on_conn_params_evt Disconnect", __LINE__);
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


#if 0
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code = 0;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
        case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

       case BLE_ADV_EVT_FAST_WHITELIST:
       case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Whitelist advertising.");
            break;

       case BLE_ADV_EVT_WHITELIST_REQUEST:
            NRF_LOG_INFO("Whitelist Request.");
            break;

       case BLE_ADV_EVT_PEER_ADDR_REQUEST: 
            NRF_LOG_INFO("Peer Address Request.");
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("No advertising.");
            // FIXME sleep_mode_enter();
            err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_OFF);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = bsp_indication_set(BSP_INDICATE_FATAL_ERROR);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .tx_phys = BLE_GAP_PHY_AUTO,
                .rx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            break;
        default:
            // No implementation needed.
            NRF_LOG_INFO("%s %d ble_evt_handler default handler for event %d", __FILE__, __LINE__, p_ble_evt->header.evt_id);
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            // FIXME sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            NRF_LOG_INFO("%d bsp_event_handler Disconnect", __LINE__);
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.config.ble_adv_on_disconnect_disabled = false;


    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);

    /* UART initialization */
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };



    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
#if 0
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
#endif

void twi_init(void)
{
    dev_i2c = new TwoWire();
}

/**
 */
void LSM6DS3SensorInit(void)
{
    AccGyr = new LSM6DS3Sensor(dev_i2c, TWI_ADDRESS_LSM6DS3);
    AccGyr->Enable_X();
    AccGyr->Enable_G();
    // AccGyr->Enable_G_Filter(LSM6DS3_ACC_GYRO_HPCF_G_16Hz32);
    // AccGyr->Enable_6D_Orientation();
}

void LIS3MDLSensorInit(void)
{
    Magneto = new LIS3MDLSensor(dev_i2c, TWI_ADDRESS_LIS3MDL);
    Magneto->Enable();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();
    twi_init();
    LSM6DS3SensorInit();
    LIS3MDLSensorInit();

    // Start execution.
    NRF_LOG_INFO("Heart Rate Sensor example started.");
    application_timers_start();
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        read_twi_sensor();
        // idle_state_handle();
    }
}


