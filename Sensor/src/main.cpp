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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_twi.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bsp.h"
#include "boards.h"

#include "app_config.h"
#include "imu.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"


IMU imu;
static uint32_t cmd_get_cnt = 0;

/* UART defines */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



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
            if (ep)
            {
                err_code_orig = ep->err_code;
                line_num = ep->line_num & 0xFFFF;
                p_file = ep->p_file_name;
            }
	    break;
	case NRF_FAULT_ID_SDK_ASSERT:
            err_code_orig = 0xEEEE; 
            if (ap)
            {
                line_num = ap->line_num & 0xFFFF;
                p_file = ap->p_file_name;
            }
	    break;
	case NRF_FAULT_ID_SD_ASSERT:
            // NRF softdevice fault assert
            err_code_orig = 0xEEE0;
            if (ap)
            {
                line_num = ap->line_num & 0xFFFF;
                p_file = ap->p_file_name;
            }
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

     NRF_LOG_ERROR("ERROR: id: 0x%x pc: 0x%x err_code: 0x%x File: %s Line: %d\n", id, pc, err_code_orig, p_file, line_num);

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


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            // FIXME sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
        case BSP_EVENT_WHITELIST_OFF:
            ble_svcs_event_handler(event);
            break;
#if 0
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
#endif

        default:
            break;
    }
}


/**@brief Function for initializing the board
 *
 */
static void board_init()
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    /* UART initialization */
#ifdef SERIAL_CONSOLE_AVAILABLE
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,			/* use_parity value */
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
#endif

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


static void interpret_app_cmd(const uint8_t cmd, IMU_CMD_t& imu_cmd, BLE_CMD_t& ble_cmd)
{
    imu_cmd = IMU_NOCMD;
    ble_cmd = BLE_NOCMD;
    if (cmd >= IMU_CMD_MIN && cmd <= IMU_CMD_MAX)
    {
        imu_cmd = static_cast<IMU_CMD_t>(cmd);
    } else {
        ble_cmd = static_cast<BLE_CMD_t>(cmd);
    }
}

void exec_app_cmd(const uint8_t cmd)
{
    IMU_CMD_t imu_cmd = IMU_NOCMD;
    BLE_CMD_t ble_cmd = BLE_NOCMD;

    interpret_app_cmd(cmd, imu_cmd, ble_cmd);
    if (imu_cmd != IMU_NOCMD)
    {
        imu.cmd(imu_cmd);
    }
    if (ble_cmd != BLE_NOCMD)
    {
        ble_svcs_cmd(ble_cmd, 0);
    }
}

#ifdef SERIAL_CONSOLE_AVAILABLE
static void check_app_cmd(void)
{
    uint8_t cmd;
    ret_code_t err_code;

    err_code = app_uart_get(&cmd);
    if (err_code == NRF_SUCCESS) {
        exec_app_cmd(cmd);
    }
}
#endif


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

    ble_svcs_timers_init();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    float roll, pitch, yaw;

    // Initialize.
    log_init();
    timers_init();
    board_init();
    power_management_init();

    // Start execution.
    NRF_LOG_INFO("AHRS example started.");

    // Initialize BLE 
    ble_svcs_init();
 
    // Start IMU
    imu = IMU();
    imu.init();

    // Start BLE Advertising
    ble_svcs_application_timers_start();
    ble_svcs_advertising_start();
  
    // Enter main loop.
    for (;; cmd_get_cnt++)
    {
#ifdef SERIAL_CONSOLE_AVAILABLE
        if ((cmd_get_cnt & 0xFF) == 0)
        {
	    check_app_cmd();
        }
#endif

        imu.update();

        imu.get_angles(roll, pitch, yaw);
        ble_svcs_send_euler_angles(roll, pitch, yaw);

#ifdef SERIAL_CONSOLE_AVAILABLE
        if ((cmd_get_cnt & 0x0F) == 0)
        {

#endif
            imu.send_all_client_data();
#ifdef SERIAL_CONSOLE_AVAILABLE
        }
#endif
    }
}


