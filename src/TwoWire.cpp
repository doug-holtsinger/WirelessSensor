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
**/

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "fds.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "boards.h"

#include "TwoWire.h"


/**
 * @brief TWI initialization.
 */
TwoWire::TwoWire(void)
{
    ret_code_t err_code;

    p_xfer_stat = new TwoWireXferStatus();
    p_xfer_stat->m_xfer_done = true;
    p_xfer_stat->m_xfer_err_code = NRF_SUCCESS;

    const nrfx_twi_config_t twi_config = {
       .scl                = TWI0_SCL_PIN, 
       .sda                = TWI0_SDA_PIN, 
       .frequency          = (nrf_twi_frequency_t)NRFX_TWI_DEFAULT_CONFIG_FREQUENCY,
       .interrupt_priority = NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY,
       .hold_bus_uninit    = NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT 
    };

    err_code = nrfx_twi_init(&m_twi, &twi_config, twi_handler, (void *)p_xfer_stat);
    // FIXME failure possible inside constructor.
    APP_ERROR_CHECK(err_code);
    nrfx_twi_enable(&m_twi);
}

uint8_t TwoWire::IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    ret_code_t err_code = NRF_SUCCESS;

    nrfx_twi_xfer_desc_t xfer = NRFX_TWI_XFER_DESC_TXRX(
		    address ,
		    &RegisterAddr,
		    sizeof(RegisterAddr),
		    pBuffer,
		    NumByteToRead);

    p_xfer_stat->m_xfer_done = false;

    err_code = nrfx_twi_xfer(&m_twi, &xfer, 0);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // wait for transfer to finish before returning
    while (!p_xfer_stat->m_xfer_done) {}

    return p_xfer_stat->m_xfer_err_code; 
}

uint8_t TwoWire::IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    ret_code_t err_code = NRF_SUCCESS;
#define TWI_WRITE_BUFFER_SIZE_MAX 32
    uint8_t buffer[TWI_WRITE_BUFFER_SIZE_MAX];
    uint8_t num_bytes_write = NumByteToWrite + 1;

    if (NumByteToWrite >= TWI_WRITE_BUFFER_SIZE_MAX)
        return NRF_ERROR_DATA_SIZE;   // FIXME

    buffer[0] = RegisterAddr;
    for (int i=1 ; i <= NumByteToWrite ; i++) {
	if (i==TWI_WRITE_BUFFER_SIZE_MAX)
		break;
        buffer[i] = pBuffer[i-1];
    }

    nrfx_twi_xfer_desc_t xfer = NRFX_TWI_XFER_DESC_TX(
		    address,
		    buffer,
		    num_bytes_write);

    p_xfer_stat->m_xfer_done = false;

    err_code = nrfx_twi_xfer(&m_twi, &xfer, 0);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // wait for transfer to finish before returning
    while (!p_xfer_stat->m_xfer_done) {}

    return p_xfer_stat->m_xfer_err_code; 
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrfx_twi_evt_t const * p_event, void * p_context)
{
    volatile TwoWireXferStatus* p_xfer_stat = (volatile TwoWireXferStatus *)p_context;

    switch (p_event->type)
    {
        case NRFX_TWI_EVT_DONE :
            p_xfer_stat->m_xfer_err_code = NRF_SUCCESS;
            break;
        case NRFX_TWI_EVT_ADDRESS_NACK:
            p_xfer_stat->m_xfer_err_code = NRFX_ERROR_DRV_TWI_ERR_ANACK;
            break;
        case NRFX_TWI_EVT_DATA_NACK:
            p_xfer_stat->m_xfer_err_code = NRFX_ERROR_DRV_TWI_ERR_DNACK;
            break;
        case NRFX_TWI_EVT_OVERRUN:
            p_xfer_stat->m_xfer_err_code = NRFX_ERROR_DRV_TWI_ERR_OVERRUN;
            break;
        default:
            p_xfer_stat->m_xfer_err_code = NRFX_ERROR_INTERNAL;
            break;
    }
    
    p_xfer_stat->m_xfer_done = true;
}


