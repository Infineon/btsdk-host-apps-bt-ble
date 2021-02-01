/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "wiced_hci.h"
#include "wiced_hci_audio_src.h"
#include "wiced_hci_ag.h"
#include "app_host.h"
#include <string.h>

bool app_host_ag_connect(uint8_t bda[6])
{

    wiced_hci_bt_ag_connect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if(p_dev && (p_dev->m_ag_handle != WICED_NULL_HANDLE))
    {
        app_host_log("AG already connected");
        return false;
    }

    app_host_log("Sending AG Connect");

    memcpy(data.bda, bda, BDA_LEN);
    return wiced_hci_ag_connect(&data);
}


bool app_host_ag_disconnect(uint8_t bda[6])
{
    wiced_hci_bt_ag_disconnect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(p_dev && (p_dev->m_ag_handle != WICED_NULL_HANDLE))
    {
        app_host_log("Sending AG disconnect");
        data.handle = p_dev->m_ag_handle;

        p_dev->m_ag_handle = WICED_NULL_HANDLE;
        p_dev->m_conn_type &= ~WICED_CONNECTION_TYPE_AG;
        p_dev->m_device_state.ag_state.ag_audio_opened = false;
        return wiced_hci_ag_disconnect(&data);
    }

    app_host_log("Device not connected as AG");
    return false;

}


bool app_host_ag_audio_open(uint8_t bda[6])
{
    wiced_hci_bt_ag_audio_open_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(p_dev && (p_dev->m_ag_handle != WICED_NULL_HANDLE))
    {
        app_host_log("Sending AG open");
        data.handle = p_dev->m_ag_handle;
        return wiced_hci_ag_audio_open(&data);
    }

    app_host_log("Device not connected as AG");
    return false;
}


bool app_host_ag_audio_close(uint8_t bda[6])
{
    wiced_hci_bt_ag_audio_close_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(p_dev && (p_dev->m_ag_handle != WICED_NULL_HANDLE))
    {
        app_host_log("Sending AG close");
        data.handle = p_dev->m_ag_handle;
        p_dev->m_device_state.ag_state.ag_audio_opened = false;
        return wiced_hci_ag_audio_close(&data);
    }

    app_host_log("Device not connected as AG");
    return false;
}


void app_host_ag_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    uint8_t    bda[6];
    wiced_hci_bt_device_t *device = 0;
    uint16_t  handle = 0;
    int i;

    switch (opcode)
    {
        // AG connected with peer
        case HCI_CONTROL_AG_EVENT_OPEN:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            if (p_data[8] == HCI_CONTROL_HF_STATUS_SUCCESS)
            {
                for (i = 0; i < 6; i++)
                    bda[5 - i] = p_data[2 + i];

                // find device in the list with received address and save the connection handle
                if ((device = app_host_find_device(bda)) == 0)
                    device = app_host_add_device(bda);

                device->m_ag_handle = handle;
                device->m_conn_type |= WICED_CONNECTION_TYPE_AG;

                device->m_device_state.ag_state.ag_audio_opened = false;

                app_host_log("HCI_CONTROL_AG_EVENT_OPEN");
            }
        }
        break;

        // AG diconnected from peer
        case HCI_CONTROL_AG_EVENT_CLOSE:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AG, handle);

            if(device)
            {
                device->m_ag_handle = WICED_NULL_HANDLE;
                device->m_conn_type &= ~WICED_CONNECTION_TYPE_AG;

                device->m_device_state.ag_state.ag_audio_opened = false;

                app_host_log("HCI_CONTROL_AG_EVENT_CLOSE");
            }
        }
        break;

        case HCI_CONTROL_AG_EVENT_CONNECTED:
        {
        }
        break;

        case HCI_CONTROL_AG_EVENT_AUDIO_OPEN:
        {
            handle   = (uint16_t) (p_data[0] | (p_data[1] << 8));
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AG, handle);

            if(device)
            {
                device->m_device_state.ag_state.ag_audio_opened = true;
            }
            app_host_log("HCI_CONTROL_AG_EVENT_AUDIO_OPEN");
        }
        break;

        case HCI_CONTROL_AG_EVENT_AUDIO_CLOSE:
        {
            handle   = (uint16_t) (p_data[0] | (p_data[1] << 8));
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AG, handle);

            if(device)
            {
                device->m_device_state.ag_state.ag_audio_opened = false;
            }
            app_host_log("HCI_CONTROL_AG_EVENT_AUDIO_CLOSE");
        }
        break;
    }

    app_host_handle_event(opcode, p_data, len);

}
