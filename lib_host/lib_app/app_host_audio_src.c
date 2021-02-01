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



#include "app_host.h"

bool app_host_audio_src_connect(wiced_hci_bt_audio_source_connect_data_t *p_data)
{
    wiced_hci_bt_device_t* p_dev = app_host_find_device(p_data->bda);
    if(p_dev && (p_dev->m_audio_handle != WICED_NULL_HANDLE))
    {
        app_host_log("AV SRC already connected");
        return false;
    }

    app_host_log("Sending AV Connect");

    return wiced_hci_audio_src_connect(p_data);
}

bool app_host_audio_src_disconnect(uint8_t bda[6])
{
    wiced_hci_bt_audio_source_disconnect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(p_dev && (p_dev->m_audio_handle != WICED_NULL_HANDLE))
    {
        app_host_log("Sending AV disconnect");
        data.handle = p_dev->m_audio_handle;
        p_dev->m_audio_handle = WICED_NULL_HANDLE;
        p_dev->m_conn_type &= ~WICED_CONNECTION_TYPE_AUDIO;
        return wiced_hci_audio_src_disconnect(&data);
    }

    app_host_log("Device not connected as AV");
    return false;
}

bool app_host_audio_src_start(uint8_t bda[6], uint8_t sample_freq, uint8_t audio_mode)
{
    wiced_hci_bt_audio_source_start_data_t start;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(!p_dev || (p_dev && (p_dev->m_audio_handle == WICED_NULL_HANDLE)))
    {
        app_host_log("AV not connected");
        return false;
    }

    app_host_log("Sending Audio Start Handle: 0x%04x", p_dev->m_audio_handle);

    start.audio_mode = audio_mode;
    start.sample_freq = sample_freq;
    start.handle = p_dev->m_audio_handle;

    return wiced_hci_audio_src_audio_start(&start);
}


bool app_host_audio_src_stop(uint8_t bda[6])
{
    wiced_hci_bt_audio_source_stop_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(!p_dev || (p_dev && (p_dev->m_audio_handle == WICED_NULL_HANDLE)))
    {
        app_host_log("AV not connected");
        return false;
    }

    app_host_log("Sending Audio stop: 0x%04x", p_dev->m_audio_handle);

    data.handle = p_dev->m_audio_handle;

    return wiced_hci_audio_src_audio_stop(&data);
}

bool app_host_audio_src_audio_data(uint16_t handle, uint8_t *p_data, uint16_t len)
{
    wiced_hci_bt_audio_source_audio_data_t audio_data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AUDIO, handle);

    if(!p_dev || (p_dev && (p_dev->m_audio_handle == WICED_NULL_HANDLE)))
    {
        app_host_log("AV not connected");
        return false;
    }

    audio_data.p_data = p_data;
    audio_data.len = len;

    return wiced_hci_audio_src_audio_data(&audio_data);
}

bool app_host_audio_src_audio_data_format(uint16_t handle, uint8_t format)
{
    wiced_hci_bt_audio_source_audio_data_format_t payload;
    wiced_hci_bt_device_t* p_dev = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AUDIO, handle);

    if(!p_dev || (p_dev && (p_dev->m_audio_handle == WICED_NULL_HANDLE)))
    {
        app_host_log("AV not connected");
        return false;
    }

    payload.format = format;
    return wiced_hci_audio_src_audio_data_format(&payload);
}

// Handle WICED HCI events for AV
void app_host_audio_src_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    uint8_t    bda[6];
    wiced_hci_bt_device_t *device = 0;
    uint16_t  handle = 0;
    int i;
    uint8_t data1, data2;

    UNUSED(len);

    switch (opcode)
    {
        // AV Src connected
        case HCI_CONTROL_AUDIO_EVENT_CONNECTED:
        {
            for (i = 0; i < 6; i++)
                bda[5 - i] = p_data[i];

            data1 = p_data[i++];
            data2 = p_data[i++];
            handle = (uint16_t)(data1 + (data2 << 8));

            // find device in the list with received address and save the connection handle
            if ((device = app_host_find_device(bda)) == 0)
                device = app_host_add_device(bda);

            device->m_audio_handle = handle;
            device->m_conn_type |= WICED_CONNECTION_TYPE_AUDIO;

            app_host_log("Audio Connected, Handle: 0x%04x", handle);

        }
        break;

        // AV Src disconnected
        case HCI_CONTROL_AUDIO_EVENT_DISCONNECTED:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AUDIO, handle);
            if(device)
            {
                device->m_audio_handle = WICED_NULL_HANDLE;
                device->m_conn_type &= ~WICED_CONNECTION_TYPE_AUDIO;
            }

            app_host_log("Audio disconnected, Handle: 0x%04x", handle);
        }
        break;

        // Streaming started
        case HCI_CONTROL_AUDIO_EVENT_STARTED:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AUDIO, handle);
            if(device)
                device->m_device_state.audio_src_state.audio_started = true;
            app_host_log("Audio started");

        }
        break;

        // Streaming stopped
        case HCI_CONTROL_AUDIO_EVENT_STOPPED:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            app_host_log("Audio stopped");
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AUDIO, handle);
            if(device)
                device->m_device_state.audio_src_state.audio_started = false;
        }
        break;

        // Embedded app requested audio data
        case HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA:
            break;

        case HCI_CONTROL_AUDIO_EVENT_COMMAND_COMPLETE:
            break;

        case HCI_CONTROL_AUDIO_EVENT_COMMAND_STATUS:
            break;

        case HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED:
            break;

        default:
            break;
     }

}
