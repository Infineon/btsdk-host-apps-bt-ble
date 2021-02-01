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
#include <string.h>

bool app_host_hidh_connect(uint8_t bda[6])
{
    wiced_hci_bt_hidh_connect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if(p_dev && (p_dev->m_hidh_handle != WICED_NULL_HANDLE))
    {
        app_host_log("HID HOST already connected");
        return false;
    }
    app_host_log("Sending HID HOST Connect BdAddr:%02x:%02x:%02x:%02x:%02x:%02x",
        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    memcpy(data.bda, bda, BDA_LEN);
    return wiced_hci_hidh_connect(&data);
}

bool app_host_hidh_set_report(uint16_t handle, uint8_t channel, uint8_t report_type, uint8_t report_id, char * string, uint32_t length)
{
    wiced_hci_bt_hidh_set_report_data_t data;

    data.handle = handle;
    data.channle = channel;
    data.report_type = report_type;
    data.report_id = report_id;
    data.string = string;
    data.length = length;

    app_host_log("Sending HID SetReport Handle:%d Channel:%d Type:%d Id:%d len:%d",
        handle, channel, report_type, report_id, length);

    return wiced_hci_hidh_set_report(&data);
}

bool app_host_hidh_get_report(uint16_t nHandle, uint8_t report_type, uint8_t report_id)
{
    wiced_hci_bt_hidh_get_report_data_t data;

    data.handle = nHandle;
    data.report_type = report_type;
    data.report_id = report_id;

    app_host_log("Sending HID GetReport Handle:%d Type:%d Id:%d",
        nHandle, report_type, report_id);

    return wiced_hci_hidh_get_report(&data);
}

bool app_host_hidh_disconnect(uint8_t bda[6])
{
    wiced_hci_bt_hidh_disconnect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if (!p_dev || (p_dev->m_hidh_handle == WICED_NULL_HANDLE))
    {
        app_host_log("app_host_hidh_disconnect() device not connected");
        return false;
    }

    data.handle = p_dev->m_hidh_handle;
    return wiced_hci_hidh_disconnect(&data);
}

bool app_host_hidh_virtual_unplug(uint8_t bda[6])
{
    wiced_hci_bt_hidh_unplag_data_t data;
    memcpy(data.bda, bda, BDA_LEN);
    return wiced_hci_hidh_virtual_unplug(&data);
}

bool app_host_hidh_get_desc(uint16_t nHandle)
{
    wiced_hci_bt_hidh_get_desc_data_t data;
    data.handle = nHandle;
    return wiced_hci_hidh_get_desc(&data);
}

bool app_host_hidh_set_proto(uint16_t nHandle, uint8_t protocol)
{
    wiced_hci_bt_hidh_set_proto_data_t data;
    data.nHandle = nHandle;
    data.protocol = protocol;
    return wiced_hci_hidh_set_proto(&data);
}

bool app_host_hidh_set_wakeup_pattern(uint8_t bda[6], uint8_t report_id, uint8_t *report_pattern, uint8_t report_len)
{
    wiced_hci_bt_hidh_pattern_data_t data;
    data.report_id = report_id;
    data.report_len = report_len;
    memset(&(data.report_pattern[0]), 0, sizeof(data.report_pattern));
    memcpy(&(data.report_pattern[0]), report_pattern, report_len);
    memcpy(data.bda, bda, BDA_LEN);
    return wiced_hci_hidh_set_wakeup_pattern(&data);
}
bool app_host_hidh_set_wakeup_control(uint8_t wakeup_gpio, uint8_t wakeup_polarity, uint8_t wakeup_state)
{
    wiced_hci_bt_hidh_wakeup_data_t data;
    data.wakeup_gpio = wakeup_gpio;
    data.wakeup_polarity = wakeup_polarity;
    data.wakeup_state = wakeup_state;

    return wiced_hci_hidh_set_wakeup_control(&data);
}

void app_host_hidh_event(uint16_t opcode, uint8_t *p_data, uint32_t len)
{
    uint8_t    bda[6];
    wiced_hci_bt_device_t *device = 0;
    uint16_t  handle = 0;
    int       i = 0;

    switch (opcode)
    {
        // HIDH connected with peer
        case HCI_CONTROL_HIDH_EVENT_CONNECTED :
        {
            for (i = 0; i < 6; i++)
                bda[5 - i] = p_data[i + 1];

            handle = (uint16_t)(p_data[7] + (p_data[8] << 8));
            device = app_host_find_device(bda);

            // if connected succussfully
            if (p_data[0]==0)
            {
                // find device in the list with received address and save the connection handle
                if (device == NULL)
                    device = app_host_add_device(bda);

                device->m_hidh_handle = handle;
                device->m_conn_type |= WICED_CONNECTION_TYPE_HIDH;

                wiced_hci_bt_hidh_connected_data_t data;
                memcpy(&data.bda, bda, BDA_LEN);
                wiced_hci_hidh_connected(&data);
            }
            else if (device)
            {
                device->m_hidh_handle = WICED_NULL_HANDLE;
            }
        }
        break;

    case HCI_CONTROL_HIDH_EVENT_DISCONNECTED:
        handle = (uint16_t)(p_data[0] | (p_data[1] << 8));

        if ((device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_HIDH, handle)))
        {
            device->m_hidh_handle = WICED_NULL_HANDLE;
            device->m_conn_type &= ~WICED_CONNECTION_TYPE_HIDH;
        }
        break;
    }

    app_host_handle_event(opcode, p_data, len);
}
