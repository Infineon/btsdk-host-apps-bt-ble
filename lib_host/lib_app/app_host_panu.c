/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "wiced_hci_panu.h"

bool is_panu_connected = false;

bool app_host_panu_connect(uint8_t bda[6])
{
    wiced_hci_bt_panu_connect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(p_dev && (p_dev->m_panu_handle != WICED_NULL_HANDLE))
    {
        app_host_log("PANU already connected p_dev->m_panu_handle = 0x%x", p_dev->m_panu_handle);
        is_panu_connected = true;
        return false;
    }

    app_host_log("Sending PANU Connect");

    memcpy(data.bda, bda, BDA_LEN);

    return wiced_hci_panu_connect(&data);
}

bool app_host_panu_disconnect(uint8_t bda[6])
{
    wiced_hci_bt_panu_disconnect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if (NULL == p_dev)
        return false;

    if (WICED_NULL_HANDLE == (data.handle = p_dev->m_panu_handle))
        return false;


    app_host_log("Sending PANU Disconnect Command, Handle: 0x%04x", data.handle);
    return wiced_hci_panu_disconnect(&data);
}

extern void script_handle_event(uint16_t opcode, uint32_t is_connected, uint32_t port_handle, uint8_t * p_data, uint32_t len);

void app_host_panu_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    uint8_t    bda[6];
    wiced_hci_bt_device_t *device = 0;
    uint16_t  handle = 0;
    int       i = 0;
    unsigned char * p_rx_data = NULL;
    uint32_t rx_data_len = 0;

    app_host_log("app_host_panu_event opcode = 0x%04x",opcode);

    switch (opcode)
    {
        case HCI_CONTROL_PANU_EVENT_CONNECTED :
        {
            for (i = 0; i < 6; i++)
                bda[5 - i] = p_data[i];

            handle = (uint16_t)(p_data[6] + (p_data[7] << 8));

            app_host_log("PANU connected %02x:%02x:%02x:%02x:%02x:%02x handle %04x",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], handle);

            // find device in the list with received address and save the connection handle
            if ((device = app_host_find_device(bda)) == 0)
                device = app_host_add_device(bda);

            device->m_panu_handle = handle;
            device->m_conn_type |= WICED_CONNECTION_TYPE_PANU;
            is_panu_connected = true;
        }
        break;

        case HCI_CONTROL_PANU_EVENT_SERVICE_NOT_FOUND:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            app_host_log("PANU SERVICE NOT FOUND, Handle: 0x%04x", handle);
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_PANU, handle);
            if (device)
            {
                device->m_panu_handle = WICED_NULL_HANDLE;
                device->m_conn_type &= ~WICED_CONNECTION_TYPE_PANU;
            }
            is_panu_connected = false;
        }
        break;

        case HCI_CONTROL_PANU_EVENT_CONNECTION_FAILED:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            app_host_log("PANU connect failed, Handle: 0x%04x", handle);
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_PANU, handle);
            if (device)
            {
                device->m_panu_handle = WICED_NULL_HANDLE;
                device->m_conn_type &= ~WICED_CONNECTION_TYPE_PANU;
            }
            is_panu_connected = false;
        }
        break;

        case HCI_CONTROL_PANU_EVENT_DISCONNECTED:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            app_host_log("PANU disconnected, Handle: 0x%04x", handle);
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_PANU, handle);
            if (device)
            {
                device->m_panu_handle = WICED_NULL_HANDLE;
                device->m_conn_type &= ~WICED_CONNECTION_TYPE_PANU;
            }
            is_panu_connected = false;
        }
        break;
    }

    script_handle_event(opcode, is_panu_connected, handle, p_rx_data, rx_data_len);
    app_host_handle_event(opcode, p_data, len);
}
