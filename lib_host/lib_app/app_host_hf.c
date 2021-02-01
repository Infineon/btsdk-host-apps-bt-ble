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


#include "app_host_hf.h"
#include "app_host.h"
#include <string.h>

bool app_host_hf_connect (uint8_t bda[BDA_LEN])
{
	wiced_hci_bt_hf_connect_data_t data;
    memcpy(data.m_address, bda, BDA_LEN);

    app_host_log("Sending HFP Connect Command, BDA: %02x:%02x:%02x:%02x:%02x:%02x",
        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

	return wiced_hci_hf_connect (&data);
}

bool app_host_hf_disconnect (uint8_t bda[BDA_LEN])
{
    wiced_hci_bt_hf_disconnect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if (NULL == p_dev)
        return false;

    if (WICED_NULL_HANDLE == (data.handle = p_dev->m_hf_handle))
        return false;

    p_dev->m_hf_handle = WICED_NULL_HANDLE;
    p_dev->m_conn_type &= ~WICED_CONNECTION_TYPE_HF;
    return wiced_hci_hf_disconnect(&data);
}

bool app_host_hf_open_audio (uint8_t bda[6])
{
    wiced_hci_bt_hf_open_audio_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if (NULL == p_dev)
        return false;

    if (WICED_NULL_HANDLE == (data.handle = p_dev->m_hf_handle))
        return false;

    return wiced_hci_hf_open_audio (&data);
}

bool app_host_hf_close_audio (uint8_t bda[6])
{
    wiced_hci_bt_hf_close_audio_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if (NULL == p_dev)
        return false;

    if (WICED_NULL_HANDLE == (data.handle = p_dev->m_hf_handle))
        return false;

    return wiced_hci_hf_close_audio (&data);
}

bool app_host_hf_at_command (uint8_t bda[BDA_LEN], int atCmd, int num, char * atStr)
{
    wiced_hci_bt_hf_at_command_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if (NULL == p_dev)
        return false;

    memset(&data, 0, sizeof(data));

    if (WICED_NULL_HANDLE == (data.handle = p_dev->m_hf_handle))
        return false;
    data.nAtCmd = atCmd;
    data.num = num;
    data.atStr = atStr;
    return wiced_hci_hf_at_command (&data);
}

bool app_host_hf_hangup(uint8_t bda[6])
{
    return app_host_hf_at_command(bda,HCI_CONTROL_HF_AT_COMMAND_CHUP,0,NULL);
}

bool app_host_hf_button_press (uint8_t bda[6])
{
    return app_host_hf_at_command(bda,HCI_CONTROL_HF_COMMAND_BUTTON_PRESS,0,NULL);
}

bool app_host_hf_long_button_press (uint8_t bda[6])
{
    return app_host_hf_at_command(bda,HCI_CONTROL_HF_COMMAND_LONG_BUTTON_PRESS,0,NULL);
}

bool app_host_hf_audio_accept_conn(uint16_t sco_index)
{
    wiced_hci_bt_hf_audio_accept_data_t data;
    data.sco_index = sco_index;

    return wiced_hci_hf_audio_accept_conn (&data);
}

void app_host_hf_event(uint32_t opcode, uint8_t *p_data, uint32_t len)
{
    uint8_t    bda[6];
    wiced_hci_bt_device_t *device = 0;
    uint16_t  handle = 0;
    int i = 0;
    UNUSED(len); // silence the warning

    switch (opcode)
    {
        case HCI_CONTROL_HF_EVENT_OPEN:
        {
            handle = p_data[0] | (p_data[1] << 8);

            if (p_data[8] == HCI_CONTROL_HF_STATUS_SUCCESS)
            {
                for (i = 0; i < 6; i++)
                    bda[5 - i] = p_data[2 + i];

                device = app_host_add_device(bda);
                if (device)
                {
                    device->m_hf_handle = handle;
                    device->m_conn_type |= WICED_CONNECTION_TYPE_HF;
                }
            }
        }
        break;

        case HCI_CONTROL_HF_EVENT_CLOSE:
        {
            handle = p_data[0] | (p_data[1] << 8);

            device = app_host_find_device_by_handle(handle);
            if (device)
            {
                if (device->m_hf_handle == handle)
                {
                    device->m_hf_handle = WICED_NULL_HANDLE;
                    device->m_conn_type &= ~WICED_CONNECTION_TYPE_HF;
                }
            }
        }
        break;

    case HCI_CONTROL_HF_EVENT_CONNECTED:
//        handle   = p_data[0] | (p_data[1] << 8);
//        features = p_data[2] | (p_data[3] << 8);
        break;
    case HCI_CONTROL_HF_EVENT_AUDIO_OPEN:
//        handle   = p_data[0] | (p_data[1] << 8);
        break;
    case HCI_CONTROL_HF_EVENT_AUDIO_CLOSE:
//        handle   = p_data[0] | (p_data[1] << 8);
        break;
    }
}
