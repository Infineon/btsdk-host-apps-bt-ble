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
#include "string.h"

bool app_host_avrc_ct_connect(uint8_t bda[6])
{
    wiced_bt_avrc_ct_connect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);
    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        app_host_log("AVRC CT already connected");
        return false;
    }

    app_host_log("Sending AVRC CT Connect");

    memcpy(data.bda, bda, BDA_LEN);
    return wiced_hci_avrc_ct_connect(&data);
}


bool app_host_avrc_ct_disconnect(uint8_t bda[6])
{
    wiced_bt_avrc_ct_disconnect_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        app_host_log("Sending AVRC CT disconnect");
        data.handle = p_dev->m_avrc_handle;

        p_dev->m_avrc_handle = WICED_NULL_HANDLE;
        p_dev->m_conn_type &= ~WICED_CONNECTION_TYPE_AVRC;

        return wiced_hci_avrc_ct_disconnect(&data);
    }

    app_host_log("Device not connected as AVRC CT");
    return false;
}


bool app_host_avrc_ct_command(uint8_t bda[6], uint8_t cmd)
{
    wiced_bt_avrc_ct_cmd_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    memset(&data, 0, sizeof(wiced_bt_avrc_ct_cmd_data_t));

    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        app_host_log("Sending AVRC CT cmd %d", cmd);
        data.handle = p_dev->m_avrc_handle;
    }

    switch(cmd)
    {
    case WICED_AVRCP_CT_PLAY_CMD:
        return wiced_hci_avrc_ct_play(&data);
    case WICED_AVRCP_CT_STOP_CMD:
        return wiced_hci_avrc_ct_stop(&data);
    case WICED_AVRCP_CT_PAUSE_CMD:
        return wiced_hci_avrc_ct_pause(&data);
    case WICED_AVRCP_CT_NEXT_CMD:
        return wiced_hci_avrc_ct_next_track(&data);
    case WICED_AVRCP_CT_PREVIOUS_CMD:
        return wiced_hci_avrc_ct_previous_track(&data);
    case WICED_AVRCP_CT_VOL_UP_CMD:
        return wiced_hci_avrc_ct_volume_up(&data);
    case WICED_AVRCP_CT_VOL_DOWN_CMD:
        return wiced_hci_avrc_ct_volume_down(&data);
    case WICED_AVRCP_CT_MUTE_CMD:
        return wiced_hci_avrc_ct_mute(&data);
    case WICED_AVRCP_CT_FF_CMD_PRESS:
        return wiced_hci_avrc_ct_skip_forward_pressed(&data);
    case WICED_AVRCP_CT_FF_CMD_RELEASE:
        return wiced_hci_avrc_ct_skip_forward_released(&data);
    case WICED_AVRCP_CT_REV_CMD_PRESS:
        return wiced_hci_avrc_ct_skip_backward_pressed(&data);
    case WICED_AVRCP_CT_REV_CMD_RELEASE:
        return wiced_hci_avrc_ct_skip_backward_released(&data);
    }

    app_host_log("Invalid cmd AVRCP CT");
    return false;
}


bool app_host_avrc_ct_repeat(uint8_t bda[6], uint8_t setting)
{
    wiced_bt_avrc_ct_settings_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    memset(&data, 0, sizeof(wiced_bt_avrc_ct_settings_data_t));

    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        data.handle = p_dev->m_avrc_handle;
    }
    app_host_log("Sending AVRC CT repeat setting %d", setting);
    data.setting = setting;
    return wiced_hci_avrc_ct_repeat(&data);
}

bool app_host_avrc_ct_shuffle(uint8_t bda[6], uint8_t setting)
{
    wiced_bt_avrc_ct_settings_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    memset(&data, 0, sizeof(wiced_bt_avrc_ct_settings_data_t));

    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        data.handle = p_dev->m_avrc_handle;
    }
    app_host_log("Sending AVRC CT shuffle setting %d", setting);
    data.setting = setting;

    return wiced_hci_avrc_ct_shuffle(&data);
}

bool app_host_avrc_ct_volume_level(uint8_t bda[6], uint8_t vol_level)
{
    wiced_bt_avrc_ct_volume_level_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    memset(&data, 0, sizeof(wiced_bt_avrc_ct_volume_level_data_t));

    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        data.handle = p_dev->m_avrc_handle;
    }

    app_host_log("Sending AVRC CT vol level %d", vol_level);
    data.volume_level = vol_level;

    return wiced_hci_avrc_ct_volume_level(&data);
}

bool app_host_avrc_unit_info(uint8_t bda[BDA_LEN])
{
    wiced_bt_avrc_ct_cmd_unit_info_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    memset(&data, 0, sizeof(wiced_bt_avrc_ct_cmd_unit_info_data_t));

    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        data.handle = p_dev->m_avrc_handle;
    }
    app_host_log("Sending unit info cmd");
    return wiced_hci_avrc_unit_info(&data);
}

bool app_host_avrc_sub_unit_info(uint8_t bda[BDA_LEN])
{
    wiced_bt_avrc_ct_cmd_sub_unit_info_data_t data;
    wiced_hci_bt_device_t* p_dev = app_host_find_device(bda);

    memset(&data, 0, sizeof(wiced_bt_avrc_ct_cmd_sub_unit_info_data_t));


    if(p_dev && (p_dev->m_avrc_handle != WICED_NULL_HANDLE))
    {
        data.handle = p_dev->m_avrc_handle;
    }
    app_host_log("Sending sub-unit info cmd");
    return wiced_hci_avrc_sub_unit_info(&data);
}


void app_host_avrc_ct_event(uint16_t opcode, uint8_t * p_data, uint16_t len)
{
    uint8_t    bda[6];
    wiced_hci_bt_device_t *device = 0;
    uint16_t  handle = 0;
    int i = 0, j = 0;

    switch (opcode)
    {
        // AVRC connected with peer
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED:
        {
           for (i = 0; i < 6; i++)
                bda[5 - i] = p_data[i];

            i++; // skip status byte

            handle = (uint16_t)(p_data[i] | (p_data[i+1] << 8));

            // find device in the list with received address and save the connection handle
            if ((device = app_host_find_device(bda)) == 0)
                device = app_host_add_device(bda);

            device->m_avrc_handle = handle;
            device->m_conn_type |= WICED_CONNECTION_TYPE_AVRC;

            app_host_log("HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED");

        }
        break;

        // AVRC diconnected from peer
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            app_host_log("HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED, %d", handle);

            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AVRC, handle);
            if (device && (device->m_avrc_handle == handle))
            {
                device->m_avrc_handle = WICED_NULL_HANDLE;
                device->m_conn_type &= ~WICED_CONNECTION_TYPE_AVRC;

            }
        }
        break;


        // Peer changed player
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE:
        {
            handle =(uint16_t)(p_data[0] | (p_data[1] << 8));

            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AVRC, handle);
            if(device)
            {
                size_t cnt = (len-2) < WICED_AVRC_MAX_MEDIA_LEN-1 ? (len-2) : WICED_AVRC_MAX_MEDIA_LEN-1;
                memcpy(device->m_device_state.avrc_ct_state.m_player, &p_data[2], cnt);
                app_host_log("HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE %s", device->m_device_state.avrc_ct_state.m_player);
            }
        }
        break;

        // Peer indicated its available settings
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_AVAILABLE:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));

            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AVRC, handle);
            if(device)
            {
                for (i=2; i<=WICED_MAX_RC_APP_SETTING_VALUE; i++)
                {
                    if(i == WICED_AVRC_PLAYER_SETTING_REPEAT)
                    {
                        device->m_device_state.avrc_ct_state.m_avialable_repeat_settings
                                = p_data[i];
                        app_host_log("HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_AVAILABLE: repeat %02x ", p_data[i]);

                        if(p_data[i] & WICED_AVRC_PLAYER_VAL_OFF)
                        {

                        }
                        if(p_data[i] & WICED_AVRC_PLAYER_VAL_SINGLE_REPEAT)
                        {

                        }
                        if(p_data[i] & WICED_AVRC_PLAYER_VAL_ALL_REPEAT)
                        {

                        }
                    }
                    if(i == WICED_AVRC_PLAYER_SETTING_SHUFFLE)
                    {
                        device->m_device_state.avrc_ct_state.m_avialable_shuffle_settings
                                = p_data[i];
                        app_host_log("HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_AVAILABLE: shuffle %02x ", p_data[i]);
                    }
                }
            }
        }
        break;

        // Peer changed settings
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));

            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AVRC, handle);
            if(device)
            {
                for (i = 0, j = 3; i < p_data[2]; i++)
                {
                    app_host_log("attr: %02x setting: %02x ", p_data[j], p_data[j + 1]);

                    switch (p_data[j++])
                    {
                    case WICED_AVRC_PLAYER_SETTING_REPEAT: // Repeat Mode
                        device->m_device_state.avrc_ct_state.m_repeat_settings_value = p_data[j++];
                        app_host_log("Repeat value %d", device->m_device_state.avrc_ct_state.m_repeat_settings_value);

                     break;

                    case WICED_AVRC_PLAYER_SETTING_SHUFFLE: // Shuffle Mode
                        device->m_device_state.avrc_ct_state.m_shuffle_settings_value = p_data[j++];
                        app_host_log("Shuffle value %d", device->m_device_state.avrc_ct_state.m_shuffle_settings_value);

                        break;
                    }
                }

            }
        }
        break;

         // Peer changed play status
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS:
        {
             handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
             device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AVRC, handle);
             if(device)
             {
                device->m_device_state.avrc_ct_state.m_play_status = p_data[2];
                app_host_log("Play status %d", device->m_device_state.avrc_ct_state.m_play_status);
             }

        }
        break;

        // Peer indicated current playing track info
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO:
        {
            handle = (uint16_t)(p_data[0] | (p_data[1] << 8));
            device = app_host_find_device_by_connection(WICED_CONNECTION_TYPE_AVRC, handle);
            if(device)
            {
                uint8_t media_type = p_data[3];
                uint16_t str_len = (uint16_t)(p_data[4] + (p_data[5] << 8));
                uint8_t string_name[WICED_AVRC_MAX_MEDIA_LEN];

                if(str_len >= WICED_AVRC_MAX_MEDIA_LEN)
                    str_len = WICED_AVRC_MAX_MEDIA_LEN -1;

                memset(string_name, 0, WICED_AVRC_MAX_MEDIA_LEN);

                if(str_len)
                {
                    memcpy(string_name, &p_data[6],  str_len);

                    memcpy(&device->m_device_state.avrc_ct_state.m_media[media_type-1][0], &p_data[6], str_len);
                }

                app_host_log("Track attr %d, name %s", media_type, (char *)string_name);
            }
        }
        break;

    }

    app_host_handle_event(opcode, p_data, len);

}
