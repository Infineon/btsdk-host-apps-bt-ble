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

#include <string.h>
#include <memory.h>
#include "wiced_hci_dm.h"
#include "hci_control_api.h"

// set local device name
bool wiced_hci_dm_setdevname(wiced_hci_dm_setdevname_data_t * dev_name)
{
    uint8_t   cmd[60];
    int    commandBytes = 0;

    commandBytes = strlen(dev_name->device_name);
    memcpy(cmd, dev_name->device_name, commandBytes);

    return wiced_hci_send_command(HCI_CONTROL_COMMAND_SET_LOCAL_NAME, cmd, commandBytes);
}

bool wiced_hci_dm_setdevaddr(wiced_hci_dm_setdevaddr_data_t *device_addr)
{
    uint8_t   cmd[16];
    uint8_t *p = cmd;
    int     commandBytes = BD_ADDR_LEN;

    BDADDR_TO_STREAM(p, device_addr->device_addr);

    return wiced_hci_send_command(HCI_CONTROL_COMMAND_SET_LOCAL_BDA, cmd, commandBytes);
}

bool wiced_hic_dm_set_pairing_mode(wiced_hci_dm_set_pairing_mode_data_t * data)
{
    uint8_t pairing_mode = data->mode;
    return wiced_hci_send_command(HCI_CONTROL_COMMAND_SET_PAIRING_MODE, &pairing_mode, 1);
}

bool wiced_hci_dm_set_vis(wiced_hci_dm_set_vis_data_t* data)
{
    uint8_t   cmd[60];
    int    commandBytes = 0;

    cmd[commandBytes++] = data->discoverable ? 1:0; //discoverable
    cmd[commandBytes++] = data->connectable ? 1:0; ; //CONNECTABLE

    return wiced_hci_send_command(HCI_CONTROL_COMMAND_SET_VISIBILITY, cmd, commandBytes);
}

bool wiced_hci_dm_reset()
{
    return wiced_hci_send_command(HCI_CONTROL_COMMAND_RESET, 0, 0);
}

bool wiced_hci_dm_delete_nvram_data(wiced_hci_dm_delete_nvram_data_t *data)
{
    uint8_t cmd[10];
    cmd[0] = (uint8_t)data->nvram_id;
    cmd[1] = (uint8_t)(data->nvram_id >> 8);
    return wiced_hci_send_command(HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA, cmd, 2);
}

bool wiced_hci_dm_inquiry(wiced_hci_dm_inquiry_data_t * data)
{
    uint8_t command[] = { data->start_stop ? 1 : 0 };
    return wiced_hci_send_command(HCI_CONTROL_COMMAND_INQUIRY, command, 1);
}

bool wiced_hci_dm_le_scan(wiced_hci_dm_le_set_scan_data_t * data)
{
    uint8_t  command[] = { data->enable ? 1 : 0 };    // scan command, len 1, enable = 1, disable = 0
    return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_SCAN, command, 1);
}

bool wiced_hci_dm_push_pairing_host_info(wiced_hic_dm_push_host_info_data_t *data)
{
    return wiced_hci_send_command(HCI_CONTROL_HIDD_COMMAND_PUSH_PAIRING_HOST_INFO, data->nvram_data, data->len);
}

bool wiced_hci_dm_push_nvram_data(wiced_hci_dm_push_nvram_data_t *data)
{
    uint8_t cmd[1000];
    cmd[0] = (uint8_t)data->nvram_id;
    cmd[1] = (uint8_t)(data->nvram_id >> 8);
    memcpy(&cmd[2], data->nvram_data, data->len);
    return wiced_hci_send_command(HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA, cmd, 2 + data->len);
}


bool wiced_hci_dm_hidh_add(wiced_hci_dm_bda_data_t * data)
{
    uint8_t cmd_param[6];
    cmd_param[0] = data->bda[5];
    cmd_param[1] = data->bda[4];
    cmd_param[2] = data->bda[3];
    cmd_param[3] = data->bda[2];
    cmd_param[4] = data->bda[1];
    cmd_param[5] = data->bda[0];

    // Add this device (to allow it to reconnect)
    return wiced_hci_send_command(HCI_CONTROL_HIDH_COMMAND_ADD, cmd_param, BD_ADDR_LEN);
}

bool wiced_hci_dm_add_battery_client(wiced_hci_dm_bda_data_t * data)
{
    uint8_t cmd_param[6];
    cmd_param[0] = data->bda[5];
    cmd_param[1] = data->bda[4];
    cmd_param[2] = data->bda[3];
    cmd_param[3] = data->bda[2];
    cmd_param[4] = data->bda[1];
    cmd_param[5] = data->bda[0];

    return wiced_hci_send_command(HCI_CONTROL_BATT_CLIENT_COMMAND_ADD, cmd_param, BD_ADDR_LEN);
}

bool wiced_hci_dm_add_findme_locator(wiced_hci_dm_bda_data_t * data)
{
    uint8_t cmd_param[6];
    cmd_param[0] = data->bda[5];
    cmd_param[1] = data->bda[4];
    cmd_param[2] = data->bda[3];
    cmd_param[3] = data->bda[2];
    cmd_param[4] = data->bda[1];
    cmd_param[5] = data->bda[0];

    return wiced_hci_send_command(HCI_CONTROL_FINDME_LOCATOR_COMMAND_ADD, cmd_param, BD_ADDR_LEN);
}

bool wiced_hci_dm_get_version_info()
{
    return wiced_hci_send_command(HCI_CONTROL_MISC_COMMAND_GET_VERSION, 0, 0);
}

bool wiced_hci_dm_set_app_traces(wiced_hci_dm_set_app_traces_data_t * data)
{
    uint8_t cmd[2];
    cmd[0] = data->enable ? 1 : 0; // Enable/Disable Bluetooth HCI trace
    cmd[1] = data->route;         //  Routing option

    // send command to configure traces
    return wiced_hci_send_command(HCI_CONTROL_COMMAND_TRACE_ENABLE, cmd, 2);
}

bool wiced_hci_dm_user_confirm(wiced_hci_dm_user_confirm_data_t * data)
{
    uint8_t command[20];
    int i = 0;

    for (i = 0; i < 6; i++)
        command[5 - i] = data->bda[i];

    command[6] = data->confirm ? 1 : 0; // 1 - accept, 0 - do not accept

    return wiced_hci_send_command(HCI_CONTROL_COMMAND_USER_CONFIRMATION, command, 7);
}

bool wiced_hci_dm_unbond_device(wiced_hci_dm_bda_data_t * data)
{
    uint8_t cmd_param[6];

    cmd_param[0] = data->bda[5];
    cmd_param[1] = data->bda[4];
    cmd_param[2] = data->bda[3];
    cmd_param[3] = data->bda[2];
    cmd_param[4] = data->bda[1];
    cmd_param[5] = data->bda[0];

    return wiced_hci_send_command(HCI_CONTROL_COMMAND_UNBOND_DEVICE, cmd_param, BD_ADDR_LEN);
}
