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
#include "wiced_hci_hf.h"
#include "hci_control_api.h"

bool wiced_hci_hf_connect (wiced_hci_bt_hf_connect_data_t * data)
{
    uint8_t cmd[60];
    int      commandBytes = 0;
    int i = 0;

    for (i = 0; i < 6; i++)
    {
        cmd[commandBytes++] = data->m_address[5 - i];
    }
    return wiced_hci_send_command(HCI_CONTROL_HF_COMMAND_CONNECT, cmd, 6);
}

bool wiced_hci_hf_disconnect (wiced_hci_bt_hf_disconnect_data_t * data)
{
    uint8_t cmd[60];
    uint32_t      commandBytes = 0;

    uint16_t nHandle = data->handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    return wiced_hci_send_command(HCI_CONTROL_HF_COMMAND_DISCONNECT, cmd, commandBytes);
}

bool wiced_hci_hf_open_audio (wiced_hci_bt_hf_open_audio_data_t * data)
{
    uint8_t    cmd[60];
    uint32_t     commandBytes = 0;
    cmd[commandBytes++] = data->handle & 0xff;
    cmd[commandBytes++] = (data->handle >> 8) & 0xff;

    return wiced_hci_send_command(HCI_CONTROL_HF_COMMAND_OPEN_AUDIO, cmd, commandBytes);
}

bool wiced_hci_hf_close_audio (wiced_hci_bt_hf_close_audio_data_t * data)
{
    uint8_t    cmd[60];
    uint32_t     commandBytes = 0;
    cmd[commandBytes++] = data->handle & 0xff;
    cmd[commandBytes++] = (data->handle >> 8) & 0xff;

    return wiced_hci_send_command(HCI_CONTROL_HF_COMMAND_CLOSE_AUDIO, cmd, commandBytes);
}

bool wiced_hci_hf_at_command (wiced_hci_bt_hf_at_command_data_t * data)
{
    uint8_t cmd[300]={0};
    uint32_t     commandBytes = 0;
    bool rval = false;

    cmd[commandBytes++] = data->handle & 0xff;
    cmd[commandBytes++] = (data->handle >> 8) & 0xff;
    cmd[commandBytes++] = data->num & 0xff;
    cmd[commandBytes++] = (data->num >> 8) & 0xff;

    if (data->atStr)
    {
        strncpy((char *)&cmd[commandBytes], data->atStr, sizeof(cmd) - commandBytes-1);
       rval = wiced_hci_send_command(HCI_CONTROL_HF_AT_COMMAND_BASE + data->nAtCmd, cmd, commandBytes + strlen(data->atStr));
    }
    else
    {
        rval = wiced_hci_send_command(HCI_CONTROL_HF_AT_COMMAND_BASE + data->nAtCmd, cmd, commandBytes);
    }

    return rval;
}

bool wiced_hci_hf_audio_accept_conn (wiced_hci_bt_hf_audio_accept_data_t * data)
{
    uint8_t  cmd[3];
    uint8_t  *p = &cmd[0];

    //Accept the connection request
    UINT16_TO_STREAM(p, data->sco_index);
    UINT8_TO_STREAM(p, 1);// 1 : Accept, 0:Reject

    return wiced_hci_send_command(HCI_CONTROL_HF_COMMAND_AUDIO_ACCEPT_CONN, cmd, 3);
}
