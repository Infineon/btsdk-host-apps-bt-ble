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

#include "wiced_hci.h"
#include "hci_control_api.h"
#include "wiced_hci_ag.h"
#include "string.h"

bool wiced_hci_ag_connect(wiced_hci_bt_ag_connect_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    BDADDR_TO_STREAM(p_cmd, p_data->bda);

    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_CONNECT, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_ag_disconnect(wiced_hci_bt_ag_disconnect_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->handle);

    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_DISCONNECT, cmd, (uint32_t)(p_cmd - cmd));

}

bool wiced_hci_ag_audio_open(wiced_hci_bt_ag_audio_open_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->handle);

    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_OPEN_AUDIO, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_ag_audio_close(wiced_hci_bt_ag_audio_close_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->handle);

    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_CLOSE_AUDIO, cmd, (uint32_t)(p_cmd - cmd));

}

bool wiced_hci_ag_send_clcc_response(wiced_hci_bt_ag_clcc_res_t *p_data)
{
    uint8_t cmd[100];
    uint8_t index=0;
    int call_counter=0;
    uint8_t     *p_cmd = cmd;
    char at_cmd[10]="+CLCC: ";
    char at_ok[5]="OK";

    UINT16_TO_STREAM(p_cmd, p_data->handle);
    index += 2;

    for(call_counter=0; call_counter < p_data->num_calls; call_counter++)
    {
        if (p_data->call_list_status[call_counter] == 0)
            break;
        // skip handle as its already set
        index = 2;
        p_cmd = cmd;
        p_cmd += 2;

        strncpy((char *)p_cmd, at_cmd, strlen(at_cmd));
        index += strlen(at_cmd);

        cmd[index++]= 9; // size for call list
        cmd[index++] = '1'+call_counter; // call id
        cmd[index++] = ',';
        cmd[index++] = '1'; // Direction incoming
        cmd[index++] = ',';
        cmd[index++] = '0'+p_data->call_list_status[call_counter]-1; // Status
        cmd[index++] = ',';
        cmd[index++] = '0'; // mode Voice
        cmd[index++] = ',';
        cmd[index++] = '0'; // multi-party

        if ( wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, index) == false )
            return false;
    }

    // skip handle as its already set
    index = 2;
    p_cmd = cmd;
    p_cmd += 2;
    strcpy((char *)p_cmd, at_ok);
    index += strlen(at_ok);

    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, index);
}

bool wiced_hci_ag_send_cind(wiced_hci_bt_ag_cind_t *p_data)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;

    strncpy((char *)p_cmd, p_data->cind_str, strlen(p_data->cind_str));
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_SET_CIND, cmd, strlen(p_data->cind_str));
}

bool wiced_hci_ag_send_ring_cmd(uint16_t handle)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[10] = "RING";

    UINT16_TO_STREAM(p_cmd, handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, 2+strlen(at_cmd));
}

bool wiced_hci_ag_send_clip_cmd(uint16_t handle)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[30] = "+CLIP: \"1234567\",129";

    UINT16_TO_STREAM(p_cmd, handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, 2+strlen(at_cmd));
}

bool wiced_hci_ag_send_ccwa_cmd(uint16_t handle)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[10] = "+CCWA: ";
    char at_arg_num[30] = "7654321";
    char at_arg_num_type[30] = "129";
    UINT16_TO_STREAM(p_cmd, handle);

    // +CCWA:
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    p_cmd += strlen(at_cmd);

    // "phonenumer"
    *p_cmd = '"';
    p_cmd += 1;
    strncpy((char *)p_cmd, at_arg_num,strlen(at_arg_num));
    p_cmd += strlen(at_arg_num);
    *p_cmd = '"';
    p_cmd += 1;

    *p_cmd = ',';
    p_cmd += 1;
    // type
    strncpy((char *)p_cmd, at_arg_num_type,strlen(at_arg_num_type));
    p_cmd += strlen(at_arg_num_type);
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_ag_send_ciev(wiced_hci_bt_ag_ciev_t *p_data)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[10] = "+CIEV: ";

    UINT16_TO_STREAM(p_cmd, p_data->handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    p_cmd += strlen(at_cmd);
    strncpy((char *)p_cmd, p_data->ciev_str, strlen(p_data->ciev_str));
    p_cmd += strlen(p_data->ciev_str);
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, (uint32_t)(p_cmd-cmd));
}

bool wiced_hci_ag_send_ok_cmd(uint16_t handle)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[10] = "OK";

    UINT16_TO_STREAM(p_cmd, handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, 2+strlen(at_cmd));
}

bool wiced_hci_ag_send_error_cmd(uint16_t handle)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[30] = "ERROR";

    UINT16_TO_STREAM(p_cmd, handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, 2+strlen(at_cmd));
}

bool wiced_hci_ag_send_spk_vol_cmd(uint16_t handle, int volume)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[30] = "+VGS: ";

    UINT16_TO_STREAM(p_cmd, handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    p_cmd += strlen(at_cmd);
    if (volume > 9)
    {
        volume -= 10;
        *p_cmd = '1';
        p_cmd++;
    }
    *p_cmd = volume + '0';
    p_cmd++;
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, (uint32_t)(p_cmd-cmd));
}

bool wiced_hci_ag_send_mic_vol_cmd(uint16_t handle, int volume)
{
    uint8_t cmd[100];
    uint8_t     *p_cmd = cmd;
    char at_cmd[30] = "+VGM: ";

    UINT16_TO_STREAM(p_cmd, handle);
    strncpy((char *)p_cmd, at_cmd,strlen(at_cmd));
    p_cmd += strlen(at_cmd);
    if (volume > 9)
    {
        volume -= 10;
        *p_cmd = '1';
        p_cmd++;
    }
    *p_cmd = volume + '0';
    p_cmd++;
    return wiced_hci_send_command(HCI_CONTROL_AG_COMMAND_STR, cmd, (uint32_t)(p_cmd-cmd));
}
