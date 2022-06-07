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
#include "wiced_hci_le_audio.h"
#include "wiced_bt_defs.h"


bool wiced_hci_le_audio_get_media_players(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_GET_MEDIA_PLAYERS, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_set_media_player(wiced_hci_bt_le_audio_set_media_player_t *p_data)
{
    uint8_t    cmd[50];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);
    UINT8_TO_STREAM(p_cmd, p_data->len);
    ARRAY_TO_STREAM(p_cmd, p_data->player_name, p_data->len);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_SET_MEDIA_PLAYER, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_play(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_PLAY, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_pause(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_PAUSE, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_volume_up(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_volume_down(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_mute(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_MUTE, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_unmute(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_set_volume(wiced_hci_bt_le_audio_vol_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);
    UINT8_TO_STREAM(p_cmd, p_data->vol);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_unmute_relative_vol_down(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_audio_unmute_relative_vol_up(wiced_bt_le_audio_cmd_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_call_control_generate_call(wiced_hci_bt_call_control_URI *p_data)
{
    uint8_t    cmd[50];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);
    UINT8_TO_STREAM(p_cmd, p_data->uri_len);
    ARRAY_TO_STREAM(p_cmd, p_data->call_URI, p_data->uri_len);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_CALL, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_call_control_accept_call(wiced_bt_ga_tbs_call_control_point_t *p_data)
{
    uint8_t    cmd[50];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);
    UINT8_TO_STREAM(p_cmd, p_data->call_id);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_ACCEPT_CALL, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_call_control_terminate_call(wiced_bt_ga_tbs_call_control_point_t *p_data)
{
    uint8_t    cmd[50];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->conn_id);
    UINT8_TO_STREAM(p_cmd, p_data->termination_data.call_id);
    UINT8_TO_STREAM(p_cmd, p_data->termination_data.termination_reason);

    return wiced_hci_send_command(HCI_CONTROL_LE_AUDIO_COMMAND_REJECT_CALL, cmd, (uint32_t)(p_cmd - cmd));
}