/*
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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


#include "app_host_gatt.h"
#include "app_host_le_audio.h"
#include "wiced_hci_le_audio.h"
#include "app_host.h"
#include "string.h"

bool app_host_le_audio_get_media_players(uint16_t conn_id)
{
    wiced_bt_le_audio_cmd_data_t data;
    app_host_log("Sending LE Audio get media players");
    data.conn_id = conn_id;
    return wiced_hci_le_audio_get_media_players(&data);
}

bool app_host_le_audio_set_media_player(uint16_t conn_id, uint8_t len, uint8_t* p_player_name)
{
    wiced_hci_bt_le_audio_set_media_player_t data;
    app_host_log("Sending LE Audio set media player");
    data.len = len;
    data.conn_id = conn_id;
    memcpy(data.player_name, p_player_name, len);
    return wiced_hci_le_audio_set_media_player(&data);
}

bool app_host_le_audio_command(uint16_t conn_id, uint8_t cmd, void *p_codec_config)
{
    wiced_bt_le_audio_cmd_data_t data;
    data.conn_id = conn_id;

    switch(cmd)
    {
    case WICED_LE_AUDIO_PLAY_CMD:
        return wiced_hci_le_audio_play(&data, p_codec_config);
    case WICED_LE_AUDIO_STOP_CMD:
        return true;
        //return wiced_hci_le_audio_stop(&data);
    case WICED_LE_AUDIO_PAUSE_CMD:
        return wiced_hci_le_audio_pause(&data);
    case WICED_LE_AUDIO_VOL_UP_CMD:
        return wiced_hci_le_audio_volume_up(&data);
    case WICED_LE_AUDIO_VOL_DOWN_CMD:
        return wiced_hci_le_audio_volume_down(&data);
    case WICED_LE_AUDIO_MUTE_CMD:
        return wiced_hci_le_audio_mute(&data);
    case WICED_LE_AUDIO_UNMUTE_CMD:
        return wiced_hci_le_audio_unmute(&data);
    case WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_UP_CMD:
        return wiced_hci_le_audio_unmute_relative_vol_up(&data);
    case WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_DOWN_CMD:
        return wiced_hci_le_audio_unmute_relative_vol_down(&data);
    case WICED_LE_AUDIO_BROADCAST_SINK_PLAY_PAUSE_CMD:
        return wiced_hci_le_audio_broadcast_sink_play_pause_broadcast_code (data.conn_id);
    }

    return false;
}

void app_host_le_audio_set_abs_volume(uint16_t conn_id, uint8_t vol)
{
    wiced_hci_bt_le_audio_vol_t data;
    data.conn_id = conn_id;
    data.vol = vol;
    wiced_hci_le_audio_set_volume(&data);
}

bool app_host_le_audio_generate_call(uint16_t conn_id, uint8_t uri_len, uint8_t* p_call_URI, uint8_t f_len, uint8_t* p_friendly_name)
{
    wiced_hci_bt_call_data_t data;
    app_host_log("Generating call from the server...");
    data.conn_id = conn_id;

    if(uri_len >= MAX_CALL_URI)
    {
       uri_len = (MAX_CALL_URI-1);
    }
    data.uri_len = uri_len;
    memcpy(data.call_URI, p_call_URI, uri_len);

    if(f_len >= MAX_CALL_FRIENDLY_NAME)
    {
       f_len = (MAX_CALL_FRIENDLY_NAME-1);
    }
    else if(f_len == 0)
    {
       f_len = strlen("Unknown");
       memcpy(p_friendly_name,"Unknown", f_len);
       app_host_log("Call friendly name NOT provided...");
    }

    data.fri_name_len = f_len;
    memcpy(data.friendly_name, p_friendly_name, f_len);

    return wiced_hci_call_control_generate_call(&data);
}

bool app_host_le_audio_handle_call_action(uint16_t conn_id, uint8_t call_id,wiced_bt_ga_tbs_call_action_t action)
{
    wiced_bt_ga_tbs_call_control_point_t data;
    uint16_t opcode = WICED_BT_GA_CCP_UNKNOWN_OPCODE;

    switch (action)
    {
    case WICED_BT_GA_CCP_ACTION_ACCEPT_CALL:
        app_host_log("Accepting the call... conn id %d call_id %d ", conn_id, call_id);
        opcode = HCI_CONTROL_LE_AUDIO_COMMAND_ACCEPT_CALL;
        break;
    case WICED_BT_GA_CCP_ACTION_HOLD_CALL:
        app_host_log("call on hold... conn id %d call_id %d ", conn_id, call_id);
        opcode = HCI_CONTROL_LE_AUDIO_COMMAND_HOLD_CALL;
        break;
    case WICED_BT_GA_CCP_ACTION_RETRIEVE_CALL:
        app_host_log("call retrieved... conn id %d call_id %d ", conn_id, call_id);
        opcode = HCI_CONTROL_LE_AUDIO_COMMAND_RETRIEVE_CALL;
        break;
    case WICED_BT_GA_CCP_ACTION_JOIN_CALL:
        app_host_log("call join... conn id %d call_id %d ", conn_id, call_id);
        opcode = HCI_CONTROL_LE_AUDIO_COMMAND_JOIN_CALL;
        break;
    default:
        app_host_log("Unknown opcode %d",opcode);
        break;
    }
    data.conn_id = conn_id;
    data.call_id = call_id;
    data.opcode = opcode;

    return wiced_hci_call_control_handle_call_action(&data);
}

bool app_host_le_audio_terminate_call(uint16_t conn_id, uint8_t call_id, bool is_reject)
{
    wiced_bt_ga_tbs_call_control_point_t data;
    app_host_log("terminating the call...");
    data.conn_id = conn_id;
    data.termination_data.call_id = call_id;
    if (is_reject)
        data.termination_data.termination_reason = WICED_BT_GA_TBS_REMOTE_CALL_END;
    else
        data.termination_data.termination_reason = WICED_BT_GA_TBS_SERVER_CALL_END;

    return wiced_hci_call_control_terminate_call(&data);
}

bool app_host_le_audio_broadcast_source_start_streaming(uint8_t start,
                                                        uint32_t codec_config,
                                                        uint8_t bis_count,
                                                        uint32_t num_channels,
                                                        uint8_t encryption,
                                                        uint32_t broadcast_id,
                                                        uint8_t *broadcast_code)
{
    wiced_bt_ga_broadcast_start_stop_streaming_t data = {0};

    app_host_log("Terminating the call...");

    data.start = start;
    data.codec_config = codec_config;
    data.channel_counts = num_channels;
    data.bis_count = bis_count;
    data.enable_encryption = encryption;
	data.broadcast_id =broadcast_id;

    if(broadcast_code)
    {
        memcpy(data.broadcast_code, broadcast_code, 16);
    }

    return wiced_hci_broadcast_source_start_streaming(&data);
}

bool app_host_le_audio_broadcast_sink_find_sources(uint8_t start)
{
    return wiced_hci_broadcast_sink_find_source(start);
}

bool app_host_le_audio_broadcast_assistant_scan_source(uint8_t start)
{
    return wiced_hci_broadcast_assistant_scan_source(start);
}

bool app_host_le_audio_broadcast_sink_sync_to_stream(uint8_t listen, uint8_t *broadcast_code, uint32_t broadcast_id)
{
    return wiced_hci_broadcast_sink_sync_to_stream(listen, broadcast_code, broadcast_id);
}

bool app_host_le_audio_broadcast_assistant_select_source(uint8_t listen, uint16_t conn_id, uint8_t *broadcast_code, uint32_t broadcast_id, uint8_t use_past)
{
    app_host_log("ADD/Remove source to delegator");
    return wiced_hci_broadcast_assistant_select_source(listen, conn_id, broadcast_code, broadcast_id, use_past);
}
