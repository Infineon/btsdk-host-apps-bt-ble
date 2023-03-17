
/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "wiced_types.h"
#include "wiced_hci_le_audio.h"
#ifndef APP_HOST_LE_AUDIO_H
#define APP_HOST_LE_AUDIO_H

#define WICED_LE_AUDIO_PLAY_CMD        1
#define WICED_LE_AUDIO_STOP_CMD        2
#define WICED_LE_AUDIO_PAUSE_CMD       3
#define WICED_LE_AUDIO_VOL_UP_CMD      4
#define WICED_LE_AUDIO_VOL_DOWN_CMD    5
#define WICED_LE_AUDIO_MUTE_CMD        6
#define WICED_LE_AUDIO_UNMUTE_CMD      7
#define WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_UP_CMD      8
#define WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_DOWN_CMD    9
#define WICED_LE_AUDIO_BROADCAST_SINK_PLAY_PAUSE_CMD         10

#define BD_ADDR_LEN     6       /**< Device Bluetooth Address Length */

bool app_host_le_audio_command(uint16_t conn_id, uint8_t cmd, void *p_codec_config);
void app_host_le_audio_set_abs_volume(uint16_t conn_id, uint8_t vol);
void app_host_le_audio_event(uint16_t opcode, uint8_t * p_data, uint16_t len);
bool app_host_le_audio_get_media_players(uint16_t conn_id);
bool app_host_le_audio_set_media_player(uint16_t conn_id, uint8_t len, uint8_t* p_player_name);

bool app_host_le_audio_generate_call(uint16_t conn_id, uint8_t uri_len, uint8_t* p_call_URI,uint8_t f_len, uint8_t* p_friendly_name);
bool app_host_le_audio_handle_call_action(uint16_t conn_id, uint8_t call_id, wiced_bt_ga_tbs_call_action_t action);
bool app_host_le_audio_terminate_call(uint16_t conn_id, uint8_t call_id, bool is_reject);
bool app_host_le_audio_broadcast_source_start_streaming(uint8_t start,uint32_t codec_config, uint8_t bis_count, uint32_t num_channels,
                                                        uint8_t encryption,
                                                        uint32_t broadcast_id,
                                                        uint8_t *broadcast_code);
bool app_host_le_audio_broadcast_sink_find_sources(uint8_t start);
bool app_host_le_audio_broadcast_assistant_scan_source(uint8_t start);
bool app_host_le_audio_broadcast_sink_sync_to_stream(uint8_t listen, uint8_t *broadcast_code, uint32_t broadcast_id);
bool app_host_le_audio_broadcast_assistant_select_source(uint8_t listen, uint16_t conn_id, uint8_t *broadcast_code, uint32_t broadcast_id, uint8_t use_past);


#endif //APP_HOST_LE_AUDIO_H
