
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

/*
 * Definitions for WICED HCI
 */

#ifndef WICED_HCI_AVRC_CT_H
#define WICED_HCI_AVRC_CT_H

// AVRC CT
typedef wiced_hci_bt_bda_t wiced_bt_avrc_ct_connect_data_t;
typedef wiced_hci_bt_handle_t wiced_bt_avrc_ct_disconnect_data_t;
typedef wiced_hci_bt_handle_t wiced_bt_avrc_ct_cmd_data_t;
typedef wiced_hci_bt_handle_t wiced_bt_avrc_ct_cmd_unit_info_data_t;
typedef wiced_hci_bt_handle_t wiced_bt_avrc_ct_cmd_sub_unit_info_data_t;

bool wiced_hci_avrc_ct_connect(wiced_bt_avrc_ct_connect_data_t *p_data);
bool wiced_hci_avrc_ct_disconnect(wiced_bt_avrc_ct_disconnect_data_t *p_data);
bool wiced_hci_avrc_ct_play(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_stop(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_pause(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_next_track(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_previous_track(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_volume_up(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_volume_down(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_mute(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_skip_forward_pressed(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_skip_forward_released(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_skip_backward_pressed(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_skip_backward_released(wiced_bt_avrc_ct_cmd_data_t *p_data);
bool wiced_hci_avrc_ct_button_press();
bool wiced_hci_avrc_ct_long_button_press();
bool wiced_hci_avrc_unit_info(wiced_bt_avrc_ct_cmd_unit_info_data_t *p_data);
bool wiced_hci_avrc_sub_unit_info(wiced_bt_avrc_ct_cmd_sub_unit_info_data_t *p_data);

// Setting type
#define WICED_AVRC_PLAYER_SETTING_REPEAT              0x02
#define WICED_AVRC_PLAYER_SETTING_SHUFFLE             0x03

// Repeat settings
#define WICED_AVRC_PLAYER_VAL_OFF_REPEAT              0x01
#define WICED_AVRC_PLAYER_VAL_SINGLE_REPEAT           0x02
#define WICED_AVRC_PLAYER_VAL_ALL_REPEAT              0x03
#define WICED_AVRC_PLAYER_VAL_GROUP_REPEAT            0x04

// Shuffle settings
#define WICED_AVRC_PLAYER_VAL_OFF_SHUFFLE             0x01
#define WICED_AVRC_PLAYER_VAL_ALL_SHUFFLE             0x02
#define WICED_AVRC_PLAYER_VAL_GROUP_SHUFFLE           0x03

typedef struct
{
    uint16_t handle;
    uint8_t  setting;
} wiced_bt_avrc_ct_settings_data_t;

bool wiced_hci_avrc_ct_repeat(wiced_bt_avrc_ct_settings_data_t *p_data);
bool wiced_hci_avrc_ct_shuffle(wiced_bt_avrc_ct_settings_data_t *p_data);

typedef struct
{
    uint16_t handle;
    uint8_t  volume_level;
} wiced_bt_avrc_ct_volume_level_data_t;

bool wiced_hci_avrc_ct_volume_level(wiced_bt_avrc_ct_volume_level_data_t *p_data);

#endif // WICED_HCI_H
