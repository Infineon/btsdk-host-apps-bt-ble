
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


#ifndef APP_HOST_AVRC_H
#define APP_HOST_AVRC_H

// AVRCP
bool app_host_avrc_ct_connect(uint8_t bda[BDA_LEN]);
bool app_host_avrc_ct_disconnect(uint8_t bda[BDA_LEN]);
bool app_host_avrc_unit_info(uint8_t bda[BDA_LEN]);
bool app_host_avrc_sub_unit_info(uint8_t bda[BDA_LEN]);

#define WICED_AVRCP_CT_PLAY_CMD        1
#define WICED_AVRCP_CT_STOP_CMD        2
#define WICED_AVRCP_CT_PAUSE_CMD       3
#define WICED_AVRCP_CT_NEXT_CMD        5
#define WICED_AVRCP_CT_PREVIOUS_CMD    6
#define WICED_AVRCP_CT_VOL_UP_CMD      7
#define WICED_AVRCP_CT_VOL_DOWN_CMD    8
#define WICED_AVRCP_CT_MUTE_CMD        9
#define WICED_AVRCP_CT_FF_CMD_PRESS    10
#define WICED_AVRCP_CT_REV_CMD_PRESS   11
#define WICED_AVRCP_CT_FF_CMD_RELEASE  12
#define WICED_AVRCP_CT_REV_CMD_RELEASE 13


/* Define the Player Application Settings IDs */
#define WICED_MAX_RC_APP_SETTING_VALUE                4
#define WICED_AVRC_PLAYER_SETTING_REPEAT              0x02
#define WICED_AVRC_PLAYER_SETTING_SHUFFLE             0x03

/* Define the possible values of the Player Application Settings */
#define WICED_AVRC_PLAYER_VAL_OFF                     0x01
#define WICED_AVRC_PLAYER_VAL_ON                      0x02
#define WICED_AVRC_PLAYER_VAL_SINGLE_REPEAT           0x02
#define WICED_AVRC_PLAYER_VAL_ALL_REPEAT              0x03
#define WICED_AVRC_PLAYER_VAL_GROUP_REPEAT            0x04
#define WICED_AVRC_PLAYER_VAL_ALL_SHUFFLE             0x02
#define WICED_AVRC_PLAYER_VAL_GROUP_SHUFFLE           0x03
#define WICED_AVRC_PLAYER_VAL_ALL_SCAN                0x02
#define WICED_AVRC_PLAYER_VAL_GROUP_SCAN              0x03

bool app_host_avrc_ct_command(uint8_t bda[BDA_LEN], uint8_t cmd);
bool app_host_avrc_ct_repeat(uint8_t bda[BDA_LEN], uint8_t setting);
bool app_host_avrc_ct_shuffle(uint8_t bda[BDA_LEN], uint8_t setting);
bool app_host_avrc_ct_volume_level(uint8_t bda[BDA_LEN], uint8_t vol_level);

void app_host_avrc_ct_event(uint16_t opcode, uint8_t * p_data, uint16_t len);


#endif
