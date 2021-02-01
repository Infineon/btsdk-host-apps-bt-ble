
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
 * Definitions for WICED HCI AG
 */

#ifndef WICED_HCI_ANP_ANS_H
#define WICED_HCI_ANP_ANS_H

// ANP
#define ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT          0
#define ANP_ALERT_CATEGORY_ID_EMAIL                 1
#define ANP_ALERT_CATEGORY_ID_NEWS                  2
#define ANP_ALERT_CATEGORY_ID_CALL                  3
#define ANP_ALERT_CATEGORY_ID_MISSED_CALL           4
#define ANP_ALERT_CATEGORY_ID_SMS_OR_MMS            5
#define ANP_ALERT_CATEGORY_ID_VOICE_MAIL            6
#define ANP_ALERT_CATEGORY_ID_SCHEDULE_ALERT        7
#define ANP_ALERT_CATEGORY_ID_HIGH_PRI_ALERT        8
#define ANP_ALERT_CATEGORY_ID_INSTANT_MESSAGE       9

#define ANP_NOTIFY_CATEGORY_COUNT                   (ANP_ALERT_CATEGORY_ID_INSTANT_MESSAGE + 1)

/* Special value used by only Alert notification
 * client in notify immedietely control
 */
#define ANP_ALERT_CATEGORY_ID_ALL_CONFIGURED        0xff

// ANS
typedef struct
{
    uint16_t alert;
} wiced_hci_bt_anp_alert_data_t;

bool wiced_hci_ans_command_set_supported_new_alert_category(wiced_hci_bt_anp_alert_data_t *p_data);
bool wiced_hci_ans_command_set_supported_unread_alert_category(wiced_hci_bt_anp_alert_data_t *p_data);


typedef struct
{
    uint8_t alert_category;
} wiced_hci_bt_anp_alert_category_data_t;

bool wiced_hci_ans_command_generate_alert(wiced_hci_bt_anp_alert_category_data_t *p_data);
bool wiced_hci_ans_command_clear_alert(wiced_hci_bt_anp_alert_category_data_t *p_data);

// ANC

bool wiced_hci_anc_command_read_server_supported_new_alerts();
bool wiced_hci_anc_command_read_server_supported_unread_alerts();

#define ANP_ALERT_CONTROL_CMD_ENABLE_NEW_ALERTS                    0
#define ANP_ALERT_CONTROL_CMD_ENABLE_UNREAD_STATUS                 1
#define ANP_ALERT_CONTROL_CMD_DISABLE_NEW_ALERTS                   2
#define ANP_ALERT_CONTROL_CMD_DISABLE_UNREAD_ALERTS                3
#define ANP_ALERT_CONTROL_CMD_NOTIFY_NEW_ALERTS_IMMEDIATE          4
#define ANP_ALERT_CONTROL_CMD_NOTIFY_UNREAD_ALERTS_IMMEDIATE       5

#define ANP_ALERT_CONTROL_CMD_COUNT   (ANP_ALERT_CONTROL_CMD_NOTIFY_UNREAD_ALERTS_IMMEDIATE + 1)


typedef struct
{
    uint8_t cmd_id;
    uint8_t alert_category;
} wiced_hci_bt_anc_control_alert_data_t;

bool wiced_hci_anc_command_control_alerts(wiced_hci_bt_anc_control_alert_data_t *p_data);
bool wiced_hci_anc_enable_new_alerts();
bool wiced_hci_anc_enable_unread_alerts();
bool wiced_hci_anc_disable_new_alerts();
bool wiced_hci_anc_disable_unread_alerts();

#endif
