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

#include "wiced_hci.h"
#include "hci_control_api.h"


bool wiced_hci_ans_command_set_supported_new_alert_category(wiced_hci_bt_anp_alert_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->alert);

    return wiced_hci_send_command(HCI_CONTROL_ANS_COMMAND_SET_SUPPORTED_NEW_ALERT_CATEGORIES, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_ans_command_set_supported_unread_alert_category(wiced_hci_bt_anp_alert_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->alert);

    return wiced_hci_send_command(HCI_CONTROL_ANS_COMMAND_SET_SUPPORTED_UNREAD_ALERT_CATEGORIES, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_ans_command_generate_alert(wiced_hci_bt_anp_alert_category_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT8_TO_STREAM(p_cmd, p_data->alert_category);

    return wiced_hci_send_command(HCI_CONTROL_ANS_COMMAND_GENERATE_ALERT, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_ans_command_clear_alert(wiced_hci_bt_anp_alert_category_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT8_TO_STREAM(p_cmd, p_data->alert_category);

    return wiced_hci_send_command(HCI_CONTROL_ANS_COMMAND_CLEAR_ALERT, cmd, (uint32_t)(p_cmd - cmd));
}

/********************* ANC *******************************/

bool wiced_hci_anc_command_read_server_supported_new_alerts()
{
    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_READ_SERVER_SUPPORTED_NEW_ALERTS, 0, 0);
}

bool wiced_hci_anc_command_read_server_supported_unread_alerts()
{
    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_READ_SERVER_SUPPORTED_UNREAD_ALERTS, 0, 0);
}

bool wiced_hci_anc_command_control_alerts(wiced_hci_bt_anc_control_alert_data_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT8_TO_STREAM(p_cmd, p_data->cmd_id);
    UINT8_TO_STREAM(p_cmd, p_data->alert_category);

    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_CONTROL_ALERTS, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_anc_enable_new_alerts()
{
    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_ENABLE_NEW_ALERTS, 0, 0);
}

bool wiced_hci_anc_enable_unread_alerts()
{
    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_ENABLE_UNREAD_ALERTS, 0, 0);
}

bool wiced_hci_anc_disable_new_alerts()
{
    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_DISABLE_NEW_ALERTS, 0, 0);
}

bool wiced_hci_anc_disable_unread_alerts()
{
    return wiced_hci_send_command(HCI_CONTROL_ANC_COMMAND_DISABLE_UNREAD_ALERTS, 0, 0);
}
