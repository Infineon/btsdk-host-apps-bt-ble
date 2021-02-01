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


#include "app_host.h"


void app_host_ans_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    switch (opcode)
    {
    case HCI_CONTROL_ANS_EVENT_ANS_ENABLED:
        {
            if(len == 2)
            {
                g_app.m_ans_supported_new_alerts = (uint16_t)(p_data[0] | (p_data[1] << 8));
                g_app.m_ans_supported_unread_alerts = (uint16_t)(p_data[0] | (p_data[1] << 8));
                g_app.m_ans_supported_alerts = (uint16_t)(p_data[0] | (p_data[1] << 8));

                app_host_log("HCI_CONTROL_ANS_EVENT_ANS_ENABLED, ID %d", g_app.m_ans_supported_alerts);
            }
        }
        break;
    case HCI_CONTROL_ANS_EVENT_CONNECTION_UP:
        g_app.m_ans_connected = true;
        app_host_log("HCI_CONTROL_ANS_EVENT_CONNECTION_UP");
        break;
    case HCI_CONTROL_ANS_EVENT_CONNECTION_DOWN:
        g_app.m_ans_connected = false;
        app_host_log("HCI_CONTROL_ANS_EVENT_CONNECTION_DOWN");
        break;

    case HCI_CONTROL_ANS_EVENT_COMMAND_STATUS:

        break;
    }

}


void app_host_anc_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    uint8_t status = 0;
    uint8_t cmd_id = 0;
    uint8_t category_id = 0;
    UNUSED(len);

    switch (opcode)
    {
    case HCI_CONTROL_ANC_EVENT_ANC_ENABLED:
        app_host_log("HCI_CONTROL_ANC_EVENT_ANC_ENABLED");
        g_app.m_anc_connected = true;
        break;

    case HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_NEW_ALERTS:
        status = p_data[0];
        g_app.m_anc_server_supported_new_alerts = (uint16_t)(p_data[1] | (p_data[2] << 8));
        app_host_log("HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_NEW_ALERTS, %d", g_app.m_anc_server_supported_new_alerts);
        break;

    case HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_UNREAD_ALERTS:
        status = p_data[0];
        g_app.m_anc_server_supported_unread_alerts = (uint16_t)(p_data[1] | (p_data[2] << 8));
        app_host_log("HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_UNREAD_ALERTS, %d", g_app.m_anc_server_supported_unread_alerts);
        break;

    case HCI_CONTROL_ANC_EVENT_CONTROL_ALERTS:
        status = p_data[0];
        cmd_id = p_data[1];
        category_id = p_data[2];
        app_host_log("HCI_CONTROL_ANC_EVENT_CONTROL_ALERTS, cmd_id %d, category_id %d", cmd_id, category_id);
        break;

    case HCI_CONTROL_ANC_EVENT_ENABLE_NEW_ALERTS:
        status = p_data[0];
        g_app.m_b_new_alerts_enabled = true;
        app_host_log("HCI_CONTROL_ANC_EVENT_ENABLE_NEW_ALERTS");
        break;

    case HCI_CONTROL_ANC_EVENT_DISABLE_NEW_ALERTS:
        status = p_data[0];
        g_app.m_b_new_alerts_enabled = false;
        app_host_log("HCI_CONTROL_ANC_EVENT_DISABLE_NEW_ALERTS");
        break;

    case HCI_CONTROL_ANC_EVENT_ENABLE_UNREAD_ALERTS:
        status = p_data[0];
        g_app.m_b_unread_alerts_enabled = true;
        app_host_log("HCI_CONTROL_ANC_EVENT_ENABLE_UNREAD_ALERTS");
        break;

    case HCI_CONTROL_ANC_EVENT_DISABLE_UNREAD_ALERTS:
        status = p_data[0];
        g_app.m_b_unread_alerts_enabled = false;
        app_host_log("HCI_CONTROL_ANC_EVENT_DISABLE_UNREAD_ALERTS");
        break;

    case HCI_CONTROL_ANC_EVENT_ANC_DISABLED:
        g_app.m_anc_connected = false;
        app_host_log("HCI_CONTROL_ANC_EVENT_ANC_DISABLED");
        break;

    case HCI_CONTROL_ANC_EVENT_COMMAND_STATUS:

        break;
    }

    UNUSED(status);

}
