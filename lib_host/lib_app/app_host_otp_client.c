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

extern void otpClientTxComplete(void);
extern void otpClientUpgradeInitDone(void);

bool app_host_otp_client_connect(uint8_t bda[6],uint8_t addr_type)
{
    app_host_log("Sending Connect");

    return wiced_hci_otp_client_connect((wiced_hci_bt_bda_t *)bda,addr_type);
}

bool app_host_otp_client_disconnect(uint8_t bda[6])
{
    app_host_log("Disconnecting ");

    return wiced_hci_otp_client_disconnect((wiced_hci_bt_bda_t *)bda);
}

bool app_host_otp_client_start_upgrade( uint32_t image_size )
{
    return wiced_hci_otp_client_start_upgrade(image_size);
}

bool app_host_otp_client_send_image_data(uint8_t *p_data, uint16_t len)
{
    return wiced_hci_otp_client_send_data(p_data, len);
}

bool app_host_otp_client_upgrade( uint32_t crc )
{
    return wiced_hci_otp_client_upgrade(crc);
}

void app_host_otp_client_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    uint8_t bda[6], i=0;
    uint8_t status;
    uint32_t oacp_feature;
    uint32_t olcp_feature;

    switch (opcode)
    {
    case HCI_CONTROL_OTP_EVENT_CONNECTED:
        for (i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        app_host_log("Object Server connected %02x:%02x:%02x:%02x:%02x:%02x, Len %d ", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], len);
        break;


    case HCI_CONTROL_OTP_EVENT_DISCONNECTED:
        for (i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        app_host_log("Object Server disconnected %02x:%02x:%02x:%02x:%02x:%02x ", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        break;

    case HCI_CONTROL_OTP_EVENT_DISCOVERY_COMPLETE:
        status = *p_data++;
        STREAM_TO_UINT32(oacp_feature,p_data);
        STREAM_TO_UINT32(olcp_feature,p_data);
        app_host_log("Discovery Complete. Status %x ,OTS Feature Support in Server:OACP Feature %x OLCP Feature %x ",
                      status,oacp_feature,olcp_feature );
        break;

    case HCI_CONTROL_OTP_EVENT_UPGRADE_INIT_DONE:
        app_host_log("received upgrade init done");
        otpClientUpgradeInitDone();
        break;

    case HCI_CONTROL_OTP_EVENT_TX_COMPLETE:
        //app_host_log("received tx complete");
        otpClientTxComplete();
        break;
    }
}
