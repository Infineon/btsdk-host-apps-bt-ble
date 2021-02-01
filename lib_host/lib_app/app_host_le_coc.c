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

extern void updateAdvBtn(bool sts);
extern void setPhy( void );
extern void recvData(char *p_data, unsigned int len);
extern void txComplete(void);

bool app_host_le_coc_set_phy(uint16_t le2m_enable)
{
    return wiced_hci_le_coc_send_phy(le2m_enable);
}

bool app_host_le_coc_send_psm(uint16_t psm)
{
    return wiced_hci_le_coc_send_psm(psm);
}

bool app_host_le_coc_send_mtu(uint16_t mtu)
{
    return wiced_hci_le_coc_send_mtu(mtu);
}

bool app_host_le_coc_connect(uint8_t bda[6])
{
    app_host_log("Sending LE COC Connect");

    return wiced_hci_le_coc_connect((wiced_hci_bt_bda_t *)bda);
}

bool app_host_le_coc_disconnect(uint8_t bda[6])
{
    app_host_log("Disconnecting LE COC ");

    return wiced_hci_le_coc_disconnect((wiced_hci_bt_bda_t *)bda);
}

bool app_host_le_coc_start_adv(bool start)
{
    return wiced_hci_le_coc_start_adv(start);
}

bool app_host_le_coc_send_data(uint8_t *p_data, uint16_t len)
{
    return wiced_hci_le_coc_send_data(p_data, len);
}

void app_host_le_coc_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    uint8_t bda[6], i=0;

    switch (opcode)
    {

    case HCI_CONTROL_LE_COC_EVENT_CONNECTED:
        /* For now receiving NULL bd address can be treat as connection failure. Todo: proper error code handshake */
        if (!p_data[0] && !p_data[1] && !p_data[2] && !p_data[3] && !p_data[4] && !p_data[5])
        {
            app_host_log("LE COC connection failed! \n");
        }
        else
        {
            for (i = 0; i < 6; i++)
                bda[5 - i] = p_data[i];
            app_host_log("LE COC connected to %02x:%02x:%02x:%02x:%02x:%02x ", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

            //set PHY to 2M if enabled
            setPhy();
        }
        break;

    case HCI_CONTROL_LE_COC_EVENT_DISCONNECTED:
        for (i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        app_host_log("LE COC disconnected from %02x:%02x:%02x:%02x:%02x:%02x ", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        break;

    case HCI_CONTROL_LE_COC_EVENT_RX_DATA:
        //app_host_log("received %d bytes from device", len);
        recvData((char*)p_data, len);
        break;

    case HCI_CONTROL_LE_COC_EVENT_TX_COMPLETE:
        //app_host_log("received tx complete");
        txComplete();
        break;

    case HCI_CONTROL_LE_COC_EVENT_ADV_STS:
        updateAdvBtn(p_data[0]);
        break;

    default:
        app_host_log("Unknown LE COC event:%d", opcode);
        break;
    }
}
