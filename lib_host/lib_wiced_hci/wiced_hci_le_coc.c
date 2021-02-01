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

bool wiced_hci_le_coc_send_phy(uint16_t le2m_enable)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, le2m_enable);
    return wiced_hci_send_command(HCI_CONTROL_LE_COC_COMMAND_ENABLE_LE2M, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_coc_send_psm(uint16_t psm)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, psm);
    return wiced_hci_send_command(HCI_CONTROL_LE_COC_COMMAND_SET_PSM, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_coc_send_mtu(uint16_t mtu)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, mtu);
    return wiced_hci_send_command(HCI_CONTROL_LE_COC_COMMAND_SET_MTU, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_coc_connect(wiced_hci_bt_bda_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    BDADDR_TO_STREAM(p_cmd, p_data->bda);

    return wiced_hci_send_command(HCI_CONTROL_LE_COC_COMMAND_CONNECT, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_coc_disconnect(wiced_hci_bt_bda_t *p_data)
{
    uint8_t    cmd[10];
    uint8_t     *p_cmd = cmd;

    BDADDR_TO_STREAM(p_cmd, p_data->bda);

    return wiced_hci_send_command(HCI_CONTROL_LE_COC_COMMAND_DISCONNECT, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_le_coc_start_adv(bool start)
{
    return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_ADVERTISE, (uint8_t*)&start, 1);
}

bool wiced_hci_le_coc_send_data(uint8_t *p_data, uint16_t len)
{
    return wiced_hci_send_command(HCI_CONTROL_LE_COC_COMMAND_SEND_DATA, p_data, len);
}
