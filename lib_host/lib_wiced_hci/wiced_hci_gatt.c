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

#include <string.h>
#include <memory.h>
#include "wiced_hci_gatt.h"
#include "hci_control_api.h"

bool wiced_hci_gatt_connect(wiced_hci_bt_gatt_connect_data_t * data)
{
    uint8_t command[20];
    uint32_t    commandBytes = 0;
    int i = 0;

    command[commandBytes++] = data->address_type;

    for (i = 0; i < 6; i++)
        command[commandBytes++] = data->address[5 - i];

    return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_CONNECT, command, commandBytes);
}


bool wiced_hci_gatt_cancel_connect(wiced_hci_bt_gatt_cancel_connect_data_t * data)
{
    // send command to connect
    uint8_t command[20] = {0};
    uint32_t    commandBytes = 0;
    int i = 0;

    // type and BDADDR
    command[commandBytes++] = data->address_type;

    for (i = 0; i < 6; i++)
        command[commandBytes++] = data->address[5 - i];

   return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT, command, 7);
}

bool wiced_hci_gatt_le_disconnect(wiced_hci_bt_gatt_le_disconnect_data_t * data)
{
    uint8_t   command[3] = {0};
    uint32_t    commandBytes = 0;

    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_DISCONNECT, command, commandBytes);
}

bool wiced_hci_gatt_disc_services(wiced_hci_gatt_disc_data_t * data)
{
    uint8_t command[8] = { 0 };
    uint32_t    commandBytes = 0;
    command[commandBytes++] = data->con_handle & 0xff;
    command[commandBytes++] = (data->con_handle >> 8) & 0xff;
    command[commandBytes++] = 1;         // start handle
    command[commandBytes++] = 0;
    command[commandBytes++] = 0xff;      // end handle
    command[commandBytes++] = 0xff;
    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DISCOVER_SERVICES, command, commandBytes);
}

bool wiced_hci_gatt_disc_chars(wiced_hci_bt_gatt_disc_chars_data_t * data)
{
    uint8_t   command[20]  = { 0 };
    uint32_t    commandBytes = 0;

    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    command[commandBytes++] = data->s_handle & 0xff;       // start handle
    command[commandBytes++] = (data->s_handle >> 8) & 0xff;
    command[commandBytes++] = data->e_handle & 0xff;       // end handle
    command[commandBytes++] = (data->e_handle >> 8) & 0xff;

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DISCOVER_CHARACTERISTICS, command, commandBytes);
}

bool wiced_hci_gatt_disc_desc(wiced_hci_bt_gatt_disc_desc_data_t * data)
{
    uint8_t   command[20]  = { 0 };
    uint32_t    commandBytes = 0;

    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    command[commandBytes++] = data->s_handle & 0xff;       // start handle
    command[commandBytes++] = (data->s_handle >> 8) & 0xff;
    command[commandBytes++] = data->e_handle & 0xff;       // end handle
    command[commandBytes++] = (data->e_handle >> 8) & 0xff;
    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DISCOVER_DESCRIPTORS, command, commandBytes);
}

bool wiced_hci_gatt_send_notif(wiced_hci_bt_gatt_send_not_data_t * data)
{
    static uint8_t  command[32]  = { 0 };
    uint32_t    commandBytes = 0;
    bool rval = true;
    static unsigned char prev_send = 0;
    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    command[commandBytes++] = (uint8_t)(data->hdlc & 0xff);
    command[commandBytes++] = (uint8_t)((data->hdlc >> 8) & 0xff);
#ifdef REPEAT_NOTIFICATIONS_FOREVER
    if (sending_notifications)
    {
        command[commandBytes] = prev_send++;
    }
    else
#endif
    {
        memcpy(&(command[commandBytes]), data->str, data->num_bytes);
      //  QString str = ui->edtBLEHandleValue->text();
        //num_bytes = GetHexValue(&command[commandBytes], sizeof(command) - commandBytes, data->str);
        prev_send = command[commandBytes] + 1;
    }
    rval = wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_NOTIFY, command, commandBytes + data->num_bytes);
#ifdef REPEAT_NOTIFICATIONS_FOREVER
    sending_notifications = TRUE;
#else
    UNUSED(prev_send);
#endif
    return rval;
}

bool wiced_hci_gatt_send_indicate(wiced_hci_bt_gatt_send_indicate_data_t * data)
{
    uint8_t   command[32]  = { 0 };
    uint32_t    commandBytes = 0;

    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    command[commandBytes++] = (uint8_t)(data->hdlc & 0xff);
    command[commandBytes++] = (uint8_t)((data->hdlc >> 8) & 0xff);
    memcpy(&(command[commandBytes]), data->value, data->num_bytes);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_INDICATE, command, commandBytes + data->num_bytes);
}

static bool do_gatt_write(wiced_hci_bt_gatt_write_char_data_t * data, uint16_t opcode)
{
    uint8_t   command[32] = { 0 };
    uint32_t    commandBytes = 0;

    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    command[commandBytes++] = (uint8_t)(data->hdlc & 0xff);
    command[commandBytes++] = (uint8_t)((data->hdlc >> 8) & 0xff);
    memcpy(&command[commandBytes], data->value, data->num_bytes);
    return wiced_hci_send_command(opcode, command, commandBytes + data->num_bytes);
}

bool wiced_hci_gatt_write_char(wiced_hci_bt_gatt_write_char_data_t * data)
{
    return do_gatt_write(data, HCI_CONTROL_GATT_COMMAND_WRITE_REQUEST);
}

bool wiced_hci_gatt_write_char_norspn(wiced_hci_bt_gatt_write_char_data_t * data)
{
    return do_gatt_write(data, HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND);
}

bool wiced_hci_gatt_read_char(wiced_hci_bt_gatt_read_char_data_t * data)
{
    uint32_t    commandBytes = 0;
    uint8_t   command[32] = { 0 };

    command[commandBytes++] = data->conn_handle & 0xff;
    command[commandBytes++] = (data->conn_handle >> 8) & 0xff;
    command[commandBytes++] = (uint8_t)(data->hdlc & 0xff);
    command[commandBytes++] = (uint8_t)((data->hdlc >> 8) & 0xff);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_READ_REQUEST, command, commandBytes);
}

bool wiced_hci_gatt_write_response(uint8_t * p_data)
{
    // send 1 byte result code 0
    p_data[4] = 0;
    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_WRITE_RESPONSE, p_data, 5);
}

bool wiced_hci_gatt_read_response(uint8_t * p_data)
{
    // send 1 byte response with value 100;
    p_data[4] = 100;
    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_READ_RESPONSE, p_data, 5);
}
