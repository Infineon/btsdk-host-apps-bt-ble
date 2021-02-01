
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

#ifndef WICED_HCI_GATT_H
#define WICED_HCI_GATT_H

#include "wiced_types.h"


// WICED HCI command
bool wiced_hci_send_command(uint16_t command, uint8_t * payload, uint32_t len);

typedef struct
{
    uint8_t address_type;
    uint8_t address[BDA_LEN];
} wiced_hci_bt_gatt_connect_data_t;

extern bool wiced_hci_gatt_connect(wiced_hci_bt_gatt_connect_data_t * data);

typedef struct
{
    uint8_t address_type;
    uint8_t address[BDA_LEN];
} wiced_hci_bt_gatt_cancel_connect_data_t;

extern bool wiced_hci_gatt_cancel_connect(wiced_hci_bt_gatt_cancel_connect_data_t * data);

typedef struct
{
    uint16_t conn_handle;
} wiced_hci_bt_gatt_le_disconnect_data_t;

extern bool wiced_hci_gatt_le_disconnect(wiced_hci_bt_gatt_le_disconnect_data_t * data);

typedef struct
{
    uint16_t con_handle;
} wiced_hci_gatt_disc_data_t;

bool wiced_hci_gatt_disc_services(wiced_hci_gatt_disc_data_t * data);


typedef struct
{
    uint32_t s_handle;
    uint32_t e_handle;
    uint16_t conn_handle;
} wiced_hci_bt_gatt_disc_chars_data_t;
bool wiced_hci_gatt_disc_chars(wiced_hci_bt_gatt_disc_chars_data_t * data);

typedef struct
{
    uint32_t s_handle;
    uint32_t e_handle;
    uint16_t conn_handle;
} wiced_hci_bt_gatt_disc_desc_data_t;
bool wiced_hci_gatt_disc_desc(wiced_hci_bt_gatt_disc_desc_data_t * data);

typedef struct
{
    uint16_t conn_handle;
    uint32_t hdlc;
    uint8_t * str;
    uint32_t num_bytes;
} wiced_hci_bt_gatt_send_not_data_t;

bool wiced_hci_gatt_send_notif(wiced_hci_bt_gatt_send_not_data_t * data);

typedef struct
{
    uint16_t conn_handle;
    uint32_t hdlc;
    uint8_t * value;
    uint32_t num_bytes;
} wiced_hci_bt_gatt_send_indicate_data_t;

bool wiced_hci_gatt_send_indicate(wiced_hci_bt_gatt_send_indicate_data_t * data);

typedef struct
{
    uint16_t conn_handle;
    uint32_t hdlc;
    uint32_t num_bytes;
    uint8_t * value;
} wiced_hci_bt_gatt_write_char_data_t;

bool wiced_hci_gatt_write_char(wiced_hci_bt_gatt_write_char_data_t * data);
bool wiced_hci_gatt_write_char_norspn(wiced_hci_bt_gatt_write_char_data_t * data);

typedef struct
{
    uint16_t conn_handle;
    uint32_t hdlc;
} wiced_hci_bt_gatt_read_char_data_t;

bool wiced_hci_gatt_read_char(wiced_hci_bt_gatt_read_char_data_t * data);

bool wiced_hci_gatt_write_response(uint8_t * p_data);
bool wiced_hci_gatt_read_response(uint8_t * p_data);
bool wiced_hci_gatt_start_stop_advert(bool start_stop);

#endif // WICED_HCI_H
