
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

#ifndef WICED_HCI_HIDH_H
#define WICED_HCI_HIDH_H

#include "wiced_types.h"
#include "wiced_hci.h"

// WICED HCI command
bool wiced_hci_send_command(uint16_t command, uint8_t * payload, uint32_t len);

// HID HOST
typedef struct
{
    uint16_t handle;
    uint8_t channle;
    uint8_t report_type;
    uint8_t report_id;
    char * string;
    uint32_t length;
} wiced_hci_bt_hidh_set_report_data_t;

typedef struct
{
    uint16_t nHandle;
    uint8_t protocol;
}wiced_hci_bt_hidh_set_proto_data_t;

typedef struct
{
    uint8_t bda[BDA_LEN];
    uint8_t report_id;
    uint8_t report_len;
    uint8_t report_pattern[255];
} wiced_hci_bt_hidh_pattern_data_t;

typedef struct
{
    uint8_t wakeup_state;
    uint8_t wakeup_gpio;
    uint8_t wakeup_polarity;
} wiced_hci_bt_hidh_wakeup_data_t;


typedef struct
{
    uint16_t handle;
    uint8_t report_type;
    uint8_t report_id;
} wiced_hci_bt_hidh_get_report_data_t;

typedef wiced_hci_bt_bda_t wiced_hci_bt_hidh_connect_data_t ;
typedef wiced_hci_bt_handle_t wiced_hci_bt_hidh_disconnect_data_t;
typedef wiced_hci_bt_handle_t wiced_hci_bt_hidh_get_desc_data_t;
typedef wiced_hci_bt_bda_t  wiced_hci_bt_hidh_unplag_data_t;
typedef wiced_hci_bt_bda_t  wiced_hci_bt_hidh_connected_data_t;

bool wiced_hci_hidh_set_report(wiced_hci_bt_hidh_set_report_data_t *p_data);
bool wiced_hci_hidh_get_report(wiced_hci_bt_hidh_get_report_data_t *p_data);
bool wiced_hci_hidh_disconnect(wiced_hci_bt_hidh_disconnect_data_t *p_data);
bool wiced_hci_hidh_virtual_unplug(wiced_hci_bt_hidh_unplag_data_t * p_data);
bool wiced_hci_hidh_get_desc(wiced_hci_bt_hidh_get_desc_data_t *data);
bool wiced_hci_hidh_set_proto(wiced_hci_bt_hidh_set_proto_data_t * data);
bool wiced_hci_hidh_set_wakeup_pattern(wiced_hci_bt_hidh_pattern_data_t * data);
bool wiced_hci_hidh_set_wakeup_control(wiced_hci_bt_hidh_wakeup_data_t * data);
bool wiced_hci_hidh_connect(wiced_hci_bt_hidh_connect_data_t * data);
bool wiced_hci_hidh_connected(wiced_hci_bt_hidh_connected_data_t *data);


#endif // WICED_HCI_H
