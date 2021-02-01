
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

#ifndef WICED_HCI_AG_H
#define WICED_HCI_AG_H

#include "wiced_types.h"

// Audio Gateway

typedef wiced_hci_bt_bda_t wiced_hci_bt_ag_connect_data_t;
typedef wiced_hci_bt_handle_t wiced_hci_bt_ag_disconnect_data_t;
typedef wiced_hci_bt_handle_t wiced_hci_bt_ag_audio_open_data_t;
typedef wiced_hci_bt_handle_t wiced_hci_bt_ag_audio_close_data_t;

bool wiced_hci_ag_connect(wiced_hci_bt_ag_connect_data_t *p_data);
bool wiced_hci_ag_disconnect(wiced_hci_bt_ag_disconnect_data_t *p_data);
bool wiced_hci_ag_audio_open(wiced_hci_bt_ag_audio_open_data_t *p_data);
bool wiced_hci_ag_audio_close(wiced_hci_bt_ag_audio_close_data_t *p_data);

#endif
