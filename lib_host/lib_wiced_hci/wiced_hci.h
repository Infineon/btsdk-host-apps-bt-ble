
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

#ifndef WICED_HCI_H
#define WICED_HCI_H

#include "wiced_types.h"

// WICED HCI command
bool wiced_hci_send_command(uint16_t command, uint8_t * payload, uint32_t len);

typedef struct
{

    uint8_t bda[BDA_LEN];
} wiced_hci_bt_bda_t;

typedef struct
{
    uint16_t handle;
} wiced_hci_bt_handle_t;

#include "wiced_hci_avrc_ct.h"
#include "wiced_hci_ag.h"
#include "wiced_hci_anp_ans.h"
#include "wiced_hci_audio_src.h"
#include "wiced_hci_dm.h"
#include "wiced_hci_gatt_db.h"
#include "wiced_hci_gatt.h"
#include "wiced_hci_hf.h"
#include "wiced_hci_hidd.h"
#include "wiced_hci_hidh.h"
#include "wiced_hci_le_coc.h"
#include "wiced_hci_otp.h"
#include "wiced_hci_spp.h"

#endif // WICED_HCI_H
