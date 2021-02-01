
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

#ifndef WICED_HCI_DM_H
#define WICED_HCI_DM_H

#include "wiced_types.h"

typedef struct
{
    char * device_name;
} wiced_hci_dm_setdevname_data_t;

typedef struct
{
    unsigned char * device_addr;
} wiced_hci_dm_setdevaddr_data_t;

typedef struct
{
    bool mode;
} wiced_hci_dm_set_pairing_mode_data_t ;

typedef struct
{
    bool enable;
    uint8_t route;
} wiced_hci_dm_set_app_traces_data_t;

typedef struct
{
    int bda[6];
} wiced_hci_dm_bda_data_t;

typedef struct
{
    bool start_stop;
} wiced_hci_dm_inquiry_data_t;

typedef struct
{
    bool enable;
} wiced_hci_dm_le_set_scan_data_t;

typedef struct
{
    int nvram_id;
} wiced_hci_dm_delete_nvram_data_t;

bool wiced_hci_dm_delete_nvram_data(wiced_hci_dm_delete_nvram_data_t *data);


// WICED HCI command
bool wiced_hci_send_command(uint16_t command, uint8_t * payload, uint32_t len);

typedef struct
{
    bool discoverable;
    bool connectable;
} wiced_hci_dm_set_vis_data_t;

bool wiced_hci_dm_set_vis(wiced_hci_dm_set_vis_data_t* data);

typedef struct
{
    uint8_t * nvram_data;
    int len;
} wiced_hic_dm_push_host_info_data_t;

bool wiced_hci_dm_push_pairing_host_info(wiced_hic_dm_push_host_info_data_t *data);

typedef struct
{
    int nvram_id;
    uint8_t * nvram_data;
    int len;
} wiced_hci_dm_push_nvram_data_t;

bool wiced_hci_dm_push_nvram_data(wiced_hci_dm_push_nvram_data_t *data);

bool wiced_hci_dm_add_battery_client(wiced_hci_dm_bda_data_t * bda);
bool wiced_hci_dm_add_findme_locator(wiced_hci_dm_bda_data_t * bda);

typedef struct
{
    uint8_t * bda;
    bool confirm;
} wiced_hci_dm_user_confirm_data_t;

bool wiced_hci_dm_user_confirm(wiced_hci_dm_user_confirm_data_t * data);
bool wiced_hci_dm_setdevname(wiced_hci_dm_setdevname_data_t * dev_name);
bool wiced_hci_dm_setdevaddr(wiced_hci_dm_setdevaddr_data_t *device_addr);
bool wiced_hic_dm_set_pairing_mode(wiced_hci_dm_set_pairing_mode_data_t * data);
bool wiced_hci_dm_set_vis(wiced_hci_dm_set_vis_data_t* data);
bool wiced_hci_dm_reset();
bool wiced_hci_dm_inquiry(wiced_hci_dm_inquiry_data_t * data);
bool wiced_hci_dm_le_scan(wiced_hci_dm_le_set_scan_data_t * data);
bool wiced_hci_dm_push_pairing_host_info(wiced_hic_dm_push_host_info_data_t *data);
bool wiced_hci_dm_push_nvram_data(wiced_hci_dm_push_nvram_data_t *data);
bool wiced_hci_dm_hidh_add(wiced_hci_dm_bda_data_t * data);
bool wiced_hci_dm_add_battery_client(wiced_hci_dm_bda_data_t * data);
bool wiced_hci_dm_add_findme_locator(wiced_hci_dm_bda_data_t * data);
bool wiced_hci_dm_get_version_info();
bool wiced_hci_dm_set_app_traces(wiced_hci_dm_set_app_traces_data_t * data);
bool wiced_hci_dm_unbond_device(wiced_hci_dm_bda_data_t * data);

#endif
