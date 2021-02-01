
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

#ifndef WICED_HCI_GATT_DB_H
#define WICED_HCI_GATT_DB_H

// GATT DB
typedef struct
{
    bool uuid_type_16;
    union
    {
        uint16_t uuid16;
        uint8_t uuid128[16];
    } u;
} wiced_hci_bt_uuid_t;

// GATT DB
typedef struct
{
    uint16_t handle;
    wiced_hci_bt_uuid_t uuid;
} wiced_hci_bt_service_data_t;


typedef struct
{
    wiced_hci_bt_service_data_t included_svc;
    uint16_t svc_handle;
    uint16_t end_grp;
} wiced_included_service_data_t;

typedef struct
{
    uint16_t handle;
    uint16_t handle_val;
    uint8_t prop;
    uint8_t perm;
    wiced_hci_bt_uuid_t uuid;
} wiced_characteristic_data_t;

typedef struct
{
    uint16_t handle;
    uint8_t perm;
    wiced_hci_bt_uuid_t uuid;
} wiced_descriptor_data_t;

typedef wiced_hci_bt_service_data_t wiced_hci_bt_primary_service_data_t;
typedef wiced_hci_bt_service_data_t wiced_hci_bt_secondary_service_data_t;

bool wiced_hci_gatt_db_primary_service(wiced_hci_bt_primary_service_data_t *p_data);
bool wiced_hci_gatt_db_secondary_service(wiced_hci_bt_secondary_service_data_t *p_data);
bool wiced_hci_gatt_db_included_service(wiced_included_service_data_t *p_data);
bool wiced_hci_gatt_db_characteristic(wiced_characteristic_data_t *p_data);
bool wiced_hci_gatt_db_descriptor(wiced_descriptor_data_t *p_data);
bool wiced_hci_gatt_db_set_advert_data(uint8_t *p_data, uint8_t size);
bool wiced_hci_gatt_db_init();

#endif
