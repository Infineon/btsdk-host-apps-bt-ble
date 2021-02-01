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
#include "wiced_bt_defs.h"

void wiced_hci_gatt_db_util_uuid_to_stream(uint8_t **p, wiced_hci_bt_uuid_t *p_uuid)
{
    int i = 0;
    if(p_uuid->uuid_type_16 == true )
    {
        UINT8_TO_STREAM(*p, LEN_UUID_16);      // UUID length
        UINT16_TO_STREAM(*p, p_uuid->u.uuid16);          // UUID 16-bit
    }
    else
    {
        UINT8_TO_STREAM(*p, LEN_UUID_128);         // UUID length
        for(i = 0; i < 16; i++)          // UUID 128-bit
        {
            UINT8_TO_STREAM(*p, p_uuid->u.uuid128[16 -1 - i]);
        }
    }
}

bool wiced_hci_gatt_db_primary_service(wiced_hci_bt_primary_service_data_t *p_data)
{
    uint8_t    cmd[32] = { 0 };
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->handle);  // Service handle

    wiced_hci_gatt_db_util_uuid_to_stream(&p_cmd, &p_data->uuid);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DB_PRIMARY_SERVICE_ADD, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_gatt_db_secondary_service(wiced_hci_bt_secondary_service_data_t *p_data)
{
    uint8_t    cmd[32] = { 0 };
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->handle);  // Service handle

    wiced_hci_gatt_db_util_uuid_to_stream(&p_cmd, &p_data->uuid);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DB_SECONDARY_SERVICE_ADD, cmd, (uint32_t)(p_cmd - cmd));
}

bool wiced_hci_gatt_db_included_service(wiced_included_service_data_t *p_data)
{
    uint8_t    cmd[32] = { 0 };
    uint8_t     *p_cmd = cmd;

    UINT16_TO_STREAM(p_cmd, p_data->included_svc.handle);  // handle
    UINT16_TO_STREAM(p_cmd, p_data->svc_handle);  // Service handle
    UINT16_TO_STREAM(p_cmd, p_data->end_grp);  // end group

    wiced_hci_gatt_db_util_uuid_to_stream(&p_cmd, &p_data->included_svc.uuid);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DB_INCLUDED_SERVICE_ADD, cmd, (uint32_t)(p_cmd - cmd));


}

bool wiced_hci_gatt_db_characteristic(wiced_characteristic_data_t *p_data)
{
    uint8_t    cmd[32] = { 0 };
    uint8_t     *p = cmd;

    UINT16_TO_STREAM(p, p_data->handle);  // handle
    UINT16_TO_STREAM(p, p_data->handle_val);  // handle value
    UINT8_TO_STREAM(p, p_data->prop);  // property
    UINT8_TO_STREAM(p, p_data->perm);  // permission

    wiced_hci_gatt_db_util_uuid_to_stream(&p, &p_data->uuid);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DB_CHARACTERISTIC_ADD, cmd, (uint32_t)(p - cmd));

}

bool wiced_hci_gatt_db_descriptor(wiced_descriptor_data_t *p_data)
{
    uint8_t    cmd[32] = { 0 };
    uint8_t     *p = cmd;

    UINT16_TO_STREAM(p, p_data->handle);  // handle
    UINT8_TO_STREAM(p, p_data->perm);  // permission

    wiced_hci_gatt_db_util_uuid_to_stream(&p, &p_data->uuid);

    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DB_DESCRIPTOR_ADD, cmd, (uint32_t)(p - cmd));

}

bool wiced_hci_gatt_db_set_advert_data(uint8_t *p_data, uint8_t size)
{
    return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA, p_data, size);
}

bool wiced_hci_gatt_db_init()
{
    return wiced_hci_send_command(HCI_CONTROL_GATT_COMMAND_DB_INIT, 0, 0);
}

bool wiced_hci_gatt_start_stop_advert(bool start)
{
    uint8_t command[2] = { 0 };
    uint8_t commandBytes = 0;
    if(start)
        command[commandBytes++] = 1;
    else
        command[commandBytes++] = 0;
    return wiced_hci_send_command(HCI_CONTROL_LE_COMMAND_ADVERTISE, command, commandBytes);
}
