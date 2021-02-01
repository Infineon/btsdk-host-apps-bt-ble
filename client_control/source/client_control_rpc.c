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
#include "gki.h"
#include "rpc.pb.h"
#include "script_app.h"
#include "client_control.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "wiced_hci.h"

BOOL32 read_uuid(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    wiced_bt_uuid_t* p_uuid = (wiced_bt_uuid_t *)*arg;
    BOOLEAN status;
    size_t len = stream->bytes_left;
    if (len == 2)
    {
        status = pb_read(stream, (void*)&(p_uuid->u.uuid16), len);
        p_uuid->uuid_type_16 = TRUE;
    }
    else (len == 16)
        ; {
        status = pb_read(stream, (void*)&(p_uuid->u.uuid128), len);
        p_uuid->uuid_type_16 = FALSE;
    }
    return status;
}

static BOOL32 wiced_hci_rpc_process_gatt_db_primary_service(PROTOBUF_PARAM* parm)
{
    wiced_bt_primary_service_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_db_primary_service request = client_control_gatt_db_primary_service_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, client_control_gatt_db_primary_service_fields, &request))
    {
        p_data.handle = request.handle;
        response.res = wiced_hci_gatt_db_primary_service(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}


static BOOL32 wiced_hci_rpc_process_gatt_db_secondary_service(PROTOBUF_PARAM* parm)
{
    wiced_bt_secondary_service_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_db_secondary_service request = client_control_gatt_db_secondary_service_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, client_control_gatt_db_secondary_service_fields, &request))
    {
        p_data.handle = request.handle;
        response.res = wiced_hci_gatt_db_secondary_service(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOL32 wiced_hci_rpc_process_gatt_db_included_service(PROTOBUF_PARAM* parm)
{
    wiced_included_service_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_db_included_service request = client_control_gatt_db_included_service_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.included_svc.uuid;
    if (pb_decode(parm->stream, client_control_gatt_db_included_service_fields, &request))
    {
        p_data.included_svc.handle = request.handle;
        p_data.svc_handle = request.svc_handle;
        p_data.end_grp = request.end_grp;
        response.res = wiced_hci_gatt_db_included_service(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOL32 wiced_hci_rpc_process_gatt_db_characteristic(PROTOBUF_PARAM* parm)
{
    wiced_characteristic_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_db_characteristic request = client_control_gatt_db_characteristic_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, client_control_gatt_db_characteristic_fields, &request))
    {
        p_data.handle = request.handle;
        p_data.handle_val = request.handle_val;
        p_data.prop = request.prop;
        p_data.perm = request.perm;
        response.res = wiced_hci_gatt_db_characteristic(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOL32 wiced_hci_rpc_process_gatt_db_descriptor(PROTOBUF_PARAM* parm)
{
    wiced_descriptor_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_db_descriptor request = client_control_gatt_db_descriptor_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, client_control_gatt_db_descriptor_fields, &request))
    {
        p_data.handle = request.handle;
        p_data.perm = request.perm;
        response.res = wiced_hci_gatt_db_descriptor(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOL32 wiced_hci_rpc_process_gatt_db_set_advert_data(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_db_set_advert_data request = client_control_gatt_db_set_advert_data_init_default;
    request.p_data.funcs.decode = &read_bytes;
    request.p_data.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, client_control_gatt_db_set_advert_data_fields, &request))
    {
        response.res = wiced_hci_gatt_db_set_advert_data((uint8_t *)request.p_data.arg, (uint8_t)request.size);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOL32 wiced_hci_rpc_process_gatt_db_init(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    response.res = wiced_hci_gatt_db_init();
    return SendBooleanResponse(parm, &response);
}

static BOOL32 wiced_hci_rpc_process_gatt_start_stop_advert(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    client_control_gatt_start_stop_advert request = client_control_gatt_start_stop_advert_init_default;
    if (pb_decode(parm->stream, client_control_gatt_start_stop_advert_fields, &request))
    {
        response.res = wiced_hci_gatt_start_stop_advert((bool)request.start);
    }
    return SendBooleanResponse(parm, &response);
}


tPROTOBUF_RPC_FUNC_PROXY *rpc_client_control_roxies[] =
{
    wiced_hci_rpc_process_gatt_db_primary_service,
    wiced_hci_rpc_process_gatt_db_secondary_service,
    wiced_hci_rpc_process_gatt_db_included_service,
    wiced_hci_rpc_process_gatt_db_characteristic,
    wiced_hci_rpc_process_gatt_db_descriptor,
    wiced_hci_rpc_process_gatt_db_set_advert_data,
    wiced_hci_rpc_process_gatt_db_init,
    wiced_hci_rpc_process_gatt_start_stop_advert,
};
