#include "wiced_types.h"
#include "rpc.pb.h"
#include "wiced_hci_gatt_db.pb.h"
#include "protobuf_rpc.h"
#include "wiced_hci.h"
#include "wiced_bt_defs.h"
#include "wiced_hci_gatt_db.h"

#include <pb_decode.h>
void wiced_trace_error(char* msg)
{
    msg = msg;//do nothing
}

bool read_uuid(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    BOOLEAN status;
    wiced_bt_uuid_t *p_uuid = (wiced_bt_uuid_t *)*arg;
    uint16_t len = stream->bytes_left;
    void* p = *arg;
    field = field;
    if (len == 2)
    {
        p = (void*)&p_uuid->uu.uuid16;
        p_uuid->uu.uuid16 = TRUE;
    }
/*    else if (p_uuid->len == 4)
        p = (void*)&p_uuid->u.uuid32; */
    else
    {
        p = (void*)&p_uuid->uu.uuid128;
        p_uuid->uu.uuid16 = FALSE;
    }
    status = pb_read(stream, p, len);

    return status;
}

static BOOLEAN wiced_hci_rpc_process_gatt_db_primary_service(PROTOBUF_PARAM* parm)
{
    wiced_hci_bt_primary_service_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_db_primary_service request = wiced_hci_gatt_db_gatt_db_primary_service_init_default;
    request.uuid.funcs.decode = read_uuid;
    request.uuid.arg = &p_data.uuid;
    bool b = pb_decode(parm->stream, wiced_hci_gatt_db_gatt_db_primary_service_fields, &request);
    if (b)
    {
        p_data.handle = request.handle;
        response.res = wiced_hci_gatt_db_primary_service(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}


static BOOLEAN wiced_hci_rpc_process_gatt_db_secondary_service(PROTOBUF_PARAM* parm)
{
    wiced_hci_bt_secondary_service_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_db_secondary_service request = wiced_hci_gatt_db_gatt_db_secondary_service_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, wiced_hci_gatt_db_gatt_db_secondary_service_fields, &request))
    {
        p_data.handle = request.handle;
        response.res = wiced_hci_gatt_db_secondary_service(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_gatt_db_included_service(PROTOBUF_PARAM* parm)
{
    wiced_included_service_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_db_included_service request = wiced_hci_gatt_db_gatt_db_included_service_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.included_svc.uuid;
    if (pb_decode(parm->stream, wiced_hci_gatt_db_gatt_db_included_service_fields, &request))
    {
        p_data.included_svc.handle = request.handle;
        p_data.svc_handle = request.svc_handle;
        p_data.end_grp = request.end_grp;
        response.res = wiced_hci_gatt_db_included_service(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_gatt_db_characteristic(PROTOBUF_PARAM* parm)
{
    wiced_characteristic_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_db_characteristic request = wiced_hci_gatt_db_gatt_db_characteristic_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, wiced_hci_gatt_db_gatt_db_characteristic_fields, &request))
    {
        p_data.handle = request.handle;
        p_data.handle_val = request.handle_val;
        p_data.prop = request.prop;
        p_data.perm = request.perm;
        response.res = wiced_hci_gatt_db_characteristic(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_gatt_db_descriptor(PROTOBUF_PARAM* parm)
{
    wiced_descriptor_data_t p_data;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_db_descriptor request = wiced_hci_gatt_db_gatt_db_descriptor_init_default;
    request.uuid.funcs.decode = &read_uuid;
    request.uuid.arg = &p_data.uuid;
    if (pb_decode(parm->stream, wiced_hci_gatt_db_gatt_db_descriptor_fields, &request))
    {
        p_data.handle = request.handle;
        p_data.perm = request.perm;
        response.res = wiced_hci_gatt_db_descriptor(&p_data);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_gatt_db_set_advert_data(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_db_set_advert_data request = wiced_hci_gatt_db_gatt_db_set_advert_data_init_default;
     request.p_data.funcs.decode = &read_bytes;
    request.p_data.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, wiced_hci_gatt_db_gatt_db_set_advert_data_fields, &request))
    {
        response.res = wiced_hci_gatt_db_set_advert_data((uint8_t *)request.p_data.arg, (uint8_t)request.size);
    }
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_gatt_db_init(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    response.res = wiced_hci_gatt_db_init();
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_gatt_start_stop_advert(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    wiced_hci_gatt_db_gatt_start_stop_advert request = wiced_hci_gatt_db_gatt_start_stop_advert_init_default;
    if (pb_decode(parm->stream, wiced_hci_gatt_db_gatt_start_stop_advert_fields, &request))
    {
        response.res = wiced_hci_gatt_start_stop_advert((bool)request.start);
    }
    return SendBooleanResponse(parm, &response);
}


tPROTOBUF_RPC_FUNC_PROXY *rpc_wiced_hci_gatt_db_roxies[] =
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

BOOLEAN wiced_hci_gatt_db_rpc_dispatch(void *vparm)
{
    PROTOBUF_PARAM *parm = (PROTOBUF_PARAM *)vparm;
    BOOLEAN status = FALSE;
    if (parm->header->function_code.wiced_hci_gatt_db_function_code < sizeof(rpc_wiced_hci_gatt_db_roxies) / sizeof(rpc_wiced_hci_gatt_db_roxies[0]))
        status = (*rpc_wiced_hci_gatt_db_roxies[parm->header->function_code.wiced_hci_gatt_db_function_code])(parm);
    return status;
}
