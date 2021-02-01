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
#include "wiced_types.h"
#include "rpc.pb.h"
#include "wiced_hci_spp.pb.h"
#include "protobuf_rpc.h"
#include "wiced_hci.h"
#include "wiced_hci_spp.h"
#include "app_host.h"
#include "rpc.pb.h"
#include "protobuf_rpc.h"
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <stdlib.h>
extern void msleep(unsigned int to);
extern bool is_spp_connected;

typedef struct
{
    unsigned char *data;
    uint32_t data_len;
} SPP_RX_TX_DATA;

static SPP_RX_TX_DATA g_spp_rx_data = { NULL, 0 };
static SPP_RX_TX_DATA g_spp_tx_data = { NULL, 0 };

bool decode_bda(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    BOOLEAN status;
    field = field;
    status = pb_read(stream, *arg, 6);

    return status;
}

bool decode_send_data(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    BOOLEAN status;
    unsigned int len = stream->bytes_left;
    g_spp_tx_data.data_len = len;
    g_spp_tx_data.data = (unsigned char *)malloc(len);
    status = pb_read(stream, g_spp_tx_data.data, len);
    field = field;
    arg = arg;
    return status;
}

bool encode_rx_data(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    arg = arg;
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream,g_spp_rx_data.data, g_spp_rx_data.data_len);
}

unsigned char g_buffer[1024];
extern wiced_result_t hci_control_send_script_event(int type, uint8_t *p_data, uint16_t data_size);

void satisfy_wait(unsigned short res, uint32_t is_connected, uint32_t port_handle, unsigned char * rx_data, unsigned int  data_len)
{
    protobuf_WICED_HCI_SPP_EVENT_PARAMS evt_response;
    if ((g_spp_rx_data.data_len = data_len) > 0)
        g_spp_rx_data.data = rx_data;
    else
        g_spp_rx_data.data = NULL;

    evt_response.rcvd_event = res;
    evt_response.data_len = data_len;
    evt_response.is_connected = is_connected;
    evt_response.port_handle = port_handle;
    evt_response.data.funcs.encode = encode_rx_data;
    evt_response.data.arg = NULL;

    pb_ostream_t ostream = pb_ostream_from_buffer(g_buffer, sizeof(g_buffer));
    pb_encode(&ostream, protobuf_WICED_HCI_SPP_EVENT_PARAMS_fields, &evt_response);

    //pb_write(&ostream,&evt_response,sizeof(evt_response));
    unsigned int bytes_to_write = ostream.bytes_written;

    if (res == protobuf_WICED_HCI_SPP_EVENT_EVT_TX_DATA_CMPL && evt_response.data_len && rx_data)
    {
        //memcpy(&(g_buffer[ostream.bytes_written]), rx_data ,evt_response.data_len);
        //bytes_to_write = ostream.bytes_written+evt_response.data_len;
    }
    hci_control_send_script_event(0, g_buffer, bytes_to_write);
}

static BOOLEAN wiced_hci_rpc_process_spp_disconnect(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    unsigned char bda[8];
    protobuf_WICED_HCI_SPP_Disconnect req = protobuf_WICED_HCI_SPP_Disconnect_init_default;
    req.bd_addr.arg = bda;
    req.bd_addr.funcs.decode = decode_bda;
    if (pb_decode(parm->stream, protobuf_WICED_HCI_SPP_Connect_fields, &req))
        response.res = app_host_spp_disconnect(bda);
    else
        response.res = false;
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_spp_send(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    unsigned char bda[8];
    protobuf_WICED_HCI_SPP_SendData req;

    req.bd_addr.arg = bda;
    req.bd_addr.funcs.decode = decode_bda;
    g_spp_tx_data.data_len = 0;
    if (g_spp_tx_data.data)
        free(g_spp_tx_data.data);
    req.data.funcs.decode = decode_send_data;

    if (pb_decode(parm->stream, protobuf_WICED_HCI_SPP_SendData_fields, &req))
        response.res = app_host_spp_send(bda,g_spp_tx_data.data,g_spp_tx_data.data_len);
    else
        response.res = false;

    if (g_spp_tx_data.data)
        free(g_spp_tx_data.data);
    g_spp_tx_data.data = NULL;
    g_spp_tx_data.data_len = 0;
    return SendBooleanResponse(parm, &response);
}

static BOOLEAN wiced_hci_rpc_process_spp_connect(PROTOBUF_PARAM* parm)
{
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    unsigned char bda[8];
    protobuf_WICED_HCI_SPP_Connect req = protobuf_WICED_HCI_SPP_Connect_init_default;
    req.bd_addr.arg = bda;
    req.bd_addr.funcs.decode = decode_bda;
    if (pb_decode(parm->stream, protobuf_WICED_HCI_SPP_Connect_fields, &req))
    {
        if (!(response.res = app_host_spp_connect(bda)))
            response.res = is_spp_connected;
    }
    else
        response.res = false;
    return SendBooleanResponse(parm, &response);
}

//extern void start_wait_thread(PROTOBUF_PARAM * param, unsigned int to);
extern void start_waiting(uint32_t timeout, uint16_t wait_event);
static BOOLEAN wiced_hci_rpc_process_spp_wait(PROTOBUF_PARAM* parm)
{
    protobuf_WICED_HCI_SPP_WaitEvent req;
    RPC_BooleanResponse response = RPC_BooleanResponse_init_default;
    response.res = false;

    if (!pb_decode(parm->stream, protobuf_WICED_HCI_SPP_WaitEvent_fields, &req))
    {
        return SendBooleanResponse(parm, &response);
    }

    start_waiting(req.timeout, req.wait_event);
    return TRUE;
}

tPROTOBUF_RPC_FUNC_PROXY *rpc_protobuf_spp_proxies[] =
{
    wiced_hci_rpc_process_spp_wait,
    wiced_hci_rpc_process_spp_connect,
    wiced_hci_rpc_process_spp_disconnect,
    wiced_hci_rpc_process_spp_send
};

BOOLEAN wiced_hci_spp_rpc_dispatch(void *vparm)
{
    PROTOBUF_PARAM *parm = (PROTOBUF_PARAM *)vparm;
    BOOLEAN status = FALSE;
    if (parm->header->function_code.spp_function_code < sizeof(rpc_protobuf_spp_proxies) / sizeof(rpc_protobuf_spp_proxies[0]))
        status = (*rpc_protobuf_spp_proxies[parm->header->function_code.spp_function_code])(parm);
    return status;
}
