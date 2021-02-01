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
#include "protobuf_rpc.h"
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

static size_t length = 0;
static uint8_t buffer[1024];
void* get_protobuf_buffer(bool bInitialize)
{
    if (bInitialize)
        memset(buffer, 0, 1024);
    return (void*)buffer;
}

bool read_bytes(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    BOOLEAN status;
    field = field;
    size_t len = stream->bytes_left;
    length = len;
    status =  pb_read(stream, *arg, len);
    *((uint8_t*)*arg + len) = '\0';
    return status;
}

BOOLEAN SendStreamedResponse(PROTOBUF_PARAM* parm, pb_ostream_t *postream)
{
    BOOLEAN status = false;
    if (postream->bytes_written > 0)
    {
        memcpy(parm->pRet, buffer, postream->bytes_written);
        *(parm->ret_size) = (USHORT)postream->bytes_written;
        status = true;
    }
    return status;
}

BOOLEAN SendUint32Response(PROTOBUF_PARAM* parm, RPC_Uint32Response* response)
{
        pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        BOOLEAN status = pb_encode(&ostream, RPC_HEADER_fields, parm->header);
        if (status)
                pb_encode(&ostream, RPC_Uint32Response_fields, response);
    return SendStreamedResponse(parm, &ostream);
}


BOOLEAN SendBooleanResponse(PROTOBUF_PARAM* parm, RPC_BooleanResponse* response)
{
        pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        BOOLEAN status = pb_encode(&ostream, RPC_HEADER_fields, parm->header);
        if (status)
                pb_encode(&ostream, RPC_BooleanResponse_fields, response);
    return SendStreamedResponse(parm, &ostream);
}

extern BOOLEAN wiced_hci_gatt_db_rpc_dispatch(PROTOBUF_PARAM *parm);
extern BOOLEAN wiced_hci_spp_rpc_dispatch(PROTOBUF_PARAM *parm);
BOOLEAN null_dispatch(PROTOBUF_PARAM *parm)
{
    parm = parm;
    return false;
}
_rpc_dispatch *dispatch_table[] = {
    wiced_hci_gatt_db_rpc_dispatch,
    wiced_hci_spp_rpc_dispatch
};

uint16_t hci_control_send_script_event (int type, uint8_t *p_data, uint16_t data_size);
static uint8_t buffer[1024];
void hci_control_script_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t len)
{
    PROTOBUF_PARAM parm;
    pb_istream_t stream_header;
    pb_istream_t stream_body;
    USHORT retLen = 0;
    bool status;
    cmd_opcode = cmd_opcode;
    RPC_HEADER header = RPC_HEADER_init_zero;

    int retType = HCI_CONTROL_SCRIPT_EVENT_RET_CODE;
    // Prepare for the parameter parser and initialize return code header.

    stream_header = pb_istream_from_buffer(p_data, RPC_HEADER_size);
    status = pb_decode(&stream_header, RPC_HEADER_fields, &header);
    {

        stream_body = pb_istream_from_buffer(p_data+ RPC_HEADER_size, len- RPC_HEADER_size);
        parm.header = &header;
        parm.stream = &stream_body;
        parm.pRet = buffer;
        parm.ret_size = &retLen;
        retLen = len - RPC_HEADER_size;
        if (header.which_function_code >= RPC_HEADER_wiced_hci_gatt_db_function_code_tag && header.which_function_code <= RPC_HEADER_wiced_hci_gatt_db_function_code_tag)
        {
            status = (*dispatch_table[header.which_function_code - RPC_HEADER_wiced_hci_gatt_db_function_code_tag])(&parm);
        }
        else if (header.which_function_code >= RPC_HEADER_spp_function_code_tag && header.which_function_code <= RPC_HEADER_spp_function_code_tag)
        {
            status = wiced_hci_spp_rpc_dispatch(&parm);
            //(*wiced_hci_spp_rpc_dispatch[header.which_function_code - RPC_HEADER_spp_function_code_tag])(&parm);
        }
    }
    if (!status)
    {
            retType = HCI_CONTROL_SCRIPT_EVENT_UNKNOWN_CMD;
    }
    else if (retLen > 0)
    {
        hci_control_send_script_event(retType, parm.pRet, retLen);
    }
}

extern BOOLEAN wiced_transport_send_buffer (int type, uint8_t* p_trans_buffer, uint16_t data_size);
uint16_t hci_control_send_script_event (int type, uint8_t *p_data, uint16_t data_size)
{
    if (wiced_transport_send_buffer (type, p_data, data_size) != WICED_SUCCESS)
    {
        return (WICED_ERROR);
    }

    return WICED_SUCCESS;
}
