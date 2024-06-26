/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9 at Wed Oct 31 16:42:06 2018. */

#include "wiced_hci_spp.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t protobuf_WICED_HCI_SPP_CONNECTED_PARAMS_fields[4] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, protobuf_WICED_HCI_SPP_CONNECTED_PARAMS, wait_status, wait_status, 0),
    PB_FIELD(  2, BOOL    , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_CONNECTED_PARAMS, is_connected, wait_status, 0),
    PB_FIELD(  3, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_CONNECTED_PARAMS, port_handle, is_connected, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_DISCONNECTED_PARAMS_fields[3] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, protobuf_WICED_HCI_SPP_DISCONNECTED_PARAMS, wait_status, wait_status, 0),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_DISCONNECTED_PARAMS, port_handle, wait_status, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_RX_DATA_PARAMS_fields[5] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, protobuf_WICED_HCI_SPP_RX_DATA_PARAMS, wait_status, wait_status, 0),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_RX_DATA_PARAMS, port_handle, wait_status, 0),
    PB_FIELD(  3, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_RX_DATA_PARAMS, data_len, port_handle, 0),
    PB_FIELD(  4, BYTES   , SINGULAR, CALLBACK, OTHER, protobuf_WICED_HCI_SPP_RX_DATA_PARAMS, data, data_len, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_EVENT_PARAMS_fields[6] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, protobuf_WICED_HCI_SPP_EVENT_PARAMS, rcvd_event, rcvd_event, 0),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_EVENT_PARAMS, is_connected, rcvd_event, 0),
    PB_FIELD(  3, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_EVENT_PARAMS, port_handle, is_connected, 0),
    PB_FIELD(  4, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_EVENT_PARAMS, data_len, port_handle, 0),
    PB_FIELD(  5, BYTES   , SINGULAR, CALLBACK, OTHER, protobuf_WICED_HCI_SPP_EVENT_PARAMS, data, data_len, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_Listen_fields[3] = {
    PB_FIELD(  1, STRING  , SINGULAR, CALLBACK, FIRST, protobuf_WICED_HCI_SPP_Listen, service_name, service_name, 0),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_Listen, mtu, service_name, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_Disconnect_fields[2] = {
    PB_FIELD(  1, BYTES   , SINGULAR, CALLBACK, FIRST, protobuf_WICED_HCI_SPP_Disconnect, bd_addr, bd_addr, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_Connect_fields[2] = {
    PB_FIELD(  1, BYTES   , SINGULAR, CALLBACK, FIRST, protobuf_WICED_HCI_SPP_Connect, bd_addr, bd_addr, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_SendData_fields[3] = {
    PB_FIELD(  1, BYTES   , SINGULAR, CALLBACK, FIRST, protobuf_WICED_HCI_SPP_SendData, bd_addr, bd_addr, 0),
    PB_FIELD(  2, BYTES   , SINGULAR, CALLBACK, OTHER, protobuf_WICED_HCI_SPP_SendData, data, bd_addr, 0),
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_Remove_fields[1] = {
    PB_LAST_FIELD
};

const pb_field_t protobuf_WICED_HCI_SPP_WaitEvent_fields[3] = {
    PB_FIELD(  1, UINT32  , SINGULAR, STATIC  , FIRST, protobuf_WICED_HCI_SPP_WaitEvent, timeout, timeout, 0),
    PB_FIELD(  2, UENUM   , SINGULAR, STATIC  , OTHER, protobuf_WICED_HCI_SPP_WaitEvent, wait_event, timeout, 0),
    PB_LAST_FIELD
};



/* @@protoc_insertion_point(eof) */
