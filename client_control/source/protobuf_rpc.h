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
#ifndef PROTOBUF_RPC_H
#define PROTOBUF_RPC_H

#ifdef __cplusplus
extern "C" {
#endif
typedef bool BOOLEAN;
typedef unsigned short USHORT;
typedef unsigned short uint16_t;
#define FALSE 0
//#define TRUE    ~FALSE
#define TRUE    1
#define WICED_SUCCESS 0
#define WICED_ERROR   1001
#define HCI_CONTROL_GROUP_SCRIPT                              0x25
#define HCI_CONTROL_SCRIPT_EVENT_RET_CODE                   ( ( HCI_CONTROL_GROUP_SCRIPT << 8 ) | 0x01 )   /* Script command return code */
#define HCI_CONTROL_SCRIPT_EVENT_UNKNOWN_CMD                ( ( HCI_CONTROL_GROUP_SCRIPT << 8 ) | 0x02 )   /* Unknown Script command */
#define HCI_CONTROL_SCRIPT_EVENT_CALLBACK                   ( ( HCI_CONTROL_GROUP_SCRIPT << 8 ) | 0x03 )   /* Async script callback */

typedef uint16_t wiced_result_t;

typedef struct  __protobuf_param {
    RPC_HEADER *header;
    pb_istream_t *stream;
    void* pRet;
    USHORT* ret_size;
} PROTOBUF_PARAM;

typedef BOOLEAN tPROTOBUF_RPC_FUNC_PROXY(PROTOBUF_PARAM* parm);
typedef BOOLEAN _rpc_dispatch(PROTOBUF_PARAM *parm);

extern bool read_bytes(pb_istream_t * stream, const pb_field_t *field, void **arg);
extern BOOLEAN SendStreamedResponse(PROTOBUF_PARAM* parm, pb_ostream_t *postream);
extern BOOLEAN SendBooleanResponse(PROTOBUF_PARAM* parm, RPC_BooleanResponse* response);
extern BOOLEAN SendUint32Response(PROTOBUF_PARAM* parm, RPC_Uint32Response* response);
extern void* get_protobuf_buffer(bool bInitialize);
#ifdef __cplusplus
}
#endif
#endif // PROTOBUF_RPC_H
