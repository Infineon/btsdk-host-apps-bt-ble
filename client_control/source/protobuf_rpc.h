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
