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
 * Sample MCU application for implemeting serial port read/write on Windows OS.
 */

// The buildt in QT serial port class cannot be used on Windows OS various bugs

#include <QObject>
#include "mainwindow.h"
#include "serial_port.h"
#include <QDebug>
#include <WinSock2.h>
#include "wiced_types.h"
#include "hci_control_api.h"
#include "wiced_hci_spp.pb.h"

#include "kp3uart_workaround.h"
using namespace Cypress::KitProg3Sepcifics;

extern bool m_bClosing;

// Serial port read/write for Win32
class Win32SerialPort
{
public:
    Win32SerialPort(HWND hWnd);
    virtual ~Win32SerialPort( );

    // open serialbus driver to access device
    BOOL OpenPort(const char *str_port_name, int baudRate , int flow_control);
    void ClosePort( );

    // read data from device
    DWORD Read( LPBYTE b, DWORD dwLen );

    DWORD ReadLine( LPBYTE b, DWORD dwLen );

    // write data to device
    DWORD Write( LPBYTE b, DWORD dwLen );

    BOOL IsOpened( );

    void indicate_close();

private:
    // overlap IO for Read and Write
    OVERLAPPED m_OverlapRead;
    OVERLAPPED m_OverlapWrite;
    HANDLE m_handle;
    HANDLE m_hShutdown;
};

static Win32SerialPort* p_win_serial_port = NULL;

WicedSerialPort::WicedSerialPort(bool hostmode)
{
    UNUSED(hostmode);
    p_win_serial_port = new Win32SerialPort(NULL);
}

WicedSerialPort::~WicedSerialPort()
{
    if (p_win_serial_port)
        delete p_win_serial_port;
    p_win_serial_port = NULL;
}

int WicedSerialPort::errorNum()
{
    return 0;
}

qint64 WicedSerialPort::read(char *data, qint64 maxlen)
{
    return p_win_serial_port->Read((BYTE *)data, maxlen);
}

qint64 WicedSerialPort::readline(char *data, qint64 maxlen)
{
    return p_win_serial_port->ReadLine((BYTE *)data, maxlen);
}

qint64 WicedSerialPort::write(char *data, qint64 len)
{
    return p_win_serial_port->Write((BYTE *)data, len);
}

bool WicedSerialPort::open(const char *str_port_name, qint32 baudRate, bool bFlowControl)
{
    return p_win_serial_port->OpenPort(str_port_name, baudRate, bFlowControl);
}

void WicedSerialPort::close()
{
    return p_win_serial_port->ClosePort();
}

bool WicedSerialPort::isOpen() const
{
    return p_win_serial_port->IsOpened();
}

void WicedSerialPort::flush()
{
}

bool WicedSerialPort::waitForBytesWritten(int iMilisec)
{
    UNUSED(iMilisec);
    return true;
}

void WicedSerialPort::indicate_close()
{
    p_win_serial_port->indicate_close();
}

void Log(const char * fmt, ...)
{
    va_list cur_arg;
    va_start(cur_arg, fmt);
    char trace[1000];
    memset(trace, 0, sizeof(trace));
    vsprintf(trace, fmt, cur_arg);
    va_end(cur_arg);

    qDebug(trace);
}

//
//Class Win32SerialPort Implementation
//
Win32SerialPort::Win32SerialPort(HWND hWnd) :
    m_handle(INVALID_HANDLE_VALUE)
{
    UNUSED(hWnd);
    memset(&m_OverlapRead, 0, sizeof(m_OverlapRead));
    memset(&m_OverlapWrite, 0, sizeof(m_OverlapWrite));

    m_handle = INVALID_HANDLE_VALUE;
    m_hShutdown = INVALID_HANDLE_VALUE;
    m_bClosing = false;
}

Win32SerialPort::~Win32SerialPort()
{
    ClosePort();
}

//
//Open Serial Bus driver
//
BOOL Win32SerialPort::OpenPort(const char *str_port_name, int baudRate, int flow_control)
{
    // open once only
    if (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_handle);
    }
    m_handle = CreateFileA(str_port_name,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL);

    if (m_handle != NULL&& m_handle != INVALID_HANDLE_VALUE)
    {
        // setup serial bus device
        BOOL bResult;
        DWORD dwError = 0;
        COMMTIMEOUTS commTimeout;
        COMMPROP commProp;
        COMSTAT comStat;
        DCB serial_config;

        PurgeComm(m_handle, PURGE_RXABORT | PURGE_RXCLEAR |PURGE_TXABORT | PURGE_TXCLEAR);

        // create events for Overlapped IO
        m_OverlapRead.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

        m_OverlapWrite.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

        // set comm timeout
        memset(&commTimeout, 0, sizeof(COMMTIMEOUTS));
        commTimeout.ReadIntervalTimeout = 1;
        commTimeout.WriteTotalTimeoutConstant = 1000;
        bResult = SetCommTimeouts(m_handle, &commTimeout);

        // set comm configuration
        memset(&serial_config, 0, sizeof(serial_config));
        serial_config.DCBlength = sizeof (DCB);
        bResult = GetCommState(m_handle, &serial_config);

        serial_config.BaudRate = baudRate;
        serial_config.ByteSize = 8;
        serial_config.Parity = NOPARITY;
        serial_config.StopBits = ONESTOPBIT;
        serial_config.fBinary = TRUE;
        if (flow_control){
            serial_config.fOutxCtsFlow = TRUE; // TRUE;
            serial_config.fRtsControl = RTS_CONTROL_HANDSHAKE;
        }
        else{
            serial_config.fOutxCtsFlow = FALSE; // TRUE;
            serial_config.fRtsControl = RTS_CONTROL_DISABLE;
        }
        serial_config.fOutxDsrFlow = FALSE; // TRUE;
        serial_config.fDtrControl = FALSE;

        serial_config.fOutX = FALSE;
        serial_config.fInX = FALSE;
        serial_config.fErrorChar = FALSE;
        serial_config.fNull = FALSE;
        serial_config.fParity = FALSE;
        serial_config.XonChar = 0;
        serial_config.XoffChar = 0;
        serial_config.ErrorChar = 0;
        serial_config.EofChar = 0;
        serial_config.EvtChar = 0;
        bResult = SetCommState(m_handle, &serial_config);

        if (!bResult)
            Log ("OpenPort SetCommState failed %d\n", GetLastError());
        else
        {
            // verify CommState
            memset(&serial_config, 0, sizeof(serial_config));
            serial_config.DCBlength = sizeof (DCB);
            bResult = GetCommState(m_handle, &serial_config);
        }

        // set IO buffer size
        memset(&commProp, 0, sizeof(commProp));
        bResult = GetCommProperties(m_handle, &commProp);

        if (!bResult)
            Log ("OpenPort GetCommProperties failed %d\n", GetLastError());
        else
        {
            // use 4096 byte as preferred buffer size, adjust to fit within allowed Max
            commProp.dwCurrentTxQueue = 4096;
            commProp.dwCurrentRxQueue = 4096;
            if (commProp.dwCurrentTxQueue > commProp.dwMaxTxQueue)
                commProp.dwCurrentTxQueue = commProp.dwMaxTxQueue;
            if (commProp.dwCurrentRxQueue > commProp.dwMaxRxQueue)
                commProp.dwCurrentRxQueue = commProp.dwMaxRxQueue;
            bResult = SetupComm(m_handle, commProp.dwCurrentRxQueue, commProp.dwCurrentTxQueue);

            if (!bResult)
                Log ("OpenPort SetupComm failed %d\n", GetLastError());
            else
            {
                memset(&commProp, 0, sizeof(commProp));
                bResult = GetCommProperties(m_handle, &commProp);

                if (!bResult)
                    Log ("OpenPort GetCommProperties failed %d\n", GetLastError());
            }
        }
        memset(&comStat, 0, sizeof(comStat));
        ClearCommError(m_handle, &dwError, &comStat);

        //
        // Workaround for https://jira.cypress.com/browse/BTSDK-4891 -
        // KP3_RTS (BT_UART_CTS) stays high when Clientcontrol com port is enabled
        // Do addtional UART control flow handling, specific for the KP3 serial device on Windows.
        // For FTDI based devices this call does nothing.
        //
        Kp3UartWorkaround::assertRtsDtrLinesForKP3OnWindows(str_port_name, m_handle);

    }
    Log ("Opened%s at speed: %u flow %s", str_port_name, baudRate, flow_control?"on":"off");
    m_bClosing = false;
    m_hShutdown = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE)
    {
        g_pMainWindow->CreateReadPortThread();
        g_pMainWindow->m_port_read_thread->start();
        return true;
    }
    return false;
}

void Win32SerialPort::ClosePort()
{
    SetEvent(m_hShutdown);

    if (m_OverlapRead.hEvent != NULL)
    {
        CloseHandle(m_OverlapRead.hEvent);
        m_OverlapRead.hEvent = NULL;
    }

    if (m_OverlapWrite.hEvent != NULL)
    {
        CloseHandle(m_OverlapWrite.hEvent);
        m_OverlapWrite.hEvent = NULL;
    }
    if (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE)
    {
        // drop DTR
        EscapeCommFunction(m_handle, CLRDTR);
        // purge any outstanding reads/writes and close device handle
        PurgeComm(m_handle, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
    }
}

BOOL Win32SerialPort::IsOpened()
{
    return (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE);
}

// read a number of bytes from Serial Bus Device
// Parameters:
//	lpBytes - Pointer to the buffer
//	dwLen   - number of bytes to read
// Return:	Number of byte read from the device.
//
DWORD Win32SerialPort::Read(LPBYTE lpBytes, DWORD dwLen)
{
    LPBYTE p = lpBytes;
    DWORD Length = dwLen;
    DWORD dwRead = 0;
    DWORD dwTotalRead = 0;

    // Loop here until request is fulfilled
    while (Length)
    {
        DWORD dwRet = WAIT_TIMEOUT;
        dwRead = 0;
        ResetEvent(m_OverlapRead.hEvent);

        if (!ReadFile(m_handle, (LPVOID)p, Length, &dwRead, &m_OverlapRead))
        {
            // Overlapped IO returns FALSE with ERROR_IO_PENDING
            if (GetLastError() != ERROR_IO_PENDING)
            {
                Log ("Win32SerialPort::ReadFile failed with %ld\n", GetLastError());
                m_bClosing = true;
                dwTotalRead = 0;
                break;
            }

            HANDLE handles[2];
            handles[0] = m_OverlapRead.hEvent;
            handles[1] = m_hShutdown;

            dwRet = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
            if (dwRet == WAIT_OBJECT_0 + 1)
            {
                m_bClosing = true;
                break;
            }
            else if (dwRet != WAIT_OBJECT_0)
            {
                Log ("Win32SerialPort::WaitForSingleObject returned with %ld err=%d\n", dwRet, GetLastError());
                dwTotalRead = 0;
                break;
            }

            // IO completed, retrieve Overlapped result
            GetOverlappedResult(m_handle, &m_OverlapRead, &dwRead, TRUE);

        }
        if (dwRead > Length)
            break;
        p += dwRead;
        Length -= dwRead;
        dwTotalRead += dwRead;
    }

    return dwTotalRead;
}

DWORD Win32SerialPort::ReadLine(LPBYTE lpBytes, DWORD dwLen)
{
    LPBYTE p = lpBytes;
    DWORD Length = dwLen;
    DWORD dwRead = 0;
    DWORD dwTotalRead = 0;

    // Loop here until request is fulfilled
    while (Length)
    {
        DWORD dwRet = WAIT_TIMEOUT;
        dwRead = 0;
        ResetEvent(m_OverlapRead.hEvent);

        if (!ReadFile(m_handle, (LPVOID)p, Length, &dwRead, &m_OverlapRead))
        {
            // Overlapped IO returns FALSE with ERROR_IO_PENDING
            if (GetLastError() != ERROR_IO_PENDING)
            {
                Log ("Win32SerialPort::ReadFile failed with %ld\n", GetLastError());
                m_bClosing = true;
                dwTotalRead = 0;
                break;
            }

            HANDLE handles[2];
            handles[0] = m_OverlapRead.hEvent;
            handles[1] = m_hShutdown;

            dwRet = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
            if (dwRet == WAIT_OBJECT_0 + 1)
            {
                m_bClosing = true;
                break;
            }
            else if (dwRet != WAIT_OBJECT_0)
            {
                Log ("Win32SerialPort::WaitForSingleObject returned with %ld err=%d\n", dwRet, GetLastError());
                dwTotalRead = 0;
                break;
            }

            // IO completed, retrieve Overlapped result
            GetOverlappedResult(m_handle, &m_OverlapRead, &dwRead, TRUE);

        }

        dwTotalRead += dwRead;
        if (dwRead > 0)
            break;
        p += dwRead;
        Length -= dwRead;

    }

    return dwTotalRead;
}

// Write a number of bytes to Serial Bus Device
// Parameters:
//	lpBytes - Pointer to the buffer
//	dwLen   - number of bytes to write
// Return:	Number of byte Written to the device.
//
DWORD Win32SerialPort::Write(LPBYTE lpBytes, DWORD dwLen)
{
    LPBYTE p = lpBytes;
    DWORD Length = dwLen;
    DWORD dwWritten = 0;
    DWORD dwTotalWritten = 0;

    if (m_handle == INVALID_HANDLE_VALUE)
    {
        Log ("ERROR - COM Port not opened");
        return (0);
    }

    while (Length)
    {
        dwWritten = 0;
        SetLastError(ERROR_SUCCESS);
        ResetEvent(m_OverlapWrite.hEvent);
        if (!WriteFile(m_handle, p, Length, &dwWritten, &m_OverlapWrite))
        {
            if (GetLastError() != ERROR_IO_PENDING)
            {
                Log ("Win32SerialPort::WriteFile failed with %ld", GetLastError());
                break;
            }
            DWORD dwRet = WaitForSingleObject(m_OverlapWrite.hEvent, INFINITE);
            if (dwRet != WAIT_OBJECT_0)
            {
                Log ("Win32SerialPort::Write WaitForSingleObject failed with %ld\n", GetLastError());
                break;
            }
            GetOverlappedResult(m_handle, &m_OverlapWrite, &dwWritten, FALSE);
        }
        if (dwWritten > Length)
            break;
        p += dwWritten;
        Length -= dwWritten;
        dwTotalWritten += dwWritten;
    }
    return dwTotalWritten;
}

void Win32SerialPort::indicate_close()
{
    SetEvent(m_hShutdown);
}

void WicedSerialPort::handleReadyRead()
{

}


bool openSerialPort(QSerialPort & serial)
{
    return serial.open(QIODevice::ReadWrite);
}

// some earlier versions of the Microsoft compiler do not support snprintf()
// so we supply this funtion which is functionaly equivalent
int ms_snprintf ( char * s, size_t n, const char * fmt, ... )
{
    memset(s,0,n);

    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(s,n-1,fmt,ap);
    va_end(ap);
    return(r);
}


WicedSerialPortHostmode::WicedSerialPortHostmode()
    : WicedSerialPort(true)
{
        m_ClientSocket  = INVALID_SOCKET;
}

qint64 WicedSerialPortHostmode::read(char *data, qint64 maxlen)
{
    if (m_ClientSocket == (int) INVALID_SOCKET)
    {
        if (!OpenSocket())
            return 0;
    }
    qint64 read_data = recv(m_ClientSocket, data, maxlen, 0);
    if((read_data == SOCKET_ERROR) || (read_data > maxlen))
        return 0;
    return read_data;

}

int WicedSerialPortHostmode::errorNum()
{
    return 0;
}

void WicedSerialPortHostmode::handleReadyRead()
{

}

qint64 WicedSerialPortHostmode::write(char *data, qint64 len)
{
    if (m_ClientSocket == (int) INVALID_SOCKET)
    {
        OpenSocket();
    }

    if (m_ClientSocket != (int) INVALID_SOCKET)
    {
        if (SOCKET_ERROR == send( m_ClientSocket, (char *)data, len, 0 ))
        {
            qDebug("send failed with error: %d\n", errno);
            return -1;
}
    }
    else
        return -1;
    return len;

}

#define SOCK_PORT_NUM	12012
//void init_winsock();

void init_winsock()
{
#ifdef WIN32
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0)
    {
        printf("WSAStartup failed with error: %d\n", iResult);
    }
#endif
}

bool WicedSerialPortHostmode::OpenSocket()
{
    init_winsock();

    //    if (INVALID_SOCKET == (m_ClientSocket = socket(AF_INET, SOCK_STREAM, 0)))
    if ((int) INVALID_SOCKET == (m_ClientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        qDebug("socket failed with error: %ld, socket thread exiting\n", (long int) errno);
        return false;
    }

    struct sockaddr_in service;

    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr("127.0.0.1");
    service.sin_port = htons(SOCK_PORT_NUM);

    // Connect to server.
   if (SOCKET_ERROR == connect( m_ClientSocket, (const sockaddr*) & service, sizeof (service)))
   {
       ::closesocket(m_ClientSocket);
       m_ClientSocket = INVALID_SOCKET;
       return false;
   }

   return true;
}

bool WicedSerialPortHostmode::open(const char *str_port_name, qint32 baudRate, bool bFlowControl)
{
    UNUSED(str_port_name);
    UNUSED(baudRate);
    UNUSED(bFlowControl);

    OpenSocket();
    g_pMainWindow->CreateReadPortThread();
    g_pMainWindow->m_port_read_thread->start();

    return true;
}

void WicedSerialPortHostmode::close()
{
    if (m_ClientSocket != (int) INVALID_SOCKET)
    {
        ::closesocket(m_ClientSocket);
        m_ClientSocket = INVALID_SOCKET;
    }
}

bool WicedSerialPortHostmode::isOpen() const
{
    return (m_ClientSocket != (int) INVALID_SOCKET);
}

void WicedSerialPortHostmode::flush()
{

}

bool WicedSerialPortHostmode::waitForBytesWritten(int iMilisec)
{
    UNUSED(iMilisec);
    return true;
}

/******************************************************/
/* Scripting support */

static SOCKET  m_ListenScriptSocket = INVALID_SOCKET;
static SOCKET  m_ClientScriptSocket = INVALID_SOCKET;
void SendScriptReturnCode(int iRet);

extern "C"
{
void start_waiting(uint32_t timeout, uint16_t wait_event);
extern void satisfy_wait(unsigned short res, uint32_t is_connected, uint32_t port_handle, unsigned char * rx_data, unsigned int  data_len);
extern void script_handle_event(uint16_t opcode, uint32_t is_connected, uint32_t port_handle, uint8_t * p_data, uint32_t len);
}

QMutex g_wait_mutex;
uint16_t g_wait_evt = protobuf_WICED_HCI_SPP_EVENT_EVT_NONE;
HANDLE g_wait_timer = NULL;

void script_handle_event(uint16_t opcode, uint32_t is_connected, uint32_t port_handle, uint8_t * p_data, uint32_t len)
{
    g_wait_mutex.lock();
    opcode = opcode & 0x00ff;
    if (g_wait_evt == protobuf_WICED_HCI_SPP_EVENT_EVT_ANY || g_wait_evt == opcode)
    {
        if (g_wait_timer)
        {
            DeleteTimerQueueTimer(NULL, g_wait_timer, NULL);
            g_wait_timer = NULL;
        }
        satisfy_wait(opcode, is_connected, port_handle, p_data, len);
        g_wait_evt = protobuf_WICED_HCI_SPP_EVENT_EVT_NONE;
    }

    g_wait_mutex.unlock();
}

VOID CALLBACK script_wait_tmr_cb(PVOID lpParameter,  BOOLEAN TimerOrWaitFired)
{
    UNUSED(lpParameter); // silence warning
    UNUSED(TimerOrWaitFired); // silence warning
    g_wait_mutex.lock();
    g_wait_evt = protobuf_WICED_HCI_SPP_EVENT_EVT_NONE;
    g_wait_mutex.unlock();
    DeleteTimerQueueTimer(NULL, g_wait_timer, NULL);
    g_wait_timer = NULL;
    satisfy_wait(protobuf_WICED_HCI_SPP_EVENT_EVT_NONE, false, 0, NULL, 0);
}

void start_waiting(uint32_t timeout, uint16_t wait_event)
{
    g_wait_mutex.lock();
    if (timeout)
        CreateTimerQueueTimer(&g_wait_timer, NULL, script_wait_tmr_cb, NULL, timeout, 0, 0);
    g_wait_evt = wait_event;
    g_wait_mutex.unlock();
}

typedef struct
{
    UINT8   pkt_type;
    UINT8   data[1200];
} tSCRIPT_PKT;


int read_script_pct(char *pPkt)
{
    unsigned int readLen, hdrLen, dataLen;

    if ( (readLen = recv (m_ClientScriptSocket, (char *)pPkt, 1, 0)) != 1)
    {
        qDebug("read_script_pct() Expected 1, got: %d", readLen);
        return (-1);
    }

    // ACL and WICED-HCI share the same basic format
    if (pPkt[0] == HCI_WICED_PKT)
    {
        if ((hdrLen = recv (m_ClientScriptSocket, (char *)&pPkt[1], 4, 0)) != 4)
        {
            qDebug("read_script_pct() Expected 4, got: %d", readLen);
            return (-1);
        }
        dataLen = pPkt[3] | (pPkt[4] << 8);
    }
    else
    {
        qDebug ("!!!!Unknown Type: %u", pPkt[0]);
        return (-1);
    }
    if (dataLen != 0)
    {
        if ((readLen = recv(m_ClientScriptSocket, (char *)&pPkt[1 + hdrLen], dataLen, 0)) != dataLen)
        {
            qDebug ("read_script_pct() Expected to read datalen of %u, actually got: %d", dataLen, readLen);
            return (-1);
        }
    }

    return (1 + hdrLen + dataLen);
}

// Script read thread
void Worker::read_script_thread()
{
    tSCRIPT_PKT         pkt;
    int                 iResult;
    SOCKADDR_IN         service;
    int                 bytes_rcvd;

    init_winsock();

    // Create a local SOCKET for incoming connection
    if (INVALID_SOCKET == (m_ListenScriptSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        qDebug("listen socket failed with error: %d, socket thread exiting\n", WSAGetLastError());
        return;
    }

    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr("127.0.0.1");
    service.sin_port = htons(11012);

    if (SOCKET_ERROR == (iResult = bind(m_ListenScriptSocket, (SOCKADDR *)& service, sizeof(service))))
    {
        qDebug ("bind failed with error: %d, socket thread exiting\n", WSAGetLastError());
        closesocket (m_ListenScriptSocket);
        return;
    }

    for ( ;  ; )
    {
        qDebug ("Listening for client to connect TCP socket....");

        if (m_ClientScriptSocket != INVALID_SOCKET)
        {
            closesocket(m_ClientScriptSocket);
            m_ClientScriptSocket = INVALID_SOCKET;
        }

        if (SOCKET_ERROR == (iResult = listen(m_ListenScriptSocket, 1)))
        {
            qDebug ("TCP socket listen failed with error: %d\n", WSAGetLastError());
            break;
        }

        // Accept the client TCP socket
        if (INVALID_SOCKET == (m_ClientScriptSocket = accept(m_ListenScriptSocket, NULL, NULL)))
        {
            qDebug("Client TCP socket accept failed with error: %d", WSAGetLastError());
            break;
        }

        qDebug("Client TCP socket accepted OK");

        // Receive until the peer shuts down the connection
        for ( ; ; )
        {
            bytes_rcvd = read_script_pct ((char *)&pkt);

            if (bytes_rcvd <= 0)
            {
                qDebug("Client TCP socket recv failed with error: %d    Closing...", WSAGetLastError());
                break;
            }

            if(pkt.pkt_type == HCI_WICED_PKT)
            {
                int iRet = m_pParent->CallWicedHciApi(pkt.data, bytes_rcvd - 1);

                SendScriptReturnCode(iRet);
            }
        }
    }
}

// Send return code to script
void SendScriptReturnCode(int iRet)
{
    // Return code to python
    tSCRIPT_PKT temp;
    memset(&temp.data, 0, 1200);
    temp.pkt_type = HCI_WICED_PKT;
    temp.data[0] = 0x1;
    temp.data[1] = 0x23;
    temp.data[2] = 5;
    temp.data[3] = iRet;
    if (SOCKET_ERROR == (iRet = send(m_ClientScriptSocket, (char *)&temp, 9+1, 0)))
    {

    }
}

extern "C" BOOLEAN wiced_transport_send_buffer (int type, uint8_t* p_trans_buffer, uint16_t data_size)
{
    UNUSED(type); // silence warning
    // Return code to python
    tSCRIPT_PKT temp;
    memset(&temp.data, 0, 1200);
    temp.pkt_type = HCI_WICED_PKT;
    temp.data[0] = 0x1;
    temp.data[1] = 0x24;
    temp.data[2] = (data_size & 0x00FF);
    temp.data[3] = (data_size >> 8) & 0x00FF;
    memcpy(&temp.data[4], p_trans_buffer, data_size);
    if (SOCKET_ERROR == send(m_ClientScriptSocket, (char *)&temp, data_size + 5, 0))
    {
        return FALSE;
    }
    return TRUE;
}
