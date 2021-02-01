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
#include <QObject>
#include "serial_port.h"
#include <QDebug>

#ifdef Q_OS_WIN32
#include <WinSock2.h>
#endif


// Serial port read/write class

WicedSerialPort::WicedSerialPort()
{
    bool bWin32 = false;
#ifdef Q_OS_WIN32
    p_win_serial_port = new Win32SerialPort(NULL);
    bWin32 = true;
#endif

    if(!bWin32)
        p_qt_serial_port = new QSerialPort(NULL);
    else
        p_qt_serial_port = NULL;

}

WicedSerialPort::~WicedSerialPort()
{
#ifdef Q_OS_WIN32
    delete p_win_serial_port;
#endif
    if(p_qt_serial_port)
        delete p_qt_serial_port;
}

qint64 WicedSerialPort::read(char *data, qint64 maxlen)
{
    qint64 read = 0;
#ifdef Q_OS_WIN32
    read = p_win_serial_port->Read((BYTE *)data, maxlen);
#endif
    if(p_qt_serial_port)
        read = p_qt_serial_port->read(data, maxlen);

    return read;
}

qint64 WicedSerialPort::write(const char *data, qint64 len)
{
    qint64 written = 0;
#ifdef Q_OS_WIN32
    written = p_win_serial_port->Write((BYTE *)data, len);
#endif
    if(p_qt_serial_port)
        written = p_qt_serial_port->write(data, len);

    return written;
}

bool WicedSerialPort::open(const char *str_port_name, qint32 baudRate, bool bFlowControl)
{
    bool bopen = false;

#ifdef Q_OS_WIN32
    bopen = p_win_serial_port->OpenPort(str_port_name, baudRate, bFlowControl);
#endif

    if(p_qt_serial_port)
    {
        QString serialPortName = str_port_name;
        p_qt_serial_port->setPortName(serialPortName);
        p_qt_serial_port->setBaudRate(baudRate);
        p_qt_serial_port->setFlowControl(bFlowControl ? QSerialPort::HardwareControl : QSerialPort::NoFlowControl);
        p_qt_serial_port->setStopBits(QSerialPort::OneStop);
        p_qt_serial_port->setDataBits(QSerialPort::Data8);
        p_qt_serial_port->setParity(QSerialPort::NoParity);

        bopen = p_qt_serial_port->open(QIODevice::ReadWrite);

        p_qt_serial_port->clear();
    }

    return bopen;

}

void WicedSerialPort::close()
{
#ifdef Q_OS_WIN32
    p_win_serial_port->ClosePort();
#endif
    if(p_qt_serial_port)
        p_qt_serial_port->close();
}

bool WicedSerialPort::isOpen() const
{
    bool bopen = false;
#ifdef Q_OS_WIN32
    bopen = p_win_serial_port->IsOpened();
#endif
    if(p_qt_serial_port)
        bopen = p_qt_serial_port->isOpen();

    return bopen;
}

void WicedSerialPort::flush()
{
    if(p_qt_serial_port)
        p_qt_serial_port->flush();
}

bool WicedSerialPort::waitForBytesWritten(int iMilisec)
{
    if(p_qt_serial_port)
        return p_qt_serial_port->waitForBytesWritten(iMilisec);

    return true;
}

void WicedSerialPort::indicate_close()
{
#ifdef Q_OS_WIN32
    p_win_serial_port->indicate_close();
#endif

}

// Win32 COM port read/write
#ifdef Q_OS_WIN32

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
    memset(&m_OverlapRead, 0, sizeof(m_OverlapRead));
    memset(&m_OverlapWrite, 0, sizeof(m_OverlapWrite));

    m_handle = INVALID_HANDLE_VALUE;
    m_hShutdown = INVALID_HANDLE_VALUE;
    m_bClosing = FALSE;
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
    }
    Log ("Opened%s at speed: %u flow %s", str_port_name, baudRate, flow_control?"on":"off");
    m_bClosing = FALSE;
    m_hShutdown = CreateEvent(NULL, FALSE, FALSE, NULL);

    return m_handle != NULL && m_handle != INVALID_HANDLE_VALUE;
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
                m_bClosing = TRUE;
                dwTotalRead = 0;
                break;
            }

            HANDLE handles[2];
            handles[0] = m_OverlapRead.hEvent;
            handles[1] = m_hShutdown;

            dwRet = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
            if (dwRet == WAIT_OBJECT_0 + 1)
            {
                m_bClosing = TRUE;
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

#endif
