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
 * Sample MCU application for implemeting serial port read/write on Linux OS.
 */

#include <QObject>
#include "serial_port.h"
#include <QDebug>
#include "mainwindow.h"
#include "hci_control_api.h"
#include <stdio.h>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include "wiced_types.h"



static QSerialPort * p_qt_serial_port;
extern DWORD qtmin(DWORD len, DWORD bufLen);

// Serial port read/write class
// On linux platform use the Qt serial port class
// just a wrapper around QSerialPort class
WicedSerialPort::WicedSerialPort(bool hostmode)
{
    if (hostmode)
{
        p_qt_serial_port = NULL;
        return;
    }

    p_qt_serial_port = new QSerialPort(NULL);
}

WicedSerialPort::~WicedSerialPort()
{
    if(p_qt_serial_port)
        delete p_qt_serial_port;
}

qint64 WicedSerialPort::read(char *data, qint64 maxlen)
{
    qint64 read = 0;
    if(p_qt_serial_port)
        read = p_qt_serial_port->read(data, maxlen);

    return read;
}

qint64 WicedSerialPort::readline(char *data, qint64 maxlen)
{
    return read(data, maxlen);
}

int WicedSerialPort::errorNum()
{
    if(p_qt_serial_port)
        return p_qt_serial_port->error();
    return 0;
}

void WicedSerialPort::handleReadyRead()
{
    g_pMainWindow->serial_read_wait.wakeAll();
}

qint64 WicedSerialPort::write(char *data, qint64 len)
{
    qint64 written = 0;

    if(p_qt_serial_port)
    {
        written = p_qt_serial_port->write(data, len);
        p_qt_serial_port->flush();
    }
    //qDebug("write returns %d",static_cast<int>(written));
    return written;
}


bool WicedSerialPort::open(const char *str_port_name, qint32 baudRate, bool bFlowControl)
{
    bool bopen = false;

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
        if (bopen)
        {
            p_qt_serial_port->clear();
            g_pMainWindow->connect(p_qt_serial_port, SIGNAL(readyRead()),g_pMainWindow, SLOT(handleReadyRead()));
            g_pMainWindow->connect(p_qt_serial_port,SIGNAL(error(QSerialPort::SerialPortError)),g_pMainWindow,SLOT(serialPortError(QSerialPort::SerialPortError)));
            g_pMainWindow->CreateReadPortThread();
            g_pMainWindow->m_port_read_thread->start();
        }
    }

    return bopen;
}

void WicedSerialPort::close()
{
    if(p_qt_serial_port)
        p_qt_serial_port->close();
}

bool WicedSerialPort::isOpen() const
{
    bool bopen = false;
    if(p_qt_serial_port)
        bopen = p_qt_serial_port->isOpen();

    return bopen;
}

void WicedSerialPort::flush()
{
    p_qt_serial_port->flush();
}

bool WicedSerialPort::waitForBytesWritten(int iMilisec)
{
    return p_qt_serial_port->waitForBytesWritten(iMilisec);
}

void WicedSerialPort::indicate_close()
{
}

bool openSerialPort(QSerialPort & serial)
{
    return serial.open(QIODevice::ReadWrite);
}

///////////////////////////////

#define DEFAULT_BUFLEN 1024+6
#define SOCK_PORT_NUM	12012
#define INVALID_SOCKET  -1
#define SOCKET_ERROR    -1

extern "C"
{
extern void LogMsgX(const char *fmt_str, ... );
}

#define TRUE    true
#define FALSE   false

WicedSerialPortHostmode::WicedSerialPortHostmode()
    : WicedSerialPort(true)
{
    m_ClientSocket  = INVALID_SOCKET;
}

qint64 WicedSerialPortHostmode::read(char *data, qint64 maxlen)
{
    if (m_ClientSocket == INVALID_SOCKET)
    {
        if (!OpenSocket())
            return 0;
    }
    return recv(m_ClientSocket, data, maxlen, 0);
}

int WicedSerialPortHostmode::errorNum()
{
    return errno;
}

void WicedSerialPortHostmode::handleReadyRead()
{

}

qint64 WicedSerialPortHostmode::write(char *data, qint64 len)
{
    if (m_ClientSocket == INVALID_SOCKET)
    {
        OpenSocket();
    }

    if (m_ClientSocket != INVALID_SOCKET)
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

bool WicedSerialPortHostmode::OpenSocket()
{

//    if (INVALID_SOCKET == (m_ClientSocket = socket(AF_INET, SOCK_STREAM, 0)))
    if (INVALID_SOCKET == (m_ClientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
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
       ::close(m_ClientSocket);
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
    if (m_ClientSocket != INVALID_SOCKET)
    {
        shutdown(m_ClientSocket,SHUT_RDWR);
        ::close(m_ClientSocket);
        m_ClientSocket = INVALID_SOCKET;
    }
}

bool WicedSerialPortHostmode::isOpen() const
{
    return (m_ClientSocket != INVALID_SOCKET);
}

void WicedSerialPortHostmode::flush()
{

}

bool WicedSerialPortHostmode::waitForBytesWritten(int iMilisec)
{
    UNUSED(iMilisec);
    return true;
}

extern "C"
{
void script_handle_event(uint16_t opcode, uint32_t is_connected, uint32_t port_handle, uint8_t * p_data, uint32_t len);
void start_waiting(uint32_t timeout, uint16_t wait_event);
extern "C" BOOLEAN wiced_transport_send_buffer (int type, uint8_t* p_trans_buffer, uint16_t data_size);
}

void script_handle_event(uint16_t opcode, uint32_t is_connected, uint32_t port_handle, uint8_t * p_data, uint32_t len)
{
}
void start_waiting(uint32_t timeout, uint16_t wait_event)
{
}
BOOLEAN wiced_transport_send_buffer (int type, uint8_t* p_trans_buffer, uint16_t data_size)
{
    return FALSE;
}
