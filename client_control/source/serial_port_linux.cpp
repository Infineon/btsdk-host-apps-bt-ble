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
#include <termios.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#if defined(Q_OS_OSX)
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#endif
#include "wiced_types.h"

#if USE_QT_SERIAL_PORT
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

#else

static int fd_serial_port = -1;

// Serial port read/write class
// On linux platform use the Qt serial port class
// just a wrapper around QSerialPort class
WicedSerialPort::WicedSerialPort(bool hostmode)
{
    if (hostmode)
    {
        fd_serial_port = -1;
        return;
    }
}

WicedSerialPort::~WicedSerialPort()
{
    if(fd_serial_port >= 0)
        fd_serial_port = 0;
}

qint64 WicedSerialPort::read(char *data, qint64 maxlen)
{
    qint64 bytes_read = 0;
    if(fd_serial_port >= 0)
        bytes_read = ::read(fd_serial_port, data, (int)maxlen);
    return bytes_read;
}

qint64 WicedSerialPort::readline(char *data, qint64 maxlen)
{
    qint64 bytes_read = 0;
    if(fd_serial_port >= 0)
        bytes_read = ::read(fd_serial_port, data, maxlen);
    return bytes_read;
}

int WicedSerialPort::errorNum()
{
    if(fd_serial_port >= 0)
        return errno;
    return 0;
}

void WicedSerialPort::handleReadyRead()
{
   // unused
}


qint64 WicedSerialPort::write(char *data, qint64 len)
{
    qint64 written = 0;

    if(fd_serial_port >= 0)
    {
        written = ::write(fd_serial_port, data, len);
        if(written < 0)
        {
            close();
        }
    }
    return written;
}

bool WicedSerialPort::open(const char *str_port_name, qint32 baudRate, bool bFlowControl)
{
    struct termios tty;
    int baud = B115200;
    bool bopen = false;
#if defined(Q_OS_MACOS)
    int custom_speed = 0;
#endif

    if(fd_serial_port < 0)
    {
        if ((fd_serial_port = ::open(str_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
        {
          //  perror(tty);
            return bopen;
        }
        tcflush(fd_serial_port, TCIOFLUSH);

        if (tcgetattr(fd_serial_port, &tty) < 0) {
         //   printf("Error from tcgetattr: %s\n", strerror(errno));
            return bopen;
        }
        switch(baudRate)
        {
            case 115200:
                baud = B115200;
                break;
#if !defined(Q_OS_OSX)
            case 921600:
                baud = B921600;
                break;
            case 3000000:
                baud = B3000000;
                break;
#endif
            default:
#if defined(Q_OS_MACOS)
                custom_speed = baudRate;
#else
                return bopen;
#endif
                break;
        }
        cfsetospeed(&tty, (speed_t)baud);
        cfsetispeed(&tty, (speed_t)baud);

        tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;         /* 8-bit characters */
        tty.c_cflag &= ~PARENB;     /* no parity bit */
        tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */

        if(bFlowControl)
        {
            tty.c_cflag |= CRTSCTS;
        }
        else
        {
            tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */
        }

        /* setup for non-canonical mode */
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        /* fetch bytes as they become available */
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_serial_port, TCSANOW, &tty) != 0) {
          //  printf("Error from tcsetattr: %s\n", strerror(errno));
            return bopen;
        }
#if defined(Q_OS_MACOS)
        if(custom_speed != 0)
        {
            // Set 3000000 baud
            if ( ioctl( fd_serial_port, IOSSIOSPEED, &custom_speed ) == -1 )
            {
              //  printf("failed to set custom baud rate\n");
                return bopen;
            }
        }
#endif
        bopen = true;

    }

    if (bopen)
    {
        g_pMainWindow->CreateReadPortThread();
        g_pMainWindow->m_port_read_thread->start();
    }
    return bopen;
}

void WicedSerialPort::close()
{
    if(fd_serial_port >= 0)
        ::close(fd_serial_port);
    fd_serial_port = -1;
}

bool WicedSerialPort::isOpen() const
{
    return (fd_serial_port >= 0);
}

void WicedSerialPort::flush()
{
    if(fd_serial_port >= 0)
        tcflush(fd_serial_port, TCOFLUSH);
}

bool WicedSerialPort::waitForBytesWritten(int iMilisec)
{
    UNUSED(iMilisec);
    return (tcdrain(fd_serial_port) == 0);
}

void WicedSerialPort::indicate_close()
{
    close();
}

bool openSerialPort(QSerialPort & serial)
{
    UNUSED(serial);
    return false;
}
#endif

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

WicedSerialPortHostmode::WicedSerialPortHostmode(QString str_cmd_ip_addr, int iSpyInstance)
    : WicedSerialPort(true)
{
    str_ip_addr = std::move(str_cmd_ip_addr);
    SpyInstance = iSpyInstance;
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
    QByteArray ipAddr = str_ip_addr.toLocal8Bit();

    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr(ipAddr.data());
    service.sin_port = htons(SOCK_PORT_NUM + SpyInstance);
    memset(service.sin_zero, 0, sizeof(service.sin_zero));

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
    UNUSED(opcode);
    UNUSED(is_connected);
    UNUSED(port_handle);
    UNUSED(p_data);
    UNUSED(len);
}
void start_waiting(uint32_t timeout, uint16_t wait_event)
{
    UNUSED(timeout);
    UNUSED(wait_event);
}
BOOLEAN wiced_transport_send_buffer (int type, uint8_t* p_trans_buffer, uint16_t data_size)
{
    UNUSED(type);
    UNUSED(p_trans_buffer);
    UNUSED(data_size);
    return FALSE;
}
