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
 * Serial port read/write
 */

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdio.h>
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

// Serial port class for read/read to serial port.
// This class wraps QSerialPort for Linux
// and OS specific implementation for Windows and MAC
// as built in QT class has limitations
class WicedSerialPort
{
public:
    WicedSerialPort (bool hostmode = false);
    virtual ~WicedSerialPort ();

    virtual qint64 read(char *data, qint64 maxlen);
    virtual qint64 readline(char *data, qint64 maxlen);

    virtual qint64 write(char *data, qint64 len);

    virtual bool open(const char *str_port_name, qint32 baudRate, bool bFlowControl);
    virtual void close();

    virtual void indicate_close();

    virtual bool isOpen() const;

    virtual void flush();

    virtual bool waitForBytesWritten(int iMilisec);
    virtual int errorNum();
    virtual void handleReadyRead();
};

class WicedSerialPortHostmode : public WicedSerialPort
{
public:
WicedSerialPortHostmode ();
    ~WicedSerialPortHostmode (){}


    virtual qint64 read(char *data, qint64 maxlen);
    virtual qint64 write(char *data, qint64 len);

    virtual bool open(const char *str_port_name, qint32 baudRate, bool bFlowControl);
    virtual void close();

    virtual bool isOpen() const;

    virtual void flush();

    virtual bool waitForBytesWritten(int iMilisec);
    virtual int errorNum();
    virtual void handleReadyRead();

    int m_ClientSocket;
    bool OpenSocket();
};

#endif // SERIAL_PORT_H
