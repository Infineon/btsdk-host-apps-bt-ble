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
 * Sample MCU application for downloading firmware .hcd file on embedded device using WICED HCI protocol.
 */


#ifndef DOWNLOAD_H
#define DOWNLOAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QtSerialPort/QSerialPort>
#include "mainwindow.h"

#define WAIT_TIMEOUT    3000

class DownloadThread : public QThread
{
    Q_OBJECT

public:
    DownloadThread(QObject *parent = 0);
    ~DownloadThread();
    void run();
    void download(MainWindow *pParent, QString &fn);
    void stop()
    {
        bStop = true;
    }
    bool getSuccess()
    {
        return success;
    }

signals:
    void dlProgress(QString* msg,int pkt_cnt, int byte_cnt);
    void dlDone(const QString &s);

private:
    bool send_hci_reset(QSerialPort & serial);
    bool ReadNextHCDRecord(FILE * fHCD, ULONG * nAddr, ULONG * nHCDRecSize, uint8_t * arHCDDataBuffer, BOOL * bIsLaunch);
    bool SendRecvCmd(BYTE * txbuf,int txbuf_sz, BYTE * rxbuf, int rxbuf_sz,QSerialPort & serial, int ms_to=WAIT_TIMEOUT);
    bool send_miniport(QSerialPort & serial);
    bool execute_change_baudrate(int newBaudRate, QSerialPort & serial);
    void Log(const char *);
    void SetupSerialPort(QSerialPort &serial);
    bool WriteNVRAMToDevice(bool bBLEDevice, QSerialPort & serial);

    MainWindow * parent;
    QString file;
    bool bStop;
    bool success;
    int nPktCnt, nByteCnt;
};


#endif // DOWNLOAD_H
