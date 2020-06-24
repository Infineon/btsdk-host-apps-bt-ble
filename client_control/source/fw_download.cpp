/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fw_download.h"
#include "hci_control_api.h"
#include <QtSerialPort/QSerialPort>
#include "wiced_types.h"

QT_USE_NAMESPACE

Q_DECLARE_METATYPE( CBtDevice* )

extern int as32BaudRate[];
extern bool openSerialPort(QSerialPort &serial);


// Download thread for downloading FW
DownloadThread::DownloadThread(QObject *parent)
    : QThread(parent)
{
    bStop = false;
    success = false;
    nPktCnt = nByteCnt = 0;
}

DownloadThread::~DownloadThread()
{
}

// Initiate a download
void DownloadThread::download(MainWindow *pParent, QString &fn)
{
    parent = pParent;   // mainwindow
    file = fn;          // .hcd filename
    bStop = false;
    success = false;
    nPktCnt = nByteCnt = 0;
    if (!isRunning())
        start();
}

// Run procedure of the thread
void DownloadThread::run()
{
    // execute the download process in a thread
    FILE * fHCD = fopen(file.toStdString().c_str(), "rb");
    if (NULL == fHCD)
    {
        QString str;

        str.sprintf("Error opening file %s",parent->ui->edPatchFile->text().toStdString().c_str());
        emit dlDone(str);   // notify mainwindow we are done
        return;
    }

    int download_baud = 0;

    if(parent->ui->btnUpgrdeTo3m->isChecked())
    {
        download_baud = 3000000;
    }
    else
    {
        download_baud = 115200;
    }

    // setup serial port
    QSerialPort serial;

    serial.setBaudRate(115200); // use 115200 for the HCI reset command
    SetupSerialPort(serial);

    // download will have to set a BDA for the device
    char sLocalBda[20]={0};
    strncpy(sLocalBda, parent->ui->edLocalBDA->text().toStdString().c_str(),sizeof(sLocalBda)-1);

    // support user input "444444444444" instead of "44:44:44:44:44:44"
    if (parent->ui->edLocalBDA->text().length() == 12)
    {
        sLocalBda[17] = 0;
        sLocalBda[16] = sLocalBda[11];
        sLocalBda[15] = sLocalBda[10];
        sLocalBda[14] = ':';
        sLocalBda[13] = sLocalBda[9];
        sLocalBda[12] = sLocalBda[8];
        sLocalBda[11] = ':';
        sLocalBda[10] = sLocalBda[7];
        sLocalBda[9] = sLocalBda[6];
        sLocalBda[8] = ':';
        sLocalBda[7] = sLocalBda[5];
        sLocalBda[6] = sLocalBda[4];
        sLocalBda[5] = ':';
        sLocalBda[4] = sLocalBda[3];
        sLocalBda[3] = sLocalBda[2];
        sLocalBda[2] = ':';
    }
    else if (parent->ui->edLocalBDA->text().length() != 17)
    {
        emit dlDone("Download failed: Specify valid BD address");   // notify mainwindow
        fclose(fHCD);
        return;
    }

    if (serial.isOpen())
        serial.close();

    if (!openSerialPort(serial))
    {
        emit dlDone("Download failed: Error opening serial port");
        fclose(fHCD);
        return;
    }
    QString sTrace;
    sTrace.sprintf("Comm port open: %s",serial.portName().toStdString().c_str());
    Log((char *)sTrace.toStdString().c_str());

    success = true;
    do
    {
        QThread::sleep(1);
        if (!send_hci_reset(serial))
        {
            // if fail retry at 4000000 or 3000000
            serial.close();
            serial.setBaudRate(download_baud);
            SetupSerialPort(serial);
            if (!openSerialPort(serial))
            {
                success = false;
                emit dlDone("Download failed: Error opening serial port");
                break;
            }
            if (!send_hci_reset(serial))
            {
                success = false;
                emit dlDone("Download failed: Error sending HCI reset");
                break;
            }
        }

        if(parent->ui->btnUpgrdeTo3m->isChecked())
        {
            if (!execute_change_baudrate(download_baud,serial))
            {
                success = false;
                emit dlDone("Download failed: Error updating baud rate");
                break;
            }

            Log("Set Baud Rate success");

            // set host baud rate
            serial.close();
            serial.setBaudRate(download_baud);
            sTrace.sprintf("Baudrate set to %d", download_baud);
            SetupSerialPort(serial);
            if (!openSerialPort(serial))
            {
                success = false;
                //qDebug("Error opening serial port: %d", (int)serial.error());
                emit dlDone("Download failed: Error opening serial port");
                break;
            }
        }

        if (!send_miniport(serial))
        {
            break;
        }

        ULONG   nAddr, nHCDRecSize;
        BYTE    arHCDDataBuffer[256];
        BOOL    bIsLaunch = FALSE;
        BYTE txbuf[261] = { 0x01, 0x4C, 0xFC, 0x00 };
        BYTE rxbuf[] = { 0x04, 0x0E, 0x04, 0x01, 0x4C, 0xFC, 0x00 };
        Log("Sending F/W file");
        nPktCnt = nByteCnt = 0;
        bool dl_ok = false;
        while (!bStop && ReadNextHCDRecord(fHCD, &nAddr, &nHCDRecSize, arHCDDataBuffer, &bIsLaunch))
        {
            txbuf[3] = (BYTE)(4 + nHCDRecSize);
            txbuf[4] = (nAddr & 0xff);
            txbuf[5] = (nAddr >> 8) & 0xff;
            txbuf[6] = (nAddr >> 16) & 0xff;
            txbuf[7] = (nAddr >> 24) & 0xff;
            memcpy(&txbuf[8], arHCDDataBuffer, nHCDRecSize);
            nByteCnt += 4 + 4 + nHCDRecSize;

            if (!SendRecvCmd(txbuf,4 + 4 + nHCDRecSize, rxbuf, sizeof(rxbuf),serial))
            {
                QString sTrace;
                sTrace.sprintf("Failed to send hcd portion at %x. Clean and rebuild application. Unplug and plug back board. Retry.", (uint32_t)nAddr);
                Log((char *)sTrace.toStdString().c_str());
                success = false;
                break;
            }

            if (bIsLaunch)
            {
                Log("Download file success");
                dl_ok = true;
                break;
            }
        }

        if (!dl_ok)
        {
            Log("Download file failed");
            success = false;
            break;
        }

        BYTE tx_cmd_launch[] = { 0x01, 0x4E, 0xFC, 0x04, 0xFF, 0xFF, 0xFF, 0xFF };
        BYTE rx_cmd_launch_rsp[] = { 0x04, 0x0E, 0x04, 0x01, 0x4E, 0xFC, 0x00 };

        Log("Launch RAM");
        if (!SendRecvCmd(tx_cmd_launch,sizeof(tx_cmd_launch),rx_cmd_launch_rsp,sizeof(rx_cmd_launch_rsp),serial,10000))
        {
            //Log("Launch RAM failed");
        }

        // Do not Reopen the HCI to send few comnands
        // Reopen the HCI at the end of the function (by simulating user click on the Open button)
#if 0
        QThread::sleep(3);

        // device is now in the application mode, send command to set BDADDR using WICED protocol
        // may need to re-open comm port at application baud rate if different from download baud rate

        if (app_baud != download_baud)
        {
            serial.close();
            serial.setBaudRate(app_baud);
            SetupSerialPort(serial);
            if (!serial.open(QIODevice::ReadWrite))
            {
                //qDebug("Error opening serial port: %d", (int)serial.error());
                Log("Download failed: Error opening serial port");
                break;
            }
        }

        int params[6];
        sscanf(sLocalBda, "%02x:%02x:%02x:%02x:%02x:%02x",
                &params[0], &params[1], &params[2], &params[3], &params[4], &params[5]);

        BYTE cmd[200], bda[6];
        for (int i = 0; i < 6; i++)
            bda[i] = (BYTE)params[5 - i];

        unsigned short command = HCI_CONTROL_COMMAND_SET_LOCAL_BDA;
        unsigned int len = 6;

        cmd[0] = HCI_WICED_PKT;
        cmd[1] = command & 0xff;
        cmd[2] = (command >> 8) & 0xff;
        cmd[3] = len & 0xff;
        cmd[4] = (len >> 8) & 0xff;

        Log("Set Local Bluetooth Device Address");
        memcpy(&cmd[5], bda, len);
        int written = serial.write((const char *)cmd, len + 5);
        success = serial.waitForBytesWritten(WAIT_TIMEOUT);

        // Write NV Ram data to device for BR/EDR and LE
        WriteNVRAMToDevice(false, serial);
        WriteNVRAMToDevice(true, serial);
#endif
    } while (0);

    qDebug("Download done, success: %d", success);

    serial.close();
    if (fHCD)
    {
        fclose(fHCD);
        fHCD = NULL;
    }
    if (success)
        emit dlDone("Finished download, success");
    else
        emit dlDone("Finished download, failed");

    // Open the COM Port (simulate User clicking the Open Button
    // The DEVICE_STARTED_EVT event will be received and the ClientControl app will send the
    // default configuration commands (NVRAM, Visibility, etc...)
    parent->ui->btnOpenPort->click();
}

void DownloadThread::Log(const char *s)
{
    //qDebug(s);
    emit dlProgress(new QString(s),0,0);
}

bool DownloadThread::ReadNextHCDRecord(FILE * fHCD, ULONG * nAddr, ULONG * nHCDRecSize, uint8_t * arHCDDataBuffer, BOOL * bIsLaunch)
{
    const   int HCD_LAUNCH_COMMAND = 0x4E;
    const   int HCD_WRITE_COMMAND = 0x4C;
    const   int HCD_COMMAND_BYTE2 = 0xFC;

    BYTE     arRecHeader[3];
    BYTE     byRecLen;
    BYTE     arAddress[4];

    *bIsLaunch = FALSE;

    if (fread(arRecHeader, 1, 3, fHCD) != 3)               // Unexpected EOF
        return false;

    byRecLen = arRecHeader[2];

    if ((byRecLen < 4) || (arRecHeader[1] != HCD_COMMAND_BYTE2) ||
        ((arRecHeader[0] != HCD_WRITE_COMMAND) && (arRecHeader[0] != HCD_LAUNCH_COMMAND)))
    {
        Log("Wrong HCD file format trying to read the command information");
        return FALSE;
    }

    if (fread(arAddress, sizeof(arAddress), 1, fHCD) != 1)      // Unexpected EOF
    {
        Log("Wrong HCD file format trying to read 32-bit address");
        return FALSE;
    }

    *nAddr = arAddress[0] + (arAddress[1] << 8) + (arAddress[2] << 16) + (arAddress[3] << 24);

    *bIsLaunch = (arRecHeader[0] == HCD_LAUNCH_COMMAND);

    *nHCDRecSize = byRecLen - 4;

    if (*nHCDRecSize > 0)
    {
        if (fread(arHCDDataBuffer, 1, *nHCDRecSize, fHCD) != *nHCDRecSize)   // Unexpected EOF
        {
            Log("Not enough HCD data bytes in record");
            return FALSE;
        }
    }

    return TRUE;
}

bool DownloadThread::SendRecvCmd(BYTE * txbuf,int txbuf_sz, BYTE * rxbuf, int rxbuf_sz, QSerialPort & serial, int ms_to)
{
    QByteArray req((const char*)txbuf,txbuf_sz);
    int nbytes = serial.write(req);
    UNUSED(nbytes);
    //int nbytes = serial.write((const char *)txbuf,txbuf_sz);

    if (serial.waitForBytesWritten(ms_to))
    {
        // read response
        if (serial.waitForReadyRead(ms_to))
        {
            QByteArray responseData = serial.read(rxbuf_sz);//readAll()
            while ((responseData.count() < rxbuf_sz) && serial.waitForReadyRead(20))
            {
                responseData += serial.read(rxbuf_sz);//serial.readAll();read(rxbuf_sz);//serial.readAll();
                if (responseData.count() >= rxbuf_sz)
                    break;
            }

            //qDebug("recv %d bytes", responseData.count());
            if (responseData.count() >= rxbuf_sz)
            {
                if (memcmp(responseData.data(),rxbuf,rxbuf_sz) == 0)
                {
                    return true;
                }
                else
                    Log("response incorrect");
            }
            else
            {
                char logMsg[256];
                sprintf(logMsg, "response too small: %d, expected %d", (responseData.count()), rxbuf_sz);
                Log(logMsg);
            }

        }
        else
        {
            Log("Wait read response timeout");
        }
    }
    else
    {
        Log("Wait write request timeout");
    }
    return false;
}

bool DownloadThread::execute_change_baudrate(int newBaudRate, QSerialPort & serial)
{
    /* set the current baudrare on Host UART */

    // serial port should already be open
    if (!serial.isOpen())
        return false;

    UINT8 hci_change_baudrate[] = {0x01, 0x18, 0xFC, 0x06, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA };
    UINT8 hci_change_baudrate_cmd_complete_event[] = { 0x04, 0x0E, 0x04, 0x01, 0x18, 0xFC, 0x00 };

    hci_change_baudrate[6] = newBaudRate & 0xff;
    hci_change_baudrate[7] = (newBaudRate >> 8) & 0xff;
    hci_change_baudrate[8] = (newBaudRate >> 16) & 0xff;
    hci_change_baudrate[9] = (newBaudRate >> 24) & 0xff;


    return SendRecvCmd(hci_change_baudrate, sizeof(hci_change_baudrate), hci_change_baudrate_cmd_complete_event, sizeof(hci_change_baudrate_cmd_complete_event),serial);
}

bool DownloadThread::send_miniport(QSerialPort & serial)
{
    // send mini driver download
    Log("Sending miniport download");
    BYTE mini_dl_cmd[] = { 0x01, 0x2E, 0xFC, 0x00 };
    BYTE mini_dl_reply[] = { 0x04, 0x0E, 0x04, 0x01, 0x2E, 0xFC, 0x00 };
    return SendRecvCmd(mini_dl_cmd,sizeof(mini_dl_cmd), mini_dl_reply, sizeof(mini_dl_reply),serial);
}

bool DownloadThread::send_hci_reset(QSerialPort & serial)
{
    Log("Sending HCI reset");
    BYTE hci_reset_cmd[] = { 0x01, 0x03, 0x0C, 0x00 };
    BYTE hci_reset_rsp[] = { 0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00 };
    return SendRecvCmd(hci_reset_cmd,sizeof(hci_reset_cmd), hci_reset_rsp, sizeof(hci_reset_rsp),serial);
}

void DownloadThread::SetupSerialPort(QSerialPort &serial)
{
    serial.setFlowControl(parent->ui->btnFlowCntrl_2->isChecked() ? QSerialPort::HardwareControl : QSerialPort::NoFlowControl);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    // Find the serial port ID from the index of serial port combo box
    serial.setPortName(parent->m_strComPortsIDs.at(parent->ui->cbCommport->currentIndex()));
}


// Restore NV RAM link keys to the device
bool DownloadThread::WriteNVRAMToDevice(bool bBLEDevice, QSerialPort & serial)
{
    QVariant var;
    CBtDevice * pDev = NULL;
    uint32_t len=0;

    bool bReturn = true;

    QComboBox *cb = parent->ui->cbDeviceList;
    if(bBLEDevice)
        cb = parent->ui->cbBLEDeviceList;

    for (int i = 0; i < cb->count(); i++)
    {
        var = cb->itemData(i);
        if (NULL == (pDev = var.value<CBtDevice *>()))
            continue;
        if (pDev->m_nvram_id == -1 || pDev->m_nvram.size() == 0)
            continue;

        BYTE cmd[300];

        unsigned short command = HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA;
        unsigned int lenCmd_HCI_WICED_PKT = 0;
        unsigned int lenCmd_PUSH_NVRAM_DATA = 0;

        if ( (len = pDev->m_nvram.size()) > sizeof(cmd)-2)
               len = sizeof(cmd)-2;

        lenCmd_PUSH_NVRAM_DATA = len + 2; //  2 = m_nvram_id

        lenCmd_HCI_WICED_PKT = 5 + lenCmd_PUSH_NVRAM_DATA; // 5 = HCI_WICED_PKT packet header

        cmd[0] = HCI_WICED_PKT;
        cmd[1] = command & 0xff;
        cmd[2] = (command >> 8) & 0xff;
        cmd[3] = lenCmd_PUSH_NVRAM_DATA & 0xff;
        cmd[4] = (lenCmd_PUSH_NVRAM_DATA >> 8) & 0xff;

        cmd[5] = (BYTE)pDev->m_nvram_id;
        cmd[6] = (BYTE)(pDev->m_nvram_id >> 8);

        memcpy(&cmd[7], pDev->m_nvram.constData(), len);

        int written = serial.write((const char *)cmd, lenCmd_HCI_WICED_PKT);
        UNUSED(written);

        bool ret = serial.waitForBytesWritten(WAIT_TIMEOUT);
        if(!ret)
            bReturn = ret;
    }

    if(cb->count())
    {
        QString sTrace;
        sTrace.sprintf("WriteNVRAMToDevice: %s : num devices: %d, type: %s",
                   bReturn ? "succeeded" : "failed", cb->count(), bBLEDevice ? "BLE" : "BR-EDR");

        Log((char *)sTrace.toStdString().c_str());
    }

    return bReturn;
}
