/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
 * Sample MCU application for implementing Apple iAP2 protocol using WICED HCI protocol.
 * The app sends/receives data using iAP2 protocol for Apple MFI devices.
 */

#include "app_include.h"

#define WICED_BT_RFCOMM_SUCCESS 0
#define HCI_IAP2_MAX_TX_BUFFER                  (1000 - 12)  /* 12 bytes of the iAP2 overhead */

extern MainWindow *g_pMainWindow;


// Initialize app
void MainWindow::InitiAP2()
{
    m_iap2_receive_file = NULL;

    m_iap2_bytes_sent = 0;
    m_iap2_total_to_send = 0;

    m_iap2_tx_complete_result = 0;

    g_pMainWindow = this;
}

// Connect to peer with iAP2 connection
void MainWindow::on_btniAPConnect_clicked()
{
    BYTE    cmd[60];
    int     commandBytes = 0;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    if(pDev->m_iap2_handle != NULL_HANDLE)
    {
        Log("iAP2 already connected for selected device");
        return;
    }

    for (int i = 0; i < 6; i++)
        cmd[commandBytes++] = pDev->m_address[5 - i];

    Log("Sending iAP2 Connect Command, BDA: %02x:%02x:%02x:%02x:%02x:%02x",
        cmd[5], cmd[4], cmd[3], cmd[2], cmd[1], cmd[0]);

    SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_CONNECT, cmd, 6);
}


// Disconnect iAP2 connection with peer
void MainWindow::on_btniAPSDisconnect_clicked()
{
    BYTE   cmd[60];
    int    commandBytes = 0;
    UINT16 nHandle = 0;
    CBtDevice * pDev = GetConnectediAP2Device();
    if (pDev == NULL)
    {
        nHandle = pDev->m_iap2_handle;
        return;
    }

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    pDev->m_iap2_handle = NULL_HANDLE;
    pDev->m_conn_type &= ~CONNECTION_TYPE_IAP2;

    Log("Sending iap2 Disconnect Command, Handle: 0x%04x", nHandle);
    SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_DISCONNECT , cmd, commandBytes);
}

// send data/file to peer device
void MainWindow::on_btniAPSend_clicked()
{
    char buf[1030] = { 0 };
    UINT16 nHandle;
    CBtDevice * pDev = GetConnectediAP2Device();
    if (pDev == NULL)
        return;

    nHandle = pDev->m_iap2_handle;

    buf[0] = nHandle & 0xff;
    buf[1] = (nHandle >> 8) & 0xff;


    if (!ui->cbiAPPSendFile->isChecked())
    {
        QString str = ui->lineEditiAPSend->text();
        strncpy(&buf[2], str.toStdString().c_str(), sizeof(buf) - 2);
        m_iap2_total_to_send = 0;
        m_iap2_bytes_sent = 0;
        SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_DATA, (LPBYTE)buf, 2 + strlen(&buf[2]));
    }
    else
    {
        ui->btniAPSend->setDisabled(true);

        QThread* thread = new QThread;
        Worker* worker = new Worker();
        worker->moveToThread(thread);

        connect(thread, SIGNAL(started()), worker, SLOT(process_iap2()));
        connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
        connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
        connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
        connect(worker, SIGNAL(finished()), this, SLOT(on_cbiAP2ThreadComplete()));

        thread->start();
    }
}

// Open a file to send data
void MainWindow::on_btnSPPBrowseSend_2_clicked()
{
    QString file = QFileDialog::getOpenFileName(this, tr("Open File"),"","");

    ui->lineEditiAP2SendFile->setText(file);
}

// open a file to receive data
void MainWindow::on_btniAPBrowseReceive_clicked()
{
    QString file = QFileDialog::getSaveFileName(this, tr("Save File"),"","");

    ui->lineEditiAPReceiveFile->setText(file);

    on_cbiAPReceiveFile_clicked();
}

// send data from file instead of text box
void MainWindow::on_cbiAPPSendFile_clicked()
{

}

// receive data to file instead of text box
void MainWindow::on_cbiAPReceiveFile_clicked()
{
    if (m_iap2_receive_file){
        fclose(m_iap2_receive_file);
        m_iap2_receive_file = NULL;
    }

    if (ui->cbiAPReceiveFile->isChecked())
    {
        QString strfile = ui->lineEditiAPReceiveFile->text();


        m_iap2_receive_file = fopen(strfile.toStdString().c_str(), "wb");

        if (!m_iap2_receive_file){
            Log("Error: could not open iAP2 receive file %s \n", strfile.toStdString().c_str());
        }
        else{
            Log("Opened iAP2 receive file %s \n", strfile.toStdString().c_str());
        }
    }
    else
    {
        if (m_iap2_receive_file)
        {
            fclose(m_iap2_receive_file);
            m_iap2_receive_file = 0;
        }
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventiAP2(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_IAP2:
        HandleiAP2PEvents(opcode, p_data, len);
        break;
    }
}


// Handle WICED HCI events for iAP2
void MainWindow::HandleiAP2PEvents(DWORD identifier, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE bda[6];
    UINT16  handle;
    static int ea_total = 0;
    static BYTE last_byte_received = 0xff;


    switch (identifier)
    {
    case HCI_CONTROL_IAP2_EVENT_CONNECTED:
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        handle = p_data[6] + (p_data[7] << 8);

        sprintf(trace, "iAP2 connected %02x:%02x:%02x:%02x:%02x:%02x handle %04x",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], handle);
        Log(trace);

        // find device in the list with received address and save the connection handle
        if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

        device->m_iap2_handle = handle;
        device->m_conn_type |= CONNECTION_TYPE_IAP2;

        SelectDevice(ui->cbDeviceList, bda);
        break;
    case HCI_CONTROL_IAP2_EVENT_SERVICE_NOT_FOUND:
        Log("iAP2 Service not found");
        break;
    case HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED:
        Log("iAP2 Connection Failed");
        break;
    case HCI_CONTROL_IAP2_EVENT_DISCONNECTED:
    {
        handle = p_data[0] | (p_data[1] << 8);
        Log("iAP2 disconnected,  %04x", handle);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_IAP2, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_iap2_handle == handle))
        {
            pDev->m_iap2_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_IAP2;
        }
    }
        break;

    case HCI_CONTROL_IAP2_EVENT_TX_COMPLETE:
        handle = p_data[0] | (p_data[1] << 8);
        m_iap2_tx_complete_result = p_data[2];

        if (!ui->cbiAPPSendFile->isChecked())
        {
            sprintf(trace, "iAP2 tx complete handle:%d result:%d", handle, m_iap2_tx_complete_result);
            Log(trace);
        }
        else
        {
            qDebug(trace, "iAP2 tx complete handle:%d result:%d %d of %d",
                handle, m_iap2_tx_complete_result, m_iap2_bytes_sent, m_iap2_total_to_send);
        }
        iap2_tx_wait.wakeAll();

        break;

    case HCI_CONTROL_IAP2_EVENT_RX_DATA:
        handle = (p_data[0] << 8) + p_data[1];
        ea_total += (len - 2);
        if (len > 32)
        {
            if (p_data[2] != (BYTE)(last_byte_received + 1))
                sprintf(trace, "----IAP2 rx complete session id:%u len:%u total:%d %02x - %02x",
                handle, (uint32_t) len - 2, ea_total, p_data[2], p_data[len - 1]);
            else
                sprintf(trace, "IAP2 rx complete session id:%u len:%u total:%d %02x - %02x",
                handle, (uint32_t)len - 2, ea_total, p_data[2], p_data[len - 1]);
            last_byte_received = p_data[len - 1];
        }
        else
        {
            sprintf(trace, "IAP2 rx complete session id:%u len:%u total:%u ",
                handle, (uint32_t)len - 2, ea_total);
            for (DWORD i = 0; i < len - 2; i++)
                sprintf(&trace[strlen(trace)], "%02x ", p_data[2 + i]);
        }

        // NOTE: Since this method is called directly from read thread
        // there should be no UI manipulation in this if block below or code above in this case statement.
        // See dm.cpp - Worker::read_serial_port_thread()
        if (m_iap2_receive_file)
            fwrite(&p_data[2], 1, len - 2, m_iap2_receive_file);
        else
        {
            qDebug("%s", trace);
            trace[0] = 0;
            for (DWORD i = 0; (i < len - 2) && (i < 100); i++)
                sprintf(&trace[strlen(trace)], "%02x ", p_data[2 + i]);

            ui->lineEditSPPreceive_2->setText(trace);
        }
        break;

    case HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_INFO:
         sprintf(trace, "Rcvd Chip Info: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
             p_data[0], p_data[1], p_data[2], p_data[3], p_data[4], p_data[5], p_data[6], p_data[7]);
         Log(trace);

         sprintf(trace, "0x%02x", p_data[0]);
         ui->lineEditiAPVersion->setText(trace);

         sprintf(trace, "0x%02x", p_data[1]);
         ui->lineEditiAPFW->setText(trace);

         sprintf(trace, "0x%02x", p_data[2]);
         ui->lineEditiAPProtocolVerMajor->setText(trace);

         sprintf(trace, "0x%02x", p_data[3]);
         ui->lineEditiAPProtocolVerMinor->setText(trace);

         sprintf(trace, "0x%02x 0x%02x 0x%02x 0x%02x", p_data[4], p_data[5], p_data[6], p_data[7]);
         ui->lineEditiAPDeviceID->setText(trace);
         break;

    case HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_CERTIFICATE:
        sprintf(trace, "Rcvd Chip Certificate len:%d", (uint32_t)len);
        Log(trace);
        for (DWORD i = 0; i < len / 32; i++)
        {
            trace[0] = 0;
            for (DWORD j = 0; (j < 32) && (32 * i + j < len); j++)
                snprintf(&trace[strlen(trace)], sizeof(trace) / sizeof(char) - 3 * j, "%02x ", p_data[32 * i + j]);
            Log(trace);
        }
        break;
    case HCI_CONTROL_IAP2_EVENT_AUTH_CHIP_SIGNATURE:
        sprintf(trace, "Rcvd Chip Signature len:%d", (uint32_t)len);
        Log(trace);
        trace[0] = 0;
        for (DWORD i = 0; i < len / 32; i++)
        {
            trace[0] = 0;
            for (DWORD j = 0; (j < 32) && (32 * i + j < len); j++)
                snprintf(&trace[strlen(trace)], sizeof(trace) / sizeof(char) - 3 * j, "%02x ", p_data[32 * i + j]);
            Log(trace);
        }
        break;
    }

}


// Get connected device from BREDR combo box for iAP2 connection
CBtDevice* MainWindow::GetConnectediAP2Device()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_iap2_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as iAP2");
        return NULL;
    }

    return pDev;
}

// Read MFI auth chip certificate
void MainWindow::on_btniAP2ReadCert_clicked()
{
    SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_CERTIFICATE, NULL, 0);
}

// get auth chip signature
void MainWindow::on_btniAP2GenSign_clicked()
{
    BYTE digest[20] = { 0x5C, 0xD1, 0xA5, 0xBD, 0x55, 0x77, 0x92, 0xF9, 0x6B, 0x98, 0xA3, 0xD0, 0xCB, 0xE1, 0x16, 0x93, 0x4D, 0x3A, 0x8D, 0x78 };
//    BYTE digest[20] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14 };
    SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_SIGNATURE, digest, sizeof(digest));

}

// read auth chip info
void MainWindow::on_btniAPRead_clicked()
{
    SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_GET_AUTH_CHIP_INFO, NULL, 0);
}

// Send data from a file using a thread
DWORD MainWindow::SendFileThreadiAP2()
{
    FILE *fp = NULL;
    char buf[1030] = { 0 };
    QString strfile = ui->lineEditiAP2SendFile->text();

    fp = fopen(strfile.toStdString().c_str(), "rb");
    if (!fp)
    {
        Log("Failed to open file %s", strfile.toStdString().c_str());
        return 0;
    }

    UINT16 nHandle;
    CBtDevice * pDev = GetConnectediAP2Device();
    if (pDev == NULL)
        return 0;

    nHandle = pDev->m_iap2_handle;

    buf[0] = nHandle & 0xff;
    buf[1] = (nHandle >> 8) & 0xff;

    fseek(fp, 0, SEEK_END);
    m_iap2_total_to_send = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    m_iap2_bytes_sent = 0;

    int read_bytes;
    QMutex mutex;

    while ((read_bytes = fread(&buf[2], 1, HCI_IAP2_MAX_TX_BUFFER, fp)) != 0)
    {
        mutex.lock();

        SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_DATA,
                         (LPBYTE)buf, 2 + read_bytes);
        m_iap2_bytes_sent += read_bytes;

        if(iap2_tx_wait.wait(&mutex, 5000) == false)
        {
            Log("Wait failed");
            //break;
        }


        if ((m_iap2_tx_complete_result != WICED_BT_RFCOMM_SUCCESS) ||
            (!ui->cbiAPPSendFile->isChecked()))
        {
            m_iap2_total_to_send = 0;
            mutex.unlock();
            break;
        }
        mutex.unlock();
    }
    fclose(fp);

    return 0;
}

void MainWindow::on_cbiAP2ThreadComplete()
{\
    Log("on_cbiAP2ThreadComplete");
    ui->btniAPSend->setDisabled(false);
}


// iAP2 thread
void Worker::process_iap2()
{
    g_pMainWindow->SendFileThreadiAP2();
    emit finished();
}

void MainWindow::on_btnHelpIAP2_clicked()
{
    onClear();
    Log("iAP2 profile help topic:");
    Log("");
    Log("Apps : hci_iap2_spp (MFI licensees only)");
    Log("Peer device - iOS device");
    Log("");

    Log("- Connect");
    Log("  Connect to an IAP2 server");
    Log("- Disconnect");
    Log("  Disconnect from an IAP2 server");
    Log("- Send");
    Log("  Send characters typed in the edit control, or a file, to the peer device");
    Log("- Receive");
    Log("  Receive data in the edit control (first 50 bytes) or receive and save data");
    Log("  to a file");

    ScrollToTop();
}
