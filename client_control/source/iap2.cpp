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
 * Sample MCU application for implementing Apple iAP2 protocol using WICED HCI protocol.
 * The app sends/receives data using iAP2 protocol for Apple MFI devices.
 */

#include "app_include.h"

#define WICED_BT_RFCOMM_SUCCESS 0
//#define HCI_IAP2_MAX_TX_BUFFER                  (1000 - 12)  /* 12 bytes of the iAP2 overhead */
#define HCI_IAP2_MAX_TX_BUFFER                  (700 - 12)  /* 12 bytes of the iAP2 overhead */

extern MainWindow *g_pMainWindow;

#define DEBUG_SENDING_FILE
#ifdef DEBUG_SENDING_FILE
    DWORD   m_cnt_packetsent;
    DWORD   m_cnt_completed;
    DWORD   m_cnt_ack;
    DWORD   m_cnt_Nack;
#endif

// Initialize app
void MainWindow::InitiAP2()
{
    m_iap2_receive_file = NULL;

    m_iap2_bytes_sent = 0;
    m_iap2_total_to_send = 0;
    m_iap2_tx_complete_result = 0;

    // todo - design the usages of buttons
    ui->btniAPConnect->setVisible(false);
    ui->btniAPSDisconnect->setVisible(false);
    ui->btniAP2GenSign->setVisible(false);
    ui->btniAP2ReadCert->setVisible(false);

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
    UINT16 session_id = 0;
    CBtDevice * pDev = GetConnectediAP2Device();
    if (pDev == NULL)
    {
        return;
    }

    session_id = pDev->m_iap2_handle;
    pDev->m_iap2_handle = NULL_HANDLE;
    pDev->m_conn_type &= ~CONNECTION_TYPE_IAP2;

    cmd[commandBytes++] = session_id & 0xff;
    cmd[commandBytes++] = (session_id >> 8) & 0xff;

    Log("Sending iap2 Disconnect Command, session_id:%d", session_id);
    SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_DISCONNECT , cmd, commandBytes);
}

// send data/file to peer device
void MainWindow::on_btniAPSend_clicked()
{
    char buf[1030] = { 0 };
    UINT16 session_id;
    CBtDevice * pDev = GetConnectediAP2Device();
    if (pDev == NULL)
        return;

    session_id = pDev->m_iap2_handle;
    buf[0] = session_id & 0xff;
    buf[1] = (session_id >> 8) & 0xff;

#ifdef DEBUG_SENDING_FILE
    m_cnt_packetsent = 0;
    m_cnt_completed = 0;
    m_cnt_ack = 0;
    m_cnt_Nack = 0;
#endif

    if (!ui->cbiAP2SendFile->isChecked())
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
void MainWindow::on_btniAP2BrowseSend_clicked()
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
void MainWindow::on_cbiAP2SendFile_clicked()
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
    UINT16  session_id;
    static int ea_total = 0;
    static BYTE last_byte_received = 0xff;

    switch (identifier)
    {
    case HCI_CONTROL_IAP2_EVENT_CONNECTED:
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        session_id = p_data[6] + (p_data[7] << 8);

        sprintf(trace, "iAP2 connected %02x:%02x:%02x:%02x:%02x:%02x session_id:%d",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], session_id);
        Log(trace);

        // find device in the list with received address and save the connection session_id
        if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

        device->m_iap2_handle = session_id;
        device->m_conn_type |= CONNECTION_TYPE_IAP2;

        SelectDevice(ui->cbDeviceList, bda);

        // enable Send button
        ui->btniAPSend->setDisabled(false);
        break;

    case HCI_CONTROL_IAP2_EVENT_SERVICE_NOT_FOUND:
        Log("iAP2 Service not found");
        break;

    case HCI_CONTROL_IAP2_EVENT_CONNECTION_FAILED:
        Log("iAP2 Connection Failed");
        break;

    case HCI_CONTROL_IAP2_EVENT_DISCONNECTED:
    {
        session_id = p_data[0] | (p_data[1] << 8);
        Log("iAP2 disconnected, session_id:%d", session_id);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_IAP2, session_id, ui->cbDeviceList);
        if (pDev && (pDev->m_iap2_handle == session_id))
        {
            pDev->m_iap2_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_IAP2;
        }
    }
        break;

    case HCI_CONTROL_IAP2_EVENT_TX_COMPLETE:
        session_id = p_data[0] | (p_data[1] << 8);
        m_iap2_tx_complete_result = p_data[2];

#ifdef DEBUG_SENDING_FILE
        m_cnt_completed++;
        if (m_iap2_tx_complete_result)
            m_cnt_Nack++;
        else
            m_cnt_ack++;
#endif

        if (!ui->cbiAP2SendFile->isChecked())
        {
            sprintf(trace, "iAP2 tx complete session_id:%d result:%d", session_id, m_iap2_tx_complete_result);
            Log(trace);
        }
        else
        {
            // sending File
            // qDebug(trace, "iAP2 tx complete session_id:%s result:%d %d of %d",
            //    session_id, m_iap2_tx_complete_result, m_iap2_bytes_sent, m_iap2_total_to_send);
        }
        iap2_tx_wait.wakeAll();
        break;

    case HCI_CONTROL_IAP2_EVENT_RX_DATA:
        session_id = (p_data[0] << 8) + p_data[1];
        ea_total += (len - 2);
        if (len > 32)
        {
            if (p_data[2] != (BYTE)(last_byte_received + 1))
                sprintf(trace, "----IAP2 rx complete session_id:%d len:%d total:%d %02x - %02x",
                session_id, (uint32_t) len - 2, ea_total, p_data[2], p_data[len - 1]);
            else
                sprintf(trace, "IAP2 rx complete session_id:%d len:%d total:%d %02x - %02x",
                session_id, (uint32_t)len - 2, ea_total, p_data[2], p_data[len - 1]);
            last_byte_received = p_data[len - 1];
        }
        else
        {
            sprintf(trace, "IAP2 rx complete session_id:%d len:%d total:%d ",
                session_id, (uint32_t)len - 2, ea_total);
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

            ui->lineEditiAP2Receive->setText(trace);
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
#define MAX_TX_RETRY_ATTEMPTS 1 // 1: no retry, just exit.
#define TX_RETRY_TIMEOUT      2 // in seconds
DWORD MainWindow::SendFileThreadiAP2()
{
    int retry_tx_attempts = 0;
    FILE *fp = NULL;
    char buf[1030] = { 0 };
    QString strfile = ui->lineEditiAP2SendFile->text();

    fp = fopen(strfile.toStdString().c_str(), "rb");
    if (!fp)
    {
        Log("Failed to open file %s", strfile.toStdString().c_str());
        return 0;
    }

    UINT16 session_id;
    CBtDevice * pDev = GetConnectediAP2Device();
    if (pDev == NULL)
    {
        fclose(fp);
        return 0;
    }

    session_id = pDev->m_iap2_handle;

    buf[0] = session_id & 0xff;
    buf[1] = (session_id >> 8) & 0xff;

    fseek(fp, 0, SEEK_END);
    m_iap2_total_to_send = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    m_iap2_bytes_sent = 0;

    int read_bytes;
    QMutex mutex;

    read_bytes = fread(&buf[2], 1, HCI_IAP2_MAX_TX_BUFFER, fp);
    while (read_bytes != 0)
    {
        mutex.lock();

        SendWicedCommand(HCI_CONTROL_IAP2_COMMAND_DATA, (LPBYTE)buf, 2 + read_bytes);
        m_iap2_bytes_sent += read_bytes;

#ifdef DEBUG_SENDING_FILE
        m_cnt_packetsent++;
        Log("read_bytes:%d bytes_sent:%ld, packets_sent:%ld, session_id:%d", read_bytes, m_iap2_bytes_sent, m_cnt_packetsent, session_id);
#endif
        // wait for iap tx complete event
        iap2_tx_wait.wait(&mutex);

        if ((m_iap2_tx_complete_result != WICED_BT_RFCOMM_SUCCESS) || (!ui->cbiAP2SendFile->isChecked()))
        {
            Log("got Nak, result:%d", m_iap2_tx_complete_result);
            Log("Sending failed. Please retry");

            mutex.unlock();
            break;
        }

        mutex.unlock();

        Log("got Ack, sending next packet");
        read_bytes = fread(&buf[2], 1, HCI_IAP2_MAX_TX_BUFFER, fp);
    }
    fclose(fp);

    return 0;
}

void MainWindow::on_cbiAP2ThreadComplete()
{
    Log("on_cbiAP2ThreadComplete");
    ui->btniAPSend->setDisabled(false);

#ifdef DEBUG_SENDING_FILE
    Log("packet sent:%ld, completed:%ld, ACK:%ld, NAK:%ld\n", m_cnt_packetsent, m_cnt_completed, m_cnt_ack, m_cnt_Nack);
#endif
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
#if 0 // todo - design the usages of buttons
    Log("");
    Log("- Connect");
    Log("  Connect to an IAP2 server");
    Log("- Disconnect");
    Log("  Disconnect from an IAP2 server");
#endif
    Log("- Send");
    Log("  Send characters typed in the edit control, or a file, to the peer device");
    Log("- Receive");
    Log("  Receive data in the edit control (first 50 bytes) or receive and save data");
    Log("  to a file");

    ScrollToTop();
}
