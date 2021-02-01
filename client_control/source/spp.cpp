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
 * Sample MCU application for implementing Serial Port profile using WICED HCI protocol.
 */


#include "app_include.h"

#define WICED_BT_RFCOMM_SUCCESS 0
#define MAX_TX_RETRY_ATTEMPTS   1
#define TX_RETRY_TIMEOUT        2 // in seconds

extern MainWindow *g_pMainWindow;

extern "C"
{
#include "app_host_spp.h"
}


// Initialize app
void MainWindow::InitSPP()
{
    m_spp_bytes_sent = 0;
    m_spp_total_to_send = 0;
    m_spp_tx_complete_result = 0;
    m_spp_receive_file = NULL;
    g_pMainWindow = this;
    m_thread_spp = NULL;
    m_worker_spp = NULL;
}

// Connect to peer with SPP connection
void MainWindow::on_btnSPConnect_clicked()
{
    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    if(pDev->m_spp_handle != NULL_HANDLE)
    {
        Log("SPP already connected for selected device");
        return;
    }

    app_host_spp_connect(pDev->m_address);
}

// Disconnect SPP connection with peer
void MainWindow::on_btnSPPDisconnect_clicked()
{
    CBtDevice * pDev = GetConnectedSPPDevice();
    if (pDev == NULL)
        return;

    app_host_spp_disconnect(pDev->m_address);
}

// send data/file to peer device
void MainWindow::on_btnSPPSend_clicked()
{
    CBtDevice * pDev = GetConnectedSPPDevice();
    if (pDev == NULL)
        return;
    m_spp_total_to_send = 0;
    m_spp_bytes_sent = 0;

    if (!ui->cbSPPSendFile->isChecked())
    {
        QString str = ui->lineEditSPPSend->text();
        app_host_spp_send(pDev->m_address, (uint8_t*)str.toStdString().c_str(), str.length());
    }
    else
    {
        ui->btnSPPSend->setDisabled(true);

        m_thread_spp = new QThread;
        m_worker_spp = new Worker();
        m_worker_spp->moveToThread(m_thread_spp);

        connect(m_thread_spp, SIGNAL(started()), m_worker_spp, SLOT(process_spp()));
        connect(m_worker_spp, SIGNAL(finished()), m_thread_spp, SLOT(quit()));
        connect(m_worker_spp, SIGNAL(finished()), m_worker_spp, SLOT(deleteLater()));
        connect(m_thread_spp, SIGNAL(finished()), m_thread_spp, SLOT(deleteLater()));
        connect(m_worker_spp, SIGNAL(finished()), this, SLOT(on_cbSPPThreadComplete()));

        m_thread_spp->start();
    }
}


// Open a file to send data
void MainWindow::on_btnSPPBrowseSend_clicked()
{
    QString file = QFileDialog::getOpenFileName(this, tr("Open File"),"","");

    ui->lineEditSPPSendFile->setText(file);
}

// open a file to receive data
void MainWindow::on_btnSPPBrowseReceive_clicked()
{
    QString file = QFileDialog::getSaveFileName(this, tr("Save File"),"","");

    ui->lineEditSPPReceiveFile->setText(file);

    on_cbSPPReceiveFile_clicked(true);
}

// send data from file instead of text box
void MainWindow::on_cbSPPSendFile_clicked(bool checked)
{
       UNUSED(checked);
}

// receive data to file instead of text box
void MainWindow::on_cbSPPReceiveFile_clicked(bool checked)
{
    UNUSED(checked);
    if (m_spp_receive_file){
        fclose(m_spp_receive_file);
        m_spp_receive_file = NULL;
    }

    if (ui->cbSPPReceiveFile->isChecked())
    {
        QString strfile = ui->lineEditSPPReceiveFile->text();


        m_spp_receive_file = fopen(strfile.toStdString().c_str(), "wb");

        if (!m_spp_receive_file){
            Log("Error: could not open spp receive file %s \n", strfile.toStdString().c_str());
        }
        else{
            Log("Opened spp receive file %s \n", strfile.toStdString().c_str());
        }
    }
    else
    {
        if (m_spp_receive_file){
            fclose(m_spp_receive_file);
            m_spp_receive_file = NULL;
        }
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventSPP(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    //app_host_spp_event(opcode, p_data, len);
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_SPP:
        HandleSPPEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for SPP
void MainWindow::HandleSPPEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE bda[6];
    UINT16  handle;
    static int ea_total = 0;
    static BYTE last_byte_received = 0xff;

    app_host_spp_event(opcode, p_data, len);
    switch (opcode)
    {
    case HCI_CONTROL_SPP_EVENT_CONNECTED:
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        // find device in the list with received address and save the connection handle
        if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

        handle = p_data[6] + (p_data[7] << 8);
        device->m_spp_handle = handle;
        device->m_conn_type |= CONNECTION_TYPE_SPP;

        SelectDevice(ui->cbDeviceList, bda);
        break;
    case HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND:
        Log("SPP Service not found");
        break;
    case HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED:
        Log("SPP Connection Failed");
        break;
    case HCI_CONTROL_SPP_EVENT_DISCONNECTED:
    {
        handle = p_data[0] | (p_data[1] << 8);
        Log("SPP disconnected, Handle: 0x%04x", handle);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_SPP, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_spp_handle == handle))
        {
            pDev->m_spp_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_SPP;
        }
    }
        break;
    case HCI_CONTROL_SPP_EVENT_TX_COMPLETE:
        handle = p_data[0] | (p_data[1] << 8);
        m_spp_tx_complete_result = p_data[2];

        if (!ui->cbSPPSendFile->isChecked())
        {
            sprintf(trace, "SPP tx complete handle:%d result:%d", handle, m_spp_tx_complete_result);
            Log(trace);
        }
        else
        {
            qDebug(trace, "SPP tx complete handle:%d result:%d %d of %d",
                handle, m_spp_tx_complete_result, m_spp_bytes_sent, m_spp_total_to_send);
        }
        spp_tx_wait.wakeAll();

        break;
    case HCI_CONTROL_SPP_EVENT_RX_DATA:

        handle = p_data[0] + (p_data[1] << 8);
        ea_total += (len - 2);
        if (len > 32)
        {
            if (p_data[2] != (BYTE)(last_byte_received + 1))
                sprintf(trace, "----SPP rx complete session id:%d len:%d total:%d %02x - %02x",
                    (int) handle, (int) len - 2, (int) ea_total, (int) p_data[2], (int) p_data[len - 1]);
            else
                sprintf(trace, "SPP rx complete session id:%d len:%d total:%d %02x - %02x",
                    (int) handle, (int) len - 2, (int) ea_total, (int) p_data[2], (int) p_data[len - 1]);
            last_byte_received = p_data[len - 1];

        }
        else
        {
            sprintf(trace, "SPP rx complete session id:%d len:%d total:%d ",
                (int) handle, (int) len - 2, (int) ea_total);
            for (DWORD i = 0; i < len - 2; i++)
                snprintf(&trace[strlen(trace)], (sizeof(trace)) / sizeof(char) - strlen(trace), "%02x ", (int) p_data[2 + i]);
        }

        // NOTE: Since this method is called directly from read thread
        // there should be no UI manipulation in this if block below or code above in this case statement.
        // See dm.cpp - Worker::read_serial_port_thread()
        if (m_spp_receive_file)
        {
            fwrite(&p_data[2], 1, len - 2, m_spp_receive_file);
        }
        else
        {
            qDebug("%s", trace);
            trace[0] = 0;
            for (DWORD i = 0; (i < len - 2) && (i < 100); i++)
                snprintf(&trace[strlen(trace)], sizeof(trace) / sizeof(char) - strlen(trace), "%02x ", p_data[2 + i]);

            ui->lineEditSPPreceive->setText(trace);
        }
        break;

    }
}

// Get connected device from BREDR combo box for SPP connection
CBtDevice* MainWindow::GetConnectedSPPDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_spp_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as SPP");
        return NULL;
    }

    return pDev;
}

// Send data from a file using a thread
DWORD MainWindow::SendFileThreadSPP()
{
    int retry_tx_attempts = 0;
    FILE *fp = NULL;
    char buf[1030] = { 0 };
    QString strfile = ui->lineEditSPPSendFile->text();

    fp = fopen(strfile.toStdString().c_str(), "rb");
    if (!fp)
    {
        Log("Failed to open file %s", strfile.toStdString().c_str());

        return 0;
    }

    CBtDevice * pDev = GetConnectedSPPDevice();
    if (pDev == NULL)
    {
        fclose(fp);
        return 0;
    }

    fseek(fp, 0, SEEK_END);
    m_spp_total_to_send = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    m_spp_bytes_sent = 0;

    int read_bytes;
    QMutex mutex;

    read_bytes = fread(buf, 1, HCI_CONTROL_SPP_MAX_TX_BUFFER, fp);

    while (read_bytes != 0)
    {
        mutex.lock();

        app_host_spp_send(pDev->m_address, (LPBYTE)buf, read_bytes);
        m_spp_bytes_sent += read_bytes;

        // wait for spp tx complete event
        spp_tx_wait.wait(&mutex);

        if ((m_spp_tx_complete_result != WICED_BT_RFCOMM_SUCCESS) ||
            (!ui->cbSPPSendFile->isChecked()))
        {
            Log("Got NAK");
            retry_tx_attempts++;
        }
        else
        {
            retry_tx_attempts = 0;
        }

        mutex.unlock();

        if(retry_tx_attempts > MAX_TX_RETRY_ATTEMPTS)
        {
            Log("Send failed, please retry");
            m_spp_total_to_send = 0;
            mutex.unlock();
            break;
        }
        else if((retry_tx_attempts !=0) && (retry_tx_attempts <= MAX_TX_RETRY_ATTEMPTS))
        {
            Log("Retrying last packet in %d seconds", TX_RETRY_TIMEOUT);
            m_thread_spp->sleep(TX_RETRY_TIMEOUT);
        }
        else
        {
            read_bytes = fread(buf, 1, HCI_CONTROL_SPP_MAX_TX_BUFFER, fp);
            retry_tx_attempts = 0;
        }
    }
    fclose(fp);

    return 0;
}

void MainWindow::on_cbSPPThreadComplete()
{
    Log("on_cbSPPThreadComplete");
    ui->btnSPPSend->setDisabled(false);

    m_thread_spp = NULL;
    m_worker_spp = NULL;
}

/* spp thread */
void Worker::process_spp()
{
    g_pMainWindow->SendFileThreadSPP();
    emit finished();
}


void MainWindow::on_btnHelpSPP_clicked()
{
    onClear();
    Log("Serial Port Profile help topic:");
    Log("");
    Log("Apps : spp_multi_port");
    Log("Peer device - PC, Phone, etc. supporting SPP profile");
    Log("");
    Log("- Connect");
    Log("  Connect to SPP server");
    Log("- Disconnect");
    Log("  Disconnect from SPP server");
    Log("- Send");
    Log("  Send characters typed in the edit control, or a file, to the peer device");
    Log("- Receive");
    Log("  Receive data in the edit control (first 50 bytes) or receive and save data");
    Log("  to a file");

    ScrollToTop();
}
