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
 * Sample MCU application for implementing LE COC using WICED HCI protocol.
 */
#include "app_include.h"
extern "C"
{
#include "app_host.h"
void updateAdvBtn(bool sts);
void setPhy( void );
void recvData(char *p_data, unsigned int len);
void txComplete(void);
}

extern MainWindow *g_pMainWindow;

QSemaphore lecoc_buf_semaphore(2);

void MainWindow::on_lineEditLecocPsm_textChanged(const QString &arg1)
{
    app_host_le_coc_send_psm(arg1.toInt());
}

void MainWindow::on_btnLecocConnect_clicked()
{
    uint8_t bda[6];

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedLEDevice();

    if (NULL == pDev)
        return;

    memcpy(bda, pDev->m_address, BDA_LEN);
    app_host_le_coc_connect(bda);
}

void MainWindow::on_btnLecocDisconnect_clicked()
{
    uint8_t bda[6];

    CBtDevice * pDev =(CBtDevice *)GetSelectedLEDevice();
    /* There is no requirement of bdaddr for disconnection. All it needs cid.
    For now allow NULL bda to send for two reasons,
    1)To allow slave to initiate disconnect,
    2) because le coc app does not need bda on disconnection,
    but keeping prototypes with bda in case if we enhance to allow multiple connections */
    if (NULL == pDev)
    {
        memset(bda, 0, BDA_LEN);
    }
    else
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }

    app_host_le_coc_disconnect(bda);
}

void MainWindow::on_cbLECOCThreadComplete()
{
    Log("on_cbLECOCThreadComplete");
    ui->btnLecocSend->setDisabled(false);
}

// Send data from a file using a thread
DWORD MainWindow::SendFileThreadLECOC()
{
    FILE *fp = NULL;
    char buf[1030] = { 0 };
    QString strfile = ui->lineEditLecocSendFile->text();

    Log("Opening file %s to send..", strfile.toStdString().c_str());

    fp = fopen(strfile.toStdString().c_str(), "rb");
    if (!fp)
    {
        Log("Failed to open file %s", strfile.toStdString().c_str());

        return 0;
    }

    fseek(fp, 0, SEEK_END);
    m_lecoc_total_to_send = ftell(fp);

    Log("Total bytes to send : %ld ", m_lecoc_total_to_send);

    fseek(fp, 0, SEEK_SET);
    m_lecoc_bytes_sent = 0;

    int read_bytes;

     m_uart_tx_size = m_settings.value("UartTxSize",ui->lineEditLecocMtu->text().toInt()).toInt();

     Log("UartTxSize set to %ld", m_uart_tx_size);

    while ((read_bytes = fread(&buf[0], 1, m_uart_tx_size, fp)) != 0)
    {
        lecoc_buf_semaphore.acquire(1);

        app_host_le_coc_send_data((uint8_t*)buf, read_bytes);
        m_lecoc_bytes_sent += read_bytes;

        if ((!ui->cbLecocSendFile->isChecked()))
        {
            m_lecoc_total_to_send = 0;
            break;
        }
    }

    fclose(fp);

    return 0;
}

void updateAdvBtn(bool sts)
{
    if (!sts)
    {
        g_pMainWindow->m_advertisments_active = FALSE;
        g_pMainWindow->ui->pushButtonLecocStartAdv->setText("Start Adv");
    }
}

void setPhy( void )
{
    app_host_log("LE2M %d", g_pMainWindow->ui->radioButtonLecoc2mPhy->isChecked());
    app_host_le_coc_set_phy((uint16_t)g_pMainWindow->ui->radioButtonLecoc2mPhy->isChecked());
}

void recvData(char *p_data, unsigned int len)
{
    g_pMainWindow->recvDataFromDevice(p_data, len);
}

void txComplete(void)
{
    g_pMainWindow->rcvdTxCompleteFromDevice();
}

// Receive from from device
void MainWindow::recvDataFromDevice(char *p_data, unsigned int len)
{
    CHAR trace[128];

    memset(trace, 0, sizeof(trace));

    if (m_lecoc_receive_file)
    {
        fwrite(&p_data[0], len, 1, m_lecoc_receive_file);
    }
    else
    {
        for (DWORD i = 0; (i < len) && (i < 100); i++)
            sprintf(&trace[strlen(trace)], "%c", p_data[i]);

        ui->lineEditLecocReceive->setText(trace);
    }
}

void MainWindow::rcvdTxCompleteFromDevice( void )
{
    lecoc_buf_semaphore.release(1);
}

/* LECOC thread */
void Worker::process_lecoc()
{
    g_pMainWindow->SendFileThreadLECOC();
    emit finished();
}

void MainWindow::on_btnLecocSend_clicked()
{
    char buf[1030] = { 0 };

    if (!ui->cbLecocSendFile->isChecked())
    {
        QString str = ui->lineEditLecocSend->text();
        strncpy(&buf[0], str.toStdString().c_str(), sizeof(buf));

        app_host_le_coc_send_data((uint8_t*)buf, strlen(&buf[0]));
    }
    else
    {
        ui->btnLecocSend->setDisabled(true);

        QThread* thread = new QThread;
        Worker* worker = new Worker();
        worker->moveToThread(thread);

        connect(thread, SIGNAL(started()), worker, SLOT(process_lecoc()));
        connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
        connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
        connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
        connect(worker, SIGNAL(finished()), this, SLOT(on_cbLECOCThreadComplete()));

        thread->start();
    }
}

void MainWindow::on_pushButtonLecocStartAdv_clicked()
{
    if (!m_advertisments_active)
    {
        m_advertisments_active = TRUE;
        ui->pushButtonLecocStartAdv->setText("Stop Adv");
    }
    else
    {
        m_advertisments_active = FALSE;
        ui->pushButtonLecocStartAdv->setText("Start Adv");
    }

    app_host_le_coc_start_adv(m_advertisments_active);
}

void MainWindow::on_btnLecocBrowseSend_clicked()
{
    QString file = QFileDialog::getOpenFileName(this, tr("Open File"),"","");

    ui->lineEditLecocSendFile->setText(file);
}

void MainWindow::on_btnLecocBrowseReceive_clicked()
{
    QString file = QFileDialog::getOpenFileName(this, tr("Open File"),"","");

    ui->lineEditLecocReceiveFile->setText(file);
}

// send file check box clicked
void MainWindow::on_cbLecocSendFile_clicked()
{
    if (ui->cbLecocSendFile->isChecked())
    {
        QString strfile = ui->lineEditLecocSendFile->text();

        if (strfile.isEmpty()){
            Log("Select a file to send first. Checbox disabled.");
            ui->cbLecocSendFile->setChecked(false);
        }
    }
}

// receive data to file instead of text box
void MainWindow::on_cbLecocReceiveFile_clicked()
{
    if (m_lecoc_receive_file){
        fclose(m_lecoc_receive_file);
        m_lecoc_receive_file = NULL;
    }

    if (ui->cbLecocReceiveFile->isChecked())
    {
        QString strfile = ui->lineEditLecocReceiveFile->text();

        m_lecoc_receive_file = fopen(strfile.toStdString().c_str(), "wb");

        if (!m_lecoc_receive_file){
            Log("Error: could not open the file \"%s\" \n", strfile.toStdString().c_str());
            ui->cbLecocReceiveFile->setChecked(false);
        }
        else{
            Log("Opening file \"%s\" to receive data.. ", strfile.toStdString().c_str());
        }
    }
    else
    {
        if (m_lecoc_receive_file){
            fclose(m_lecoc_receive_file);
            m_lecoc_receive_file = NULL;
        }
    }
}

// Initialize app
void MainWindow::InitLecoc()
{
    m_lecoc_bytes_sent = 0;
    m_lecoc_total_to_send = 0;
    m_lecoc_tx_complete_result = 0;
    m_lecoc_receive_file = NULL;
    g_pMainWindow = this;
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventLECOC(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if (opcode == HCI_CONTROL_EVENT_DEVICE_STARTED)
    {
        ui->tabLECOC->setEnabled(true);
        app_host_le_coc_send_psm(ui->lineEditLecocPsm->text().toInt());
        app_host_le_coc_send_mtu(ui->lineEditLecocMtu->text().toInt());
        return;
    }

    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_LE_COC:
        app_host_le_coc_event(opcode, p_data, len);
        break;
    }
}
