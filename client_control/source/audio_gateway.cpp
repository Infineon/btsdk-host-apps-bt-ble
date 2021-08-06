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
 * Sample MCU application for Audio Gateway (AG) using WICED HCI protocol.
 */

#include "app_include.h"

extern "C"
{
#include "app_host.h"
}

#define AT_MIC_VOLUME_EVT_STR    "AT+VGM="
#define AT_SPK_VOLUME_EVT_STR    "AT+VGS="
static bool skip_vol_update = FALSE;

// Initialize app
void MainWindow::InitAG()
{
    m_audio_connection_active = false;
    ui->comboBoxAGCallID_1->setCurrentIndex(0);
    ui->comboBoxAGCallID_2->setCurrentIndex(0);
    ui->comboBoxAGBattLevel->setCurrentIndex(5);
    ui->comboBoxAGCallSetup->setCurrentIndex(0);
    ui->comboBoxAGCallStatus->setCurrentIndex(0);
    ui->comboBoxAGHeldCall->setCurrentIndex(0);
    ui->comboBoxAGService->setCurrentIndex(1);
    ui->comboBoxAGSignal->setCurrentIndex(5);
}

void MainWindow::handle_ag_call_status_update()
{
    int count = 0;
    char cmd[20];
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
    {
        Log("Device not selected ");
        return;
    }

    cmd[count++] = '0'+ui->comboBoxAGCallStatus->currentIndex(); // call 0,1
    cmd[count++] =',';
    cmd[count++] = '0'+ui->comboBoxAGCallSetup->currentIndex(); // call setup
    cmd[count++] =',';
    cmd[count++] = '0'+ui->comboBoxAGHeldCall->currentIndex(); // call held
    cmd[count++] =',';
    cmd[count++] = ui->comboBoxAGService->currentIndex() + '0'; // service
    cmd[count++] = ',';
    cmd[count++] = ui->comboBoxAGBattLevel->currentIndex() + '0'; // battchg
    cmd[count++] = ',';
    cmd[count++] = ui->comboBoxAGSignal->currentIndex() + '0'; // signal
    cmd[count++] = ',';
    cmd[count++] = '0'; // roam

    app_host_ag_update_cind(cmd, count);
}

// Connect AG to peer device
void MainWindow::on_btnAGConnect_clicked()
{
    if (m_CommPort == NULL)
        return;

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    app_host_ag_connect(pDev->m_address);

}

void MainWindow::on_btnAGRing_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    app_host_ag_send_ring_cmd(pDev->m_address);
    app_host_ag_send_clip_cmd(pDev->m_address);
}

void MainWindow::on_btnAGCCWA_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    app_host_ag_send_ccwa_cmd(pDev->m_address);
}

// Disconnect AG from peer device
void MainWindow::on_btnAGDisconnect_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    app_host_ag_disconnect(pDev->m_address);
}

// Connect or disconnect audio connection with peer device
void MainWindow::on_btnAGAudioConnect_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    if (!m_audio_connection_active)
    {
        app_host_ag_audio_open(pDev->m_address);
        return;
    }
    else
    {
        app_host_ag_audio_close(pDev->m_address);
        return;
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventAG(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_AG:
        HandleAgEvents(opcode, p_data, len);
        break;
    }
}

int MainWindow::ag_get_volume(char *str, int len)
{
    int volume = 0;
    int index=0;
    char   trace[1024];
    if (len > 2)
    {
        sprintf(trace,"[ag_get_volume] Invalid Length %d",len);
        Log(trace);
        return -1;
    }
    while(len--)
    {
        volume = (volume*10) + (str[index]-'0');
        index++;
    }

    if (volume > 15)
    {
        sprintf(trace,"[ag_get_volume] Invalid volume %d",volume);
        Log(trace);
        return -1;
    }
    return volume;
}

// Handle WICED HCI events for AG
void MainWindow::HandleAgEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE    bda[6];

    UINT16  handle, features;


    app_host_ag_event(opcode, p_data, len);


    switch (opcode)
    {
    // AG connected with peer
    case HCI_CONTROL_AG_EVENT_OPEN:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd HCI_CONTROL_AG_EVENT_OPEN   BDA: %02x:%02x:%02x:%02x:%02x:%02x  Status: %u",
            handle, p_data[7], p_data[6], p_data[5], p_data[4], p_data[3], p_data[2], p_data[8]);
        Log(trace);

        if (p_data[8] == HCI_CONTROL_HF_STATUS_SUCCESS)
        {
            for (int i = 0; i < 6; i++)
                bda[5 - i] = p_data[2 + i];

            // find device in the list with received address and save the connection handle
            if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
                device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

            device->m_ag_handle = handle;
            device->m_conn_type |= CONNECTION_TYPE_AG;

            SelectDevice(ui->cbDeviceList, bda);

        }
        handle_ag_call_status_update();
    }
        break;

    // AG diconnected from peer
    case HCI_CONTROL_AG_EVENT_CLOSE:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x12 - HCI_CONTROL_AG_EVENT_CLOSE", handle);
        Log(trace);

        ui->btnAGAudioConnect->setText("Audio Connect");
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_AG, handle, ui->cbDeviceList);
        if (pDev)
        {
            pDev->m_hf_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_AG;
            m_audio_connection_active = false;
            ui->btnAGAudioConnect->setText("Audio Connect");
        }
    }

        break;
    case HCI_CONTROL_AG_EVENT_CONNECTED:
        handle   = p_data[0] | (p_data[1] << 8);
        features = p_data[2] | (p_data[3] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x13 - HCI_CONTROL_AG_EVENT_CONNECTED  Features: 0x%04x", handle, features);
        Log(trace);
        m_audio_connection_active = false;
        ui->btnAGAudioConnect->setText("Audio Connect");
        break;

    // AG audio connected with peer
    case HCI_CONTROL_AG_EVENT_AUDIO_OPEN:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u, wbs_supported: %d, wbs_used: %d] Rcvd Event 0x14 - HCI_CONTROL_AG_EVENT_AUDIO_OPEN",
                handle,
                p_data[2],
                p_data[3]);
        Log(trace);
        ui->btnAGAudioConnect->setText("Audio Disconnect");
        m_audio_connection_active = true;
        break;

    // AG audio diconnected from peer
    case HCI_CONTROL_AG_EVENT_AUDIO_CLOSE:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x15 - HCI_CONTROL_AG_EVENT_AUDIO_CLOSE", handle);
        Log(trace);
        ui->btnAGAudioConnect->setText("Audio Connect");
        m_audio_connection_active = false;
        break;
    case HCI_CONTROL_AG_EVENT_AT_CMD:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x15 - HCI_CONTROL_AG_EVENT_AT_CMD %s", handle, (char *)&p_data[2]);
        Log(trace);

        if (strncmp((char *)&p_data[2],AT_SPK_VOLUME_EVT_STR,strlen(AT_SPK_VOLUME_EVT_STR)) == 0)
        {
            int spk_evt_len = strlen(AT_SPK_VOLUME_EVT_STR);
            int volume = ag_get_volume((char *)&p_data[2+spk_evt_len],len-spk_evt_len-3);
            if (volume != -1)
            {
                if ( ui->comboBoxAGCallSpkVol->currentIndex() != volume )
                {
                    skip_vol_update = TRUE;
                    ui->comboBoxAGCallSpkVol->setCurrentIndex(volume);
                }
            }
        }
        if (strncmp((char *)&p_data[2],AT_MIC_VOLUME_EVT_STR,strlen(AT_MIC_VOLUME_EVT_STR)) == 0)
        {
            int mic_evt_len = strlen(AT_MIC_VOLUME_EVT_STR);
            int volume = ag_get_volume((char *)&p_data[2+mic_evt_len],len-mic_evt_len-3);
            if (volume != -1)
            {
                if ( ui->comboBoxAGCallMicVol->currentIndex() != volume )
                {
                    skip_vol_update = TRUE;
                    ui->comboBoxAGCallMicVol->setCurrentIndex(volume);
                }
            }
        }

        break;
    // CLCC Request from HF
    case HCI_CONTROL_AG_EVENT_CLCC_REQ:
        {
            CBtDevice * pDev = GetConnectedAGDevice();
            UINT8 call_list[2];
            if (pDev == NULL)
                return;

            call_list[0] = ui->comboBoxAGCallID_1->currentIndex();
            call_list[1] = ui->comboBoxAGCallID_2->currentIndex();
            handle   = p_data[0] | (p_data[1] << 8);
            sprintf(trace, "[Handle: %u] Rcvd Event - HCI_CONTROL_AG_EVENT_CLCC_REQ", handle);
            Log(trace);
            app_host_ag_send_clcc_response(pDev->m_address, call_list, 2);
            break;
        }
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedAGDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_ag_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as AG");
        return NULL;
    }

    return pDev;
}


void MainWindow::on_comboBoxAGCallID_1_currentIndexChanged(int index)
{
    handle_ag_call_status_update();
    UNUSED(index);
}

void MainWindow::on_comboBoxAGCallID_2_currentIndexChanged(int index)
{
    handle_ag_call_status_update();
    UNUSED(index);
}

void MainWindow::handle_ag_send_ciev_command(int id, int index)
{
    char cmd[4]={'0',',','0'};
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;
    cmd[0] = '0'+id;
    cmd[2] = '0'+index;

    app_host_ag_send_ciev(pDev->m_address,cmd, 3);
}
#define BTA_AG_CIND_INFO        "(\"call\",(0,1)),(\"callsetup\",(0-3)),(\"callheld\",(0-2)),(\"service\",(0,1),(\"battchg\",(0-5)),(\"signal\",(0-5)),(\"roam\",(0-1))"
// Call 1
// Callsetup 2
// Callheld 3
// Service 4
// battchg 5
// signal 6
// roam 7
void MainWindow::on_comboBoxAGHeldCall_currentIndexChanged(int index)
{
    handle_ag_send_ciev_command(3,index);
}

void MainWindow::on_comboBoxAGCallSetup_currentIndexChanged(int index)
{
    handle_ag_send_ciev_command(2,index);
}

void MainWindow::on_comboBoxAGCallStatus_currentIndexChanged(int index)
{
    handle_ag_send_ciev_command(1,index);
}

void MainWindow::on_comboBoxAGBattLevel_currentIndexChanged(int index)
{
    handle_ag_send_ciev_command(5,index);
}

void MainWindow::on_comboBoxAGService_currentIndexChanged(int index)
{
    handle_ag_send_ciev_command(4,index);
}

void MainWindow::on_comboBoxAGSignal_currentIndexChanged(int index)
{
    handle_ag_send_ciev_command(6,index);
}

void MainWindow::on_btnAGOK_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    app_host_ag_send_ok_cmd(pDev->m_address);
}

void MainWindow::on_btnAGError_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    app_host_ag_send_error_cmd(pDev->m_address);
}

void MainWindow::on_comboBoxAGCallSpkVol_currentIndexChanged(int index)
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;
    if (skip_vol_update == TRUE)
    {
        skip_vol_update = FALSE;
        return;
    }
    app_host_ag_send_spk_vol_cmd(pDev->m_address, index);
}

void MainWindow::on_comboBoxAGCallMicVol_currentIndexChanged(int index)
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    if (skip_vol_update == TRUE)
    {
        skip_vol_update = FALSE;
        return;
    }

    skip_vol_update = FALSE;
    app_host_ag_send_mic_vol_cmd(pDev->m_address, index);
}
