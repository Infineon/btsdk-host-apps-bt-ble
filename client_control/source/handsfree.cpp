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
 * Sample MCU application for Hands-free profile using WICED HCI protocol.
 */


#include "app_include.h"
extern "C"
{
#include "app_host_hf.h"
}

static const char *hf_ag_response[] =
{
    "OK",
    "ERROR",
    "+CME ERROR:",
    "RING",
    "+VGS:",
    "+VGM:",
    "+CCWA:",
    "+CHLD:",
    "+CIND:",
    "+CLIP:",
    "+CIEV:",
    "+BINP:",
    "+BVRA:",
    "+BSIR:",
    "+CNUM:",
    "+BTRH:",
    "+COPS:",
    "+CLCC:",
    "+BIND:",
    "UNKNOWN AT"
};

const char *hf_ag_command[] =
{
    "+VGS",
    "+VGM",
    "A",
    "+BINP",
    "+BVRA",
    "+BLDN",
    "+CHLD",
    "+CHUP",
    "+CIND",
    "+CNUM",
    "D",
    "+NREC",
    "+VTS",
    "+BTRH",
    "+COPS",
    "+CMEE",
    "+CLCC",
    "+BIA",
    "+BIEV",
    ""
};

// Initialize app
void MainWindow::InitHF()
{
    m_audio_connection_active = false;
    m_mic_cur_pos = 8;
    m_speaker_cur_pos = 8;
    ui->horizontalSliderHFMic->setRange(0, 15);
    ui->horizontalSliderHFSpeaker->setRange(0, 15);
    ui->horizontalSliderHFMic->setSliderPosition(m_mic_cur_pos);
    ui->horizontalSliderHFSpeaker->setSliderPosition(m_speaker_cur_pos);
    ui->cbHFDTMF->setCurrentIndex(11);
    ui->cbHFHeldCalls->setCurrentIndex(0);
}

// User clicked button to connect HF
void MainWindow::on_btnConnectHF_clicked()
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

    if(pDev->m_hf_handle != NULL_HANDLE)
    {
        Log("HF already connected for selected device");
        return;
    }

    Log("Sending HFP Connect Command, BDA: %02x:%02x:%02x:%02x:%02x:%02x",
           pDev->m_address[0], pDev->m_address[1], pDev->m_address[2],
           pDev->m_address[3], pDev->m_address[4], pDev->m_address[5]);

    app_host_hf_connect(pDev->m_address);
}

// User clicked button to Disconenct HF
void MainWindow::on_btnDisconnectHF_clicked()
{
    CBtDevice * pDev = GetConnectedHFDevice();
    if (pDev == NULL)
        return;
    app_host_hf_disconnect(pDev->m_address);
}

// User clicked button to connect or disconnect audio
void MainWindow::on_btnHFConnectAudio_clicked()
{
    CBtDevice * pDev = GetConnectedHFDevice();
    if (pDev == NULL)
        return;

    if (!m_audio_connection_active)
    {
        app_host_hf_open_audio(pDev->m_address);
    }
    else
    {
        app_host_hf_close_audio(pDev->m_address);
    }

    Log("Sending Audio %s Command, bda: %02x:%02x:%02x:%02x:%02x:%02x",
        (m_audio_connection_active) ? "Disconnect" : "Connect",
        pDev->m_address[0], pDev->m_address[1], pDev->m_address[2],
        pDev->m_address[3], pDev->m_address[4], pDev->m_address[5]);
}

// hang up call
void MainWindow::on_btnHFHangup_clicked()
{
    CBtDevice * pDev = GetConnectedHFDevice();
    if (pDev == NULL)
        return;

    app_host_hf_hangup(pDev->m_address);
}

// answer call
void MainWindow::on_btnHFAnswer_clicked()
{
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_A, 0, NULL);
}

// redial
void MainWindow::on_btnHFRedial_clicked()
{
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_BLDN, 0, NULL);
}

// make call
void MainWindow::on_btnHFDial_clicked()
{
    char    atStr[201] = { 0 };

    memset(atStr, 0, sizeof(atStr));
    QString str = ui->lineEditHFDial->text();
    strncpy(atStr, str.toStdString().c_str(), 200);
    if (atStr[strlen(atStr)] != ';')
        atStr[strlen(atStr)] = ';';
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_D, 0, atStr);

}

// DTMF command
void MainWindow::on_btnHFDTMF_clicked()
{
    QString str = ui->cbHFDTMF->itemText(ui->cbHFDTMF->currentIndex());

    char    atStr[201] = { 0 };

    memset(atStr, 0, sizeof(atStr));
    strncpy(atStr, str.toStdString().c_str(), 200);


    char digit[2] = { 0, 0 };
    digit[0] = (char) atStr[0];
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_VTS, 0, digit);
}

// voice recognition command
void MainWindow::on_btnHFVoiceReco_clicked()
{
    static BYTE voice_recognition_enabled = FALSE;
    voice_recognition_enabled ^= 1;
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_BVRA, voice_recognition_enabled, NULL);
    ui->btnHFVoiceReco->setText((voice_recognition_enabled ? "Stop Voice Recognition" : "Start Voice Recognition"));

}

// Call held
void MainWindow::on_btnHFCallHeld_clicked()
{
    int chld_action = ui->cbHFHeldCalls->currentIndex();
    int call_id     = ui->cbHFCallID->currentIndex();
    int command = HCI_CONTROL_HF_AT_COMMAND_CHLD;

    if (chld_action == 0)
        return;

    if (chld_action > 5)
    {
        chld_action -= 5;
        command = HCI_CONTROL_HF_AT_COMMAND_BTRH;
    }

    if(call_id == 0)
    {
        SendAtCmd(command, chld_action - 1, NULL);
    }
    else
    {
        char str[2] = { 0, 0 };
        str[0] = (char) ('0'+ call_id);
        SendAtCmd(command, chld_action - 1, str);
    }

}

// NREC
void MainWindow::on_btnHFNREC_clicked()
{
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_NREC, 0, NULL);
}

// CNUM
void MainWindow::on_btnHFCNUM_clicked()
{
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_CNUM, 0, NULL);
}

// BINP
void MainWindow::on_btnHFBINP_clicked()
{
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_BINP, 1, NULL);
}

// Slider moved for microphone setting
void MainWindow::on_horizontalSliderHFMic_sliderMoved(int position)
{
    if (m_mic_cur_pos != position)
    {
        m_mic_cur_pos = position;
        SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_MIC, m_mic_cur_pos, NULL);
    }
}

// Slider moved for speaker setting
void MainWindow::on_horizontalSliderHFSpeaker_sliderMoved(int position)
{
    if (m_speaker_cur_pos != position)
    {
        m_speaker_cur_pos = position;
        SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_SPK, m_speaker_cur_pos, NULL);
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventHF(unsigned int opcode, unsigned char *p_data, unsigned int len)
{

    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_HF:
        HandleHFEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for HF
void MainWindow::HandleHFEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE    bda[6];

    UINT16  handle=0, features, num;
    const char   *pAtStr;

    app_host_hf_event(opcode, p_data, len);

    switch (opcode)
    {
    case HCI_CONTROL_HF_EVENT_OPEN:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd HCI_CONTROL_HF_EVENT_OPEN   BDA: %02x:%02x:%02x:%02x:%02x:%02x  Status: %u",
            handle, p_data[2], p_data[3], p_data[4], p_data[5], p_data[6], p_data[7], p_data[8]);
        Log(trace);

        if (p_data[8] == HCI_CONTROL_HF_STATUS_SUCCESS)
        {
            for (int i = 0; i < 6; i++)
                bda[5 - i] = p_data[2 + i];

            // find device in the list with received address and save the connection handle
            if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
                device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

            device->m_hf_handle = handle;
            device->m_conn_type |= CONNECTION_TYPE_HF;

            SelectDevice(ui->cbDeviceList, bda);

        }
    }
        break;
    case HCI_CONTROL_HF_EVENT_CLOSE:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x12 - HCI_CONTROL_HF_EVENT_CLOSE", handle);
        Log(trace);
        m_audio_connection_active = false;
        ui->btnHFConnectAudio->setText("Audio Connect");

        CBtDevice * pDev = FindInList(CONNECTION_TYPE_HF, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_hf_handle == handle))
        {
            pDev->m_hf_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_HF;
        }
    }
        break;
    case HCI_CONTROL_HF_EVENT_CONNECTED:
        handle   = p_data[0] | (p_data[1] << 8);
        features = p_data[2] | (p_data[3] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x13 - HCI_CONTROL_HF_EVENT_CONN  Features: 0x%04x", handle, features);
        Log(trace);
        break;
    case HCI_CONTROL_HF_EVENT_AUDIO_OPEN:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x14 - HCI_CONTROL_HF_EVENT_AUDIO_OPEN", handle);
        Log(trace);
        m_audio_connection_active = TRUE;
        ui->btnHFConnectAudio->setText("Audio Disconnect");
        break;
    case HCI_CONTROL_HF_EVENT_AUDIO_CLOSE:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x15 - HCI_CONTROL_HF_EVENT_AUDIO_CLOSE", handle);
        Log(trace);
        m_audio_connection_active = FALSE;
        ui->btnHFConnectAudio->setText("Audio Connect");
        break;

    case HCI_CONTROL_HF_EVENT_AUDIO_CONN_REQ:
        {
            UINT16 sco_index;
            STREAM_TO_BDADDR(bda, p_data);
            STREAM_TO_UINT16(sco_index, p_data);
            sprintf(trace, "[Handle: %u] Rcvd Event 0x16 - HCI_CONTROL_HF_EVENT_AUDIO_CONN_REQ BDA %02x:%02x:%02x:%02x:%02x:%02x SCO index %x",
                    handle, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], sco_index);
            Log(trace);

            //Accept the connection request
            app_host_hf_audio_accept_conn(sco_index);
        }
        break;

    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGS:
        {
            handle = p_data[0] | (p_data[1] << 8);
            num = p_data[2] | (p_data[3] << 8);
            Log("[Handle :%u] Event : HCI_CONTROL_HF_AT_EVENT_VGS Num : %d ",handle, num);
            if (m_speaker_cur_pos != num)
            {
                m_speaker_cur_pos = num;
                ui->horizontalSliderHFSpeaker->setSliderPosition(num);
            }
        }
        break;
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGM:
        {
            handle = p_data[0] | (p_data[1] << 8);
            num = p_data[2] | (p_data[3] << 8);
            Log("[Handle :%u] Event : HCI_CONTROL_HF_AT_EVENT_VGM Num : %d ",handle, num);
            if (m_mic_cur_pos != num)
            {
                m_mic_cur_pos = num;
                ui->horizontalSliderHFMic->setSliderPosition(num);
            }
        }
        break;

    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BIND:
        {
            handle = p_data[0] | (p_data[1] << 8);
            uint8_t ind = p_data[4] - '0';
            uint8_t ind_val = p_data[6] - '0';
            Log("[Handle :%u] Event : HCI_CONTROL_HF_AT_EVENT_BIND ind : %d value : %d",handle, ind, ind_val);

            switch(ind)
            {
                case 1:
                    ui->HFcheckBoxind1->setChecked(ind_val);
                    break;
                case 2:
                    ui->HFcheckBoxind2->setChecked(ind_val);
                    break;
                case 3:
                    ui->HFcheckBoxind3->setChecked(ind_val);
                    break;
                case 4:
                    ui->HFcheckBoxind4->setChecked(ind_val);
                    break;
                case 5:
                    ui->HFcheckBoxind5->setChecked(ind_val);
                    break;
                case 6:
                    ui->HFcheckBoxind6->setChecked(ind_val);
                    break;
                case 7:
                    ui->HFcheckBoxind7->setChecked(ind_val);
                    break;
            }
        }
        break;

    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_OK:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_ERROR:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_RING:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CCWA:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CHLD:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CIND:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLIP:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CIEV:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BINP:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BVRA:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BSIR:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CNUM:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BTRH:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_COPS:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CMEE:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLCC:
    case HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_UNAT:
        {
            char rsp[512] = { 0 };
            handle = p_data[0] | (p_data[1] << 8);
            pAtStr = hf_ag_response[opcode - HCI_CONTROL_HF_AT_EVENT_BASE];
            num = p_data[2] | (p_data[3] << 8);

            sprintf(trace, "[Handle: %u] Rcvd Event 0x%02x - AG Response: %s  Num: %u  Params: ", handle, (unsigned int)(opcode - HCI_CONTROL_HF_AT_EVENT_BASE), pAtStr, num);
            strncpy(rsp, (const char *)&p_data[4], len - 4);
            strcat(trace, rsp);
            Log(trace);
            break;
        }
    }
}

// Set AT command
void MainWindow::SendAtCmd(int nAtCmd, int num, char *atStr)
{
    CBtDevice * pDev = GetConnectedHFDevice();
    if (pDev == NULL)
        return;

    char trace[300];
    sprintf(trace, "Sending HFP AT Command: %u  %s Handle: %d  Num : %u  AT Param: %s",
        nAtCmd, hf_ag_command[nAtCmd], pDev->m_hf_handle, num, (NULL == atStr) ? "NULL" : atStr);

    Log(trace);

    app_host_hf_at_command (pDev->m_address, nAtCmd, num, atStr);
}


// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedHFDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_hf_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as HF");
        return NULL;
    }

    return pDev;
}

// Simulate button press for handsfree device.
// Imlementation is embedded device dependent
void MainWindow::on_btnHFBtnPress_clicked()
{
    CBtDevice * pDev = GetConnectedHFDevice();
    if (pDev == NULL)
        return;

    app_host_hf_button_press(pDev->m_address);
}

// Simulate long button press (press and hold) for handsfree device
// Imlementation is embedded device dependent
void MainWindow::on_btnHFLongBtnPress_clicked()
{
    CBtDevice * pDev = GetConnectedHFDevice();
    if (pDev == NULL)
        return;

    app_host_hf_long_button_press(pDev->m_address);
}

void MainWindow::on_btnHFActiveCalls_clicked()
{
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_CLCC, 0, NULL);
}

// Send BIEV command
void MainWindow::on_btnHFUpdate_ind_clicked()
{
    int indicator = ui->cbHFIndicator->currentIndex();
    int val = ui->lineEditHFInd_val->text().toInt();

    if( (indicator >= 1) && (indicator<= 7))
    {
        char str[4] = { 0, 0 };
        str[0] = (char) ('0'+ indicator);
        str[1] = (char) (',');
        str[2] = (char) ('0'+ val);
        SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_BIEV, 0, str);
    }
}

// Send BIA command
void MainWindow::on_btnHFBIA_clicked()
{
    char cmd[20];
    uint8_t cmd_len = 0;
    cmd[cmd_len++] = ui->HFcheckBoxind1->isChecked() + '0';
    cmd[cmd_len++] = ',';
    cmd[cmd_len++] = ui->HFcheckBoxind2->isChecked() + '0';
    cmd[cmd_len++] = ',';
    cmd[cmd_len++] = ui->HFcheckBoxind3->isChecked() + '0';
    cmd[cmd_len++] = ',';
    cmd[cmd_len++] = ui->HFcheckBoxind4->isChecked() + '0';
    cmd[cmd_len++] = ',';
    cmd[cmd_len++] = ui->HFcheckBoxind5->isChecked() + '0';
    cmd[cmd_len++] = ',';
    cmd[cmd_len++] = ui->HFcheckBoxind6->isChecked() + '0';
    cmd[cmd_len++] = ',';
    cmd[cmd_len++] = ui->HFcheckBoxind7->isChecked() + '0';
    cmd[cmd_len++] = '\0';
    SendAtCmd(HCI_CONTROL_HF_AT_COMMAND_BIA, 0, cmd);
}

void MainWindow::on_btnHelpHF_clicked()
{
    onClear();
    Log("Hands-free device help topic:");
    Log("");
    Log("Apps : hci_handsfree");
    Log("Peer device - phone or device supporting the Audio Gateway profile");
    Log("");
    Log("- Connect");
    Log("  Connect to a peer device supporting the AG profile");
    Log("- Disconnect");
    Log("  Disconnect with a peer device");
    Log("- Connect Audio");
    Log("  Open a SCO audio channel with a peer device");
    Log("- Hangup, Answer, Redial, etc.");
    Log("  These controls send the HF profile AT commands for various operations.");
    ScrollToTop();
}
