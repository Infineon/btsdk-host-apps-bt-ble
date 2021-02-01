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
 * Sample MCU application for A2DP Audio Source using WICED HCI protocol.
 */


#include "app_include.h"
extern "C"
{
#include "app_host.h"
}
#include <QStandardItemModel>

#define MAX_AUDIO_CONNECTION    2
const char * audio_freq_DualA2DP[] =
{
    "16 kHz",
    "32 kHz",
    "44.1 kHz",
    "48 kHz"
};

extern AudioFileWriter * pAudioFileWriter;

// Initialize app
void MainWindow::InitAudioSrc_DualA2DP()
{
    m_audio_connected_dual_a2dp= false;
    m_audio_started_dual_a2dp= false;
    m_audio_i2s_input_enable_dual_a2dp = false;
    m_audio_mp3_format_enable_dual_a2dp = false;
    m_bPortOpen = false;
    m_audio_connected_num_dual_a2dp = 0;
    memset(&m_uAudio, 0, sizeof(m_uAudio));

    // setup signals/slots
    connect(ui->btnStartAudio_DualA2DP, SIGNAL(clicked()), this, SLOT(onStartAudio_DualA2DP()));
    connect(ui->btnStopAudio_DualA2DP, SIGNAL(clicked()), this, SLOT(onStopAudio_DualA2DP()));
    connect(ui->btnConnectAudio_DualA2DP, SIGNAL(clicked()), this, SLOT(onConnectAudioSrc_DualA2DP()));
    connect(ui->btnDisconnectAudio_DualA2DP, SIGNAL(clicked()), this, SLOT(onDisconnectAudioSrc_DualA2DP()));
    connect(ui->btnFindAudioFile_DualA2DP, SIGNAL(clicked()), this, SLOT(onFindAudioFile_DualA2DP()));
    connect(ui->rbAudioSrcFile_DualA2DP, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcFile_DualA2DP(bool)));
    connect(ui->rbAudioSrcSine_DualA2DP, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcSine_DualA2DP(bool)));
    connect(ui->rbAudioSrcI2S_DualA2DP, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcI2S_DualA2DP(bool)));
    connect(ui->rbAudioFileFormatWav_DualA2DP, SIGNAL(clicked(bool)), this, SLOT(onAudioFileFormatWav_DualA2DP(bool)));
    connect(ui->rbAudioFileFormatMp3_DualA2DP, SIGNAL(clicked(bool)), this, SLOT(onAudioFileFormatMp3_DualA2DP(bool)));
    ui->edAudioFile_DualA2DP->setText( m_settings.value("AudioFileDualA2DP","").toString());
    ui->rbAudioSrcFile_DualA2DP->setChecked(m_settings.value("AudioSrcFileDualA2DP",true).toBool());
    ui->rbAudioSrcI2S_DualA2DP->setChecked(m_settings.value("AudioSrcI2SDualA2DP",true).toBool());
    ui->rbAudioSrcSine_DualA2DP->setChecked(!m_settings.value("AudioSrcFileDualA2DP",false).toBool() &&
                                   !m_settings.value("AudioSrcI2SDualA2DP",false).toBool());
    ui->rbAudioFileFormatWav_DualA2DP->setChecked(m_settings.value("AudioSrcFormatWavDualA2DP", true).toBool());
    ui->rbAudioFileFormatMp3_DualA2DP->setChecked(m_settings.value("AudioSrcFormatMp3DualA2DP", true).toBool());
    ui->cbSineFreq_DualA2DP->clear();
    for (int i = 0; i < 4; i++)
    {
        ui->cbSineFreq_DualA2DP->addItem(audio_freq_DualA2DP[i]);
    }

    QStandardItemModel *model =
      qobject_cast<QStandardItemModel *>(ui->cbSineFreq_DualA2DP->model());

    // disable not supported freq
    for (int i = 0; i < 2; i++)
    {
        QStandardItem *item = model->item(i);
        item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }

    setAudioSrcUI_DualA2DP();

    ui->rbAudioModeMono_DualA2DP->setChecked(false);
    ui->rbAudioModeMono_DualA2DP->setEnabled(false);
    ui->rbAudioModeSterio_DualA2DP->setChecked(true);
    ui->cbSineFreq_DualA2DP->setCurrentIndex(3); // 48KHz

}

// Connect to peer device
void MainWindow::onConnectAudioSrc_DualA2DP()
{
    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();

    if (pDev == NULL)
    {
        Log("No device selected");
        return;
    }

    if (ui->rbAudioSrcFile_DualA2DP->isChecked())
    {
        if (!InitializeAudioFile_DualA2DP())
        {
            Log("InitializeAudioFile_DualA2DP failed");
            return;
        }

    }

    wiced_hci_bt_audio_source_connect_data_t data;

    memcpy(data.bda, pDev->m_address, BDA_LEN);

    if (ui->rbAudioSrcFile_DualA2DP->isChecked())
    {
        data.audio_route = AUDIO_SRC_ROUTE_UART;
    }
    else if (ui->rbAudioSrcI2S_DualA2DP->isChecked())
    {
        data.audio_route = AUDIO_SRC_ROUTE_I2S;
    }
    else /* (ui->rbAudioSrcSine->isChecked()) */
    {
        data.audio_route = AUDIO_SRC_ROUTE_SINE_WAVE;
    }

    app_host_audio_src_connect(&data);
}

// Disconnect from peer devic
void MainWindow::onDisconnectAudioSrc_DualA2DP()
{
    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetConnectedAudioSrcDevice();
    if (NULL == pDev)
        return;

    app_host_audio_src_disconnect( pDev->m_address);
}


// Handle WICED HCI events
void MainWindow::onHandleWicedEventAudioSrc_DualA2DP(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if(!ui->tabDualA2DP->isEnabled())
    {
        return;
    }

    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_DEVICE:
        HandleDeviceEventsAudioSrc_DualA2DP(opcode, p_data, len);
        break;

    case HCI_CONTROL_GROUP_AUDIO:
        HandleA2DPEventsAudioSrc_DualA2DP(opcode, p_data, len);
        break;

    default:
        break;
    }
}

// Handle WICED HCI events for local device
void MainWindow::HandleDeviceEventsAudioSrc_DualA2DP(DWORD opcode, LPBYTE p_data, DWORD len)
{
    UNUSED(p_data);
    UNUSED(len);

    switch (opcode)
    {
        case HCI_CONTROL_EVENT_DEVICE_STARTED:
            m_audio_connected_dual_a2dp = false;
            m_audio_started_dual_a2dp = false;
            setAudioSrcUI_DualA2DP();
            break;
    }
}

// Handle WICED HCI events for AV
void MainWindow::HandleA2DPEventsAudioSrc_DualA2DP(DWORD opcode, BYTE *p_data, DWORD len)
{
    BYTE       bda[6];
    CBtDevice *device;
    UINT16     handle;

    app_host_audio_src_event(opcode, p_data, len);

    switch (opcode)
    {
    // AV Src connected
    case HCI_CONTROL_AUDIO_EVENT_CONNECTED:
    {
        int i = 0;
        for (i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        uint8_t data1 = p_data[i++];
        uint8_t data2 = p_data[i++];
        handle = data1 + (data2 << 8);

        // find device in the list with received address and save the connection handle
        if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
        {
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);
        }

        if (m_audio_connected_num_dual_a2dp < MAX_AUDIO_CONNECTION)
        {
            m_audio_connected_num_dual_a2dp++;
        }

        device->m_audio_handle = handle;
        device->m_conn_type |= CONNECTION_TYPE_AUDIO;

        SelectDevice(ui->cbDeviceList, bda);
        m_audio_connected_dual_a2dp = true;

        Log("Audio Connected, Handle: 0x%04x", handle);

        // if connect then device must be paired
        if (!device->m_paired)
            SetDevicePaired(device->m_address);

        setAudioSrcUI_DualA2DP();

        if (ui->rbAudioSrcFile_DualA2DP->isChecked())
        {
            // send audio data format to device
            uint8_t format;

            if (ui->rbAudioFileFormatMp3_DualA2DP->isChecked())
            {
                format = AUDIO_SRC_AUDIO_DATA_FORMAT_MP3;
            }
            else /* ui->rbAudioFileFormatWav_DualA2DP->isChecked() */
            {
                format = AUDIO_SRC_AUDIO_DATA_FORMAT_PCM;
            }

            app_host_audio_src_audio_data_format(handle, format);
        }
    }
        break;

    // AV Src disconnected
    case HCI_CONTROL_AUDIO_EVENT_DISCONNECTED:
    {
        handle = p_data[0] | (p_data[1] << 8);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_AUDIO, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_audio_handle == handle))
        {
            pDev->m_audio_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_AUDIO;
        }
        if (m_audio_connected_num_dual_a2dp > 0)
        {
            m_audio_connected_num_dual_a2dp--;
        }
        if (m_audio_connected_num_dual_a2dp == 0)
        {
            m_audio_connected_dual_a2dp = false;
        }
        m_audio_started_dual_a2dp = false;
        Log("Audio disconnected, Handle: 0x%04x", handle);
        setAudioSrcUI_DualA2DP();
        if(m_hidh_audio_started == false)
            EnableAppTraces();
    }
        break;

    // Streaming started
    case HCI_CONTROL_AUDIO_EVENT_STARTED:
        Log("Audio started");
        m_audio_started_dual_a2dp = true;
        DisableAppTraces();
        if (ui->rbAudioSrcFile_DualA2DP->isChecked() && (m_uAudio.m_pAudioData == NULL))
            InitializeAudioFile_DualA2DP();
        setAudioSrcUI_DualA2DP();
        break;

    // Streaming stopped
    case HCI_CONTROL_AUDIO_EVENT_STOPPED:
        Log("Audio stopped");
        m_audio_started_dual_a2dp = false;

        setAudioSrcUI_DualA2DP();
        break;

    // Embedded app requested audio data
    case HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA:
        if (m_uAudio.m_pAudioData != NULL)
            HandleA2DPAudioRequestEvent(p_data, len);
        break;

    case HCI_CONTROL_AUDIO_EVENT_COMMAND_COMPLETE:
        Log("Audio event command complete");
        break;

    case HCI_CONTROL_AUDIO_EVENT_COMMAND_STATUS:
        Log("Audio event command status");
        break;

    case HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED:
        Log("Audio event connection attempt failed (0x%lX)", opcode);
        break;

    case HCI_CONTROL_AUDIO_EVENT_SUPPORT_FEATURES:
    {
        uint8_t features = p_data[0];

        if (features & AUDIO_SRC_FEATURE_I2S_INPUT)
        {
            Log("Dual A2DP: Feature I2S INPUT enabled");
            m_audio_i2s_input_enable = true;
        }

        if (features & AUDIO_SRC_FEATURE_MP3_FORMAT)
        {
            Log("Dual A2DP: Feature MP3 FORMAT enabled");
            m_audio_mp3_format_enable = true;
        }

        setAudioSrcUI_DualA2DP();
    }
        break;

    default:
        Log("Rcvd cmd: %ld (0x%lX)", opcode, opcode);
        /* Unhandled */
        break;
    }

}


// The embedded device calls this method to request for audio data to AV SRC streaming.
// NOTE: Since this method is called directly from read thread
// there should be no UI manipulation from this method.
// See dm.cpp - Worker::read_serial_port_thread()
void MainWindow::HandleA2DPAudioRequestEvent_DualA2DP(BYTE * pu8Data, DWORD len)
{
    if(len < 3)
    {
        Log("HandleA2DPAudioRequestEvent bad length");
        return;
    }

    int bytes_per_packet = pu8Data[0] | (pu8Data[1] << 8);
    int num_packets = pu8Data[2];

    m_uAudio.m_BytesPerPacket = bytes_per_packet;

    m_audio_packets.lock();
    m_uAudio.m_PacketsToSend += num_packets;

    m_audio_packets.unlock();
    // m_audio_play_status_send_limit_counter will be reset on play status timeout.
    m_audio_play_status_send_limit_counter += num_packets;
#ifdef A2DP_STATS
    m_audio_total_sent_pkt_count += num_packets;
#endif
    if (m_audio_play_status_send_limit_counter > m_audio_play_status_send_limit_count)
    {
        m_audio_play_status_send_limit_counter -= m_audio_play_status_send_limit_count;
        // TODO : Update player status from other thread module
        //PlayerStatus();
    }


    if (pAudioFileWriter== NULL)
    {
        Log("thread not running\n");
        return;
    }

    if (!m_uAudio.m_pAudioData)
    {
        Log("Setup the audio file\n");
        return;
    }

    // signal the thread to send down data to embedded app
    audio_tx_wait.wakeAll();
}

// returns selected sampling frequency in UI
int MainWindow::GetSamplingFrequencyValue_DualA2DP(int index)
{
    switch(index)
    {
        case 0:
                return 16000;
        case 1:
                return 32000;
        case 2:
                return 441000;
        case 3:
        default:
                return 48000;
    }
}

// Start audio streaming
void MainWindow::onStartAudio_DualA2DP()
{
    CBtDevice * pDev = GetConnectedAudioSrcDevice();

    if (pDev == NULL)
        return;

    uint8_t sample_freq = ui->cbSineFreq_DualA2DP->currentIndex();
    uint8_t audio_mode = ui->rbAudioModeMono_DualA2DP->isChecked() ? AUDIO_SRC_CHANNEL_MODE_MONO : AUDIO_SRC_CHANNEL_MODE_STEREO;

    if (!m_audio_started_dual_a2dp)
    {
        if (ui->rbAudioSrcFile_DualA2DP->isChecked())
        {
            if (!InitializeAudioFile_DualA2DP())
            {
                return;
            }
        }
    }
#ifdef A2DP_STATS
    m_audio_total_sent_pkt_count = 0;
#endif
    m_audio_play_status_send_limit_counter = 0;

    // calculate no. of pkt count per playstatus timeout
    m_audio_play_status_send_limit_count =  ( m_tg_play_status_timeout_ms * GetSamplingFrequencyValue_DualA2DP(ui->cbSineFreq_DualA2DP->currentIndex()) ) / (128*1000);

    TGPlay();
    app_host_audio_src_start(pDev->m_address, sample_freq, audio_mode);
}

// Stop audio streaming
void MainWindow::onStopAudio_DualA2DP()
{
    if (!m_audio_started_dual_a2dp)
        return;

    CBtDevice * pDev = GetConnectedAudioSrcDevice();
    if (pDev == NULL)
        return;

    TGStop();
    app_host_audio_src_stop(pDev->m_address);
}

// Select audio file
void MainWindow::onFindAudioFile_DualA2DP()
{
    QString fileName;
    if (ui->rbAudioFileFormatWav_DualA2DP->isChecked())
    {
        fileName = QFileDialog::getOpenFileName(this, tr("Open Audio File"),
            "", tr("Audio Files (*.wav)"));
    }
    else
    {
        fileName = QFileDialog::getOpenFileName(this, tr("Open Audio File"),
            "", tr("Audio Files (*.mp3)"));
    }
    ui->edAudioFile_DualA2DP->setText(fileName);
    m_settings.setValue("AudioFileDualA2DP",fileName);
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedAudioSrcDevice_DualA2DP()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_audio_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as AV SRC");
        return NULL;
    }

    return pDev;
}

// Initialize audio stream from audio file
#define MAX_PATH          260
bool MainWindow::InitializeAudioFile_DualA2DP()
{
    char audioFile[MAX_PATH] = { 0 };
    strncpy(audioFile, ui->edAudioFile_DualA2DP->text().toStdString().c_str(), MAX_PATH-1);

    if (!ExecuteSetAudioFile_DualA2DP(audioFile)){
        return false;
    }

    return true;
}

void MainWindow::onAudioSrcSine_DualA2DP(bool)
{
    m_settings.setValue("AudioSrcFileDualA2DP",false);
    m_settings.setValue("AudioSrcI2SDualA2DP",false);
    ui->rbAudioFileFormatWav_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->rbAudioFileFormatMp3_DualA2DP->setEnabled(m_audio_mp3_format_enable && !m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->cbSineFreq_DualA2DP->setEnabled(!m_audio_started_dual_a2dp);
    ui->rbAudioModeMono_DualA2DP->setEnabled(true);
}

void MainWindow::onAudioSrcFile_DualA2DP(bool)
{
#ifdef __MACH__
    Log("!!! WARNING !!! Audio streaming from a file on MacOS is unreliable because of serial port issues. Use sine wave instead.");
    ui->rbAudioSrcSine->setChecked(true);
    return;
#endif
    m_settings.setValue("AudioSrcFileDualA2DP",true);
    ui->rbAudioFileFormatWav_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->rbAudioFileFormatMp3_DualA2DP->setEnabled(m_audio_mp3_format_enable && !m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->cbSineFreq_DualA2DP->setEnabled(!ui->rbAudioFileFormatMp3_DualA2DP->isChecked());
    ui->rbAudioModeMono_DualA2DP->setEnabled(!ui->rbAudioFileFormatMp3_DualA2DP->isChecked());
}

void MainWindow::onAudioSrcI2S_DualA2DP(bool)
{
    m_settings.setValue("AudioSrcI2SDualA2DP",true);
    ui->rbAudioFileFormatWav_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->rbAudioFileFormatMp3_DualA2DP->setEnabled(m_audio_mp3_format_enable && !m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->cbSineFreq_DualA2DP->setEnabled(!m_audio_started_dual_a2dp);
    ui->rbAudioModeMono_DualA2DP->setEnabled(true);
}

void MainWindow::onAudioFileFormatWav_DualA2DP(bool)
{
    m_audio_format = 0;
    m_settings.setValue("AudioSrcFormatWavDualA2DP", true);
    m_settings.setValue("AudioSrcFormatMp3DualA2DP", false);
    /* clear previous selected file */
    ui->edAudioFile_DualA2DP->clear();
    m_settings.setValue("AudioFileDualA2DP", "");
    /* set the selection of sample rate */
    ui->cbSineFreq_DualA2DP->setEnabled(!m_audio_started_dual_a2dp);
    ui->rbAudioModeMono_DualA2DP->setEnabled(true);
}

void MainWindow::onAudioFileFormatMp3_DualA2DP(bool)
{
    m_audio_format = 1;
    m_settings.setValue("AudioSrcFormatWavDualA2DP", false);
    m_settings.setValue("AudioSrcFormatMp3DualA2DP", true);
    /* clear previous selected file */
    ui->edAudioFile_DualA2DP->clear();
    m_settings.setValue("AudioFileDualA2DP", "");
    /* disable the selection of sample rate */
    ui->cbSineFreq_DualA2DP->setEnabled(false);
    ui->rbAudioModeMono_DualA2DP->setEnabled(false);
}

void MainWindow::setAudioSrcUI_DualA2DP()
{
    ui->btnConnectAudio_DualA2DP->setEnabled(!(m_audio_connected_num_dual_a2dp >= MAX_AUDIO_CONNECTION));
    ui->btnDisconnectAudio_DualA2DP->setEnabled(m_audio_connected_dual_a2dp);
    ui->btnStartAudio_DualA2DP->setEnabled(!m_audio_started_dual_a2dp & m_audio_connected_dual_a2dp);
    ui->btnStopAudio_DualA2DP->setEnabled(m_audio_started_dual_a2dp & m_audio_connected_dual_a2dp);
    ui->rbAudioSrcSine_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp);
    ui->rbAudioSrcFile_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp);
    ui->rbAudioSrcI2S_DualA2DP->setEnabled(m_audio_i2s_input_enable && !m_audio_connected_dual_a2dp);
    //ui->rbAudioModeMono_DualA2DP->setEnabled(!m_audio_started_dual_a2dp);
    ui->cbSineFreq_DualA2DP->setEnabled(!m_audio_started_dual_a2dp);
    ui->btnFindAudioFile_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp);
    ui->edAudioFile_DualA2DP->setEnabled(!m_audio_started_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->rbAudioFileFormatWav_DualA2DP->setEnabled(!m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    ui->rbAudioFileFormatMp3_DualA2DP->setEnabled(m_audio_mp3_format_enable && !m_audio_connected_dual_a2dp && ui->rbAudioSrcFile_DualA2DP->isChecked());
    if (ui->rbAudioSrcFile_DualA2DP->isChecked() && ui->rbAudioFileFormatMp3_DualA2DP->isChecked())
    {
        ui->cbSineFreq_DualA2DP->setEnabled(false);
    }
}

void MainWindow::closeEventAudioSrc_DualA2DP(QCloseEvent *event)
{
    UNUSED(event);
    onDisconnectAudioSrc();
    m_settings.setValue("AudioSrcFileDualA2DP",ui->rbAudioSrcFile_DualA2DP->isChecked());
    m_settings.setValue("AudioSrcI2SDualA2DP",ui->rbAudioSrcI2S_DualA2DP->isChecked());
}

// Get audio data chunks
BYTE* MainWindow::GetAudioDataDataChunk_DualA2DP(BYTE *pAudioData, DWORD dwAudioDataLen, DWORD *pdwDataLen)
{
    BYTE* pData = NULL;
    DWORD dwChunkLen;

    if (ui->rbAudioFileFormatWav_DualA2DP->isChecked())
    {
        do
        {
            // skip "RIFF", check RIFF chunk lenght and skip type ID (WAVE)
            if (dwAudioDataLen < 12)
                break;
            if (0 != memcmp(pAudioData, "RIFF", 4))
                break;
            dwChunkLen = LE_DWORD(pAudioData + 4);
            if (dwChunkLen + 8 > dwAudioDataLen)
                break;
            if (0 != memcmp(pAudioData + 8, "WAVE", 4))
                break;
            pAudioData += 12;
            dwAudioDataLen -= 12;
            // find data chunk and return it to caller
            while (dwAudioDataLen > 8)
            {
                dwChunkLen = LE_DWORD(pAudioData + 4);
                if (dwChunkLen + 8 > dwAudioDataLen)
                    break;
                if (0 == memcmp(pAudioData, "data", 4))
                {
                    pData = pAudioData + 8;
                    *pdwDataLen = dwChunkLen;
                    break;
                }
                pAudioData += 8 + dwChunkLen;
                dwAudioDataLen -= 8 + dwChunkLen;
            }
        } while (false);
    }
    else /* ui->rbAudioFileFormatMp3_DualA2DP->isChecked() */
    {
        pData =pAudioData;
        *pdwDataLen = dwAudioDataLen;
    }
    return pData;
}


BYTE * MainWindow::ExecuteSetAudioFile_DualA2DP(char *pcFileName)
{
    // Read file with audio data

    if (m_uAudio.m_pAudioData)
    {
        free(m_uAudio.m_pAudioData);

        m_uAudio.m_pAudioData = NULL;
        m_uAudio.m_pData = NULL;
        m_uAudio.m_dwAudioDataLen = 0;
        m_uAudio.m_dwChunkLen = 0;
        m_uAudio.m_dwAudioSent = 0;
    }

    if (NULL == (m_uAudio.m_pAudioData = ReadFile(pcFileName, &m_uAudio.m_dwAudioDataLen)))
    {
        Log("Could not open audio file %s", pcFileName);
        return NULL;
    }

    //get Data chunk pointer and length
    if (NULL == (m_uAudio.m_pData = GetAudioDataDataChunk_DualA2DP(m_uAudio.m_pAudioData, m_uAudio.m_dwAudioDataLen, &m_uAudio.m_dwChunkLen)))
    {
        Log("Error: could not get the data section of the audio file %s ", pcFileName);
        return NULL;
    }


    return m_uAudio.m_pData;
}


void MainWindow::on_btnHelpAVSRC_clicked_DualA2DP()
{
    onClear();
    Log("Audio Source help topic:");
    Log("");
    Log("Apps : watch");
    Log("");

    Log("Peer device - headset, speaker, car-kit, etc. containing Audio Sink service.");
    Log("- Connect:");
    Log("  Connect to the selected BR/EDR device to create an A2DP connection. The peer");
    Log("  selected device should be an audio sink capable device such as a headset,");
    Log("  speaker, or car-kit. Select the 'Media' and 'Mode' to stream before creating");
    Log("  the A2DP connection.");
    Log("- Disconnect:");
    Log("  Disconnect an existing A2DP connection from selected BR/EDR.");

    ScrollToTop();

}
