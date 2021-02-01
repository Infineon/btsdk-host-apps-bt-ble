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

const char * audio_freq[] =
{
    "16 kHz",
    "32 kHz",
    "44.1 kHz",
    "48 kHz"
};

AudioFileWriter * pAudioFileWriter=NULL;

// Initialize app
void MainWindow::InitAudioSrc()
{
    m_audio_connected = false;
    m_audio_started = false;
    m_audio_i2s_input_enable = false;
    m_audio_mp3_format_enable = false;
    m_bPortOpen = false;
    m_fpAudioFile = NULL;
    memset(&m_uAudio, 0, sizeof(m_uAudio));
    m_audio_format = (m_settings.value("AudioSrcFormatMp3", true).toBool()) ? 1 : 0;

    // setup signals/slots
    connect(ui->btnStartAudio, SIGNAL(clicked()), this, SLOT(onStartAudio()));
    connect(ui->btnStopAudio, SIGNAL(clicked()), this, SLOT(onStopAudio()));
    connect(ui->btnConnectAudio, SIGNAL(clicked()), this, SLOT(onConnectAudioSrc()));
    connect(ui->btnDisconnectAudio, SIGNAL(clicked()), this, SLOT(onDisconnectAudioSrc()));
    connect(ui->btnFindAudioFile, SIGNAL(clicked()), this, SLOT(onFindAudioFile()));
    connect(ui->rbAudioSrcFile, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcFile(bool)));
    connect(ui->rbAudioSrcSine, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcSine(bool)));
    connect(ui->rbAudioSrcI2S, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcI2S(bool)));
    connect(ui->rbAudioFileFormatWav, SIGNAL(clicked(bool)), this, SLOT(onAudioFileFormatWav(bool)));
    connect(ui->rbAudioFileFormatMp3, SIGNAL(clicked(bool)), this, SLOT(onAudioFileFormatMp3(bool)));
    ui->edAudioFile->setText( m_settings.value("AudioFile","").toString());
    ui->rbAudioSrcFile->setChecked(m_settings.value("AudioSrcFile",true).toBool());
    ui->rbAudioSrcI2S->setChecked(m_settings.value("AudioSrcI2S",true).toBool());
    ui->rbAudioSrcSine->setChecked(!m_settings.value("AudioSrcFile",false).toBool() &&
                                   !m_settings.value("AudioSrcI2S",false).toBool());
    ui->rbAudioFileFormatWav->setChecked(m_settings.value("AudioSrcFormatWav", true).toBool());
    ui->rbAudioFileFormatMp3->setChecked(m_settings.value("AudioSrcFormatMp3", true).toBool());
    ui->cbSineFreq->clear();

    for (int i = 0; i < 4; i++)
    {
        ui->cbSineFreq->addItem(audio_freq[i]);
    }

    QStandardItemModel *model =
      qobject_cast<QStandardItemModel *>(ui->cbSineFreq->model());

    // disable not supported freq
    for (int i = 0; i < 2; i++)
    {
        QStandardItem *item = model->item(i);
        item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }

    setAudioSrcUI();

    ui->rbAudioModeMono->setChecked(false);
    ui->rbAudioModeSterio->setChecked(true);
    ui->cbSineFreq->setCurrentIndex(3); // 48.1 KHz

    // Create a thread to send audio file data to embedded app
    pAudioFileWriter=new AudioFileWriter (this);
    pAudioFileWriter->start(QThread::TimeCriticalPriority);

}


// Connect to peer device
void MainWindow::onConnectAudioSrc()
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

    if (ui->rbAudioSrcFile->isChecked())
    {
        if (!InitializeAudioFile())
        {
            Log("InitializeAudioFile failed");
            return;
        }

    }

    wiced_hci_bt_audio_source_connect_data_t data;

    memcpy(data.bda, pDev->m_address, BDA_LEN);

    if (ui->rbAudioSrcFile->isChecked())
    {
        data.audio_route = AUDIO_SRC_ROUTE_UART;
    }
    else if (ui->rbAudioSrcI2S->isChecked())
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
void MainWindow::onDisconnectAudioSrc()
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
void MainWindow::onHandleWicedEventAudioSrc(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if(ui->tabDualA2DP->isEnabled())
    {
        return;
    }

    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_DEVICE:
        HandleDeviceEventsAudioSrc(opcode, p_data, len);
        break;

    case HCI_CONTROL_GROUP_AUDIO:
        HandleA2DPEventsAudioSrc(opcode, p_data, len);
        break;

    default:
        break;
    }
}

// Handle WICED HCI events for local device
void MainWindow::HandleDeviceEventsAudioSrc(DWORD opcode, LPBYTE p_data, DWORD len)
{
    UNUSED(p_data);
    UNUSED(len);

    switch (opcode)
    {
        case HCI_CONTROL_EVENT_DEVICE_STARTED:
            m_audio_connected = false;
            m_audio_started = false;
            setAudioSrcUI();
            break;
    }
}

// Handle WICED HCI events for AV
void MainWindow::HandleA2DPEventsAudioSrc(DWORD opcode, BYTE *p_data, DWORD len)
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
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

        device->m_audio_handle = handle;
        device->m_conn_type |= CONNECTION_TYPE_AUDIO;

        SelectDevice(ui->cbDeviceList, bda);
        m_audio_connected = true;

        Log("Audio Connected, Handle: 0x%04x", handle);

        // if connect then device must be paired
        if (!device->m_paired)
            SetDevicePaired(device->m_address);

        setAudioSrcUI();

        if (ui->rbAudioSrcFile->isChecked())
        {
            // send audio data format to device
            uint8_t format;

            if (ui->rbAudioFileFormatMp3->isChecked())
            {
                format = AUDIO_SRC_AUDIO_DATA_FORMAT_MP3;
            }
            else /* ui->rbAudioFileFormatWav->isChecked() */
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
        m_audio_connected = false;
        handle = p_data[0] | (p_data[1] << 8);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_AUDIO, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_audio_handle == handle))
        {
            pDev->m_audio_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_AUDIO;
        }
        m_audio_started = false;
        Log("Audio disconnected, Handle: 0x%04x", handle);
        setAudioSrcUI();
        if(m_hidh_audio_started == false)
            EnableAppTraces();
    }
        break;

    // Streaming started
    case HCI_CONTROL_AUDIO_EVENT_STARTED:
        Log("Audio started");
        m_audio_started = true;
        DisableAppTraces();
        if (ui->rbAudioSrcFile->isChecked() && (m_uAudio.m_pAudioData == NULL))
            InitializeAudioFile();
        setAudioSrcUI();
        break;

    // Streaming stopped
    case HCI_CONTROL_AUDIO_EVENT_STOPPED:
        Log("Audio stopped");
        m_audio_started = false;

        setAudioSrcUI();
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
            Log("AV Source: Feature I2S INPUT enabled");
            m_audio_i2s_input_enable = true;
        }

        if (features & AUDIO_SRC_FEATURE_MP3_FORMAT)
        {
            Log("AV Source: Feature MP3 FORMAT enabled");
            m_audio_mp3_format_enable = true;
        }

        setAudioSrcUI();
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
void MainWindow::HandleA2DPAudioRequestEvent(BYTE * pu8Data, DWORD len)
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


    if (pAudioFileWriter == NULL)
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
int MainWindow::GetSamplingFrequencyValue(int index)
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
void MainWindow::onStartAudio()
{
    CBtDevice * pDev = GetConnectedAudioSrcDevice();

    if (pDev == NULL)
        return;

    uint8_t sample_freq = ui->cbSineFreq->currentIndex();
    uint8_t audio_mode = ui->rbAudioModeMono->isChecked() ? AUDIO_SRC_CHANNEL_MODE_MONO : AUDIO_SRC_CHANNEL_MODE_STEREO;

    if (!m_audio_started)
    {
        if (ui->rbAudioSrcFile->isChecked())
        {
            if (!InitializeAudioFile())
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
    m_audio_play_status_send_limit_count =  ( m_tg_play_status_timeout_ms * GetSamplingFrequencyValue(ui->cbSineFreq->currentIndex()) ) / (128*1000);

    TGPlay();
    app_host_audio_src_start(pDev->m_address, sample_freq, audio_mode);
}

// Stop audio streaming
void MainWindow::onStopAudio()
{
    if (!m_audio_started)
        return;

    CBtDevice * pDev = GetConnectedAudioSrcDevice();
    if (pDev == NULL)
        return;

    TGStop();
    app_host_audio_src_stop(pDev->m_address);
}

// Select audio file
void MainWindow::onFindAudioFile()
{
    QString fileName;
    if (ui->rbAudioFileFormatWav->isChecked())
    {
        fileName = QFileDialog::getOpenFileName(this, tr("Open Audio File"),
            "", tr("Audio Files (*.wav)"));
    }
    else
    {
        fileName = QFileDialog::getOpenFileName(this, tr("Open Audio File"),
            "", tr("Audio Files (*.mp3)"));
    }
    ui->edAudioFile->setText(fileName);
    m_settings.setValue("AudioFile",fileName);
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedAudioSrcDevice()
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
bool MainWindow::InitializeAudioFile()
{
    char audioFile[MAX_PATH] = { 0 };
    strncpy(audioFile, ui->edAudioFile->text().toStdString().c_str(), MAX_PATH-1);

    if (!ExecuteSetAudioFile(audioFile)){
        return false;
    }

    return true;
}

void MainWindow::onAudioSrcSine(bool)
{
    m_settings.setValue("AudioSrcFile",false);
    m_settings.setValue("AudioSrcI2S",false);
    ui->rbAudioFileFormatWav->setEnabled(!m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->rbAudioFileFormatMp3->setEnabled(m_audio_mp3_format_enable && !m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->cbSineFreq->setEnabled(!m_audio_started);
    ui->rbAudioModeMono->setEnabled(true);
}

void MainWindow::onAudioSrcFile(bool)
{
#ifdef __MACH__
    Log("!!! WARNING !!! Audio streaming from a file on MacOS is unreliable because of serial port issues. Use sine wave instead.");
    ui->rbAudioSrcSine->setChecked(true);
    return;
#endif
    m_settings.setValue("AudioSrcFile",true);
    ui->rbAudioFileFormatWav->setEnabled(!m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->rbAudioFileFormatMp3->setEnabled(m_audio_mp3_format_enable && !m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->cbSineFreq->setEnabled(!ui->rbAudioFileFormatMp3->isChecked());
    ui->rbAudioModeMono->setEnabled(!ui->rbAudioFileFormatMp3->isChecked());
}

void MainWindow::onAudioSrcI2S(bool)
{
    m_settings.setValue("AudioSrcI2S",true);
    ui->rbAudioFileFormatWav->setEnabled(!m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->rbAudioFileFormatMp3->setEnabled(m_audio_mp3_format_enable && !m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->cbSineFreq->setEnabled(!m_audio_started);
    ui->rbAudioModeMono->setEnabled(true);
}

void MainWindow::onAudioFileFormatWav(bool)
{
    m_audio_format = 0;
    m_settings.setValue("AudioSrcFormatWav", true);
    m_settings.setValue("AudioSrcFormatMp3", false);
    /* clear previous selected file */
    ui->edAudioFile->clear();
    m_settings.setValue("AudioFile", "");
    /* set the selection of sample rate */
    ui->cbSineFreq->setEnabled(!m_audio_started);
    ui->rbAudioModeMono->setEnabled(true);
}

void MainWindow::onAudioFileFormatMp3(bool)
{
    m_audio_format = 1;
    m_settings.setValue("AudioSrcFormatWav", false);
    m_settings.setValue("AudioSrcFormatMp3", true);
    /* clear previous selected file */
    ui->edAudioFile->clear();
    m_settings.setValue("AudioFile", "");
    /* disable the selection of sample rate */
    ui->cbSineFreq->setEnabled(false);
    ui->rbAudioModeMono->setEnabled(false);
}

void MainWindow::setAudioSrcUI()
{
    ui->btnConnectAudio->setEnabled(!m_audio_connected );
    ui->btnDisconnectAudio->setEnabled(m_audio_connected );
    ui->btnStartAudio->setEnabled(!m_audio_started & m_audio_connected );
    ui->btnStopAudio->setEnabled(m_audio_started & m_audio_connected );
    ui->rbAudioSrcSine->setEnabled(!m_audio_connected);
    ui->rbAudioSrcFile->setEnabled(!m_audio_connected);
    ui->rbAudioSrcI2S->setEnabled(m_audio_i2s_input_enable && !m_audio_connected);
    ui->rbAudioModeMono->setEnabled(!m_audio_started);
    ui->cbSineFreq->setEnabled(!m_audio_started);
    ui->btnFindAudioFile->setEnabled(!m_audio_connected);
    ui->edAudioFile->setEnabled(!m_audio_started && ui->rbAudioSrcFile->isChecked());
    ui->rbAudioFileFormatWav->setEnabled(!m_audio_connected && ui->rbAudioSrcFile->isChecked());
    ui->rbAudioFileFormatMp3->setEnabled(m_audio_mp3_format_enable && !m_audio_connected && ui->rbAudioSrcFile->isChecked());
    if (ui->rbAudioSrcFile->isChecked() && ui->rbAudioFileFormatMp3->isChecked())
    {
        ui->cbSineFreq->setEnabled(false);
        ui->rbAudioModeMono->setEnabled(false);
    }
}

void MainWindow::closeEventAudioSrc(QCloseEvent *event)
{
    UNUSED(event);
    onDisconnectAudioSrc();
    m_settings.setValue("AudioSrcFile",ui->rbAudioSrcFile->isChecked());
    m_settings.setValue("AudioSrcI2S",ui->rbAudioSrcI2S->isChecked());
}

// Read audio file
BYTE* MainWindow::ReadFile(const char* FilePathName, DWORD *pdwAudioDataLen)
    {
    BYTE* res = NULL;
    FILE *fp = NULL;

    if (NULL == (fp = fopen(FilePathName, "rb")))
        qDebug("ReadFile: fopen failed. file=%s\n", FilePathName);
    else
    {
        fseek(fp, 0, SEEK_END);
        long size = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        if(size > 0)
        {
            res = (BYTE*)malloc(size);
            if (res)
            {
                if (fread(res, 1, size, fp) != (size_t)size)
                {
                    free(res);
                    res = NULL;
                }
                else
                    *pdwAudioDataLen = size;
            }
        }
        fclose(fp);
    }
    return res;
}

// Get audio data chunks
BYTE* MainWindow::GetAudioDataDataChunk(BYTE *pAudioData, DWORD dwAudioDataLen, DWORD *pdwDataLen)
{
    BYTE* pData = NULL;
    DWORD dwChunkLen;

    if (ui->rbAudioFileFormatWav->isChecked())
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
    else /* ui->rbAudioFileFormatMp3->isChecked() */
    {
        pData =pAudioData;
        *pdwDataLen = dwAudioDataLen;
    }
    return pData;
}


BYTE * MainWindow::ExecuteSetAudioFile(char *pcFileName)
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
    if (NULL == (m_uAudio.m_pData = GetAudioDataDataChunk(m_uAudio.m_pAudioData, m_uAudio.m_dwAudioDataLen, &m_uAudio.m_dwChunkLen)))
    {
        Log("Error: could not get the data section of the audio file %s ", pcFileName);
        return NULL;
    }


    return m_uAudio.m_pData;
}

/******************************************************************/
// thread for reading Audio file
AudioFileWriter::AudioFileWriter(MainWindow * pParent) : QThread()
{
    m_pParent = pParent;
    moveToThread(this);
}


uint8_t *gu8AudioBuffer = NULL;
int gs32AudioBufferSize = 0;

// Send audio packets to serial port
void AudioFileWriter::SendNextData(hci_audio_sample_t * puHci, int bytesPerPacket)
{
    int remaining = puHci->m_dwChunkLen - puHci->m_dwAudioSent;

    BYTE au8Hdr[5];
    UINT16 hciCmd;


    if (m_pParent->m_audio_format == 1)
    {
        hciCmd = HCI_CONTROL_AUDIO_DATA_MP3;
    }
    else /* m_pParent->ui->rbAudioFileFormatWav->isChecked() */
    {
        hciCmd = HCI_CONTROL_AUDIO_DATA;
    }

    au8Hdr[0] = HCI_WICED_PKT;
    au8Hdr[1] = (BYTE)(hciCmd & 0xff);
    au8Hdr[2] = (BYTE)((hciCmd >> 8) & 0xff);
    au8Hdr[3] = (BYTE)(bytesPerPacket & 0xff);
    au8Hdr[4] = (BYTE)((bytesPerPacket >> 8) & 0xff);

    int headerLen = 0;
    int written = 0;

    if (gs32AudioBufferSize < bytesPerPacket){
        if (gu8AudioBuffer){
            free(gu8AudioBuffer);
            gu8AudioBuffer = NULL;
        }
        gu8AudioBuffer = (uint8_t *)malloc(bytesPerPacket + sizeof(au8Hdr) + headerLen);
        gs32AudioBufferSize = bytesPerPacket;
    }

    memcpy(gu8AudioBuffer + written, au8Hdr, sizeof(au8Hdr));
    written += sizeof(au8Hdr);

    if (remaining >= bytesPerPacket){
        memcpy(gu8AudioBuffer + written, puHci->m_pData + puHci->m_dwAudioSent, bytesPerPacket);
        written += bytesPerPacket;

        puHci->m_dwAudioSent += bytesPerPacket;
    }
    else{
        memcpy(gu8AudioBuffer + written, puHci->m_pData + puHci->m_dwAudioSent, remaining);
        written += remaining;

        // resetting the audio file to the origin.
        puHci->m_dwAudioSent = 0;

        memcpy(gu8AudioBuffer + written, puHci->m_pData + puHci->m_dwAudioSent, bytesPerPacket - remaining);
        written += bytesPerPacket - remaining;

        puHci->m_dwAudioSent += bytesPerPacket - remaining;
    }

    m_pParent->m_audio_packets.lock();
    int sent = 0;
    int max = written;

    while ((sent + max) < written)
    {
        if  (!m_pParent->SendWicedCommand(0, gu8AudioBuffer + sent, max))
        {
            m_pParent->Log("audio write failed");
            m_pParent->m_audio_packets.unlock();
            return;
        }
        sent += max;
    }

    if  (!m_pParent->SendWicedCommand(0, gu8AudioBuffer + sent, written - sent))
    {
        m_pParent->Log("audio write failed");
        m_pParent->m_audio_packets.unlock();
        return;
    }

#ifdef Q_OS_LINUX
     // usleep 0 is for Linux only, w/o for audio choppiness.
    usleep(0);
#endif

    m_pParent->m_audio_packets.unlock();

    if (puHci->m_dwAudioSent >= puHci->m_dwChunkLen){
        puHci->m_dwAudioSent = 0;
    }
}

// Loop till the embedded app asks for audio data
void AudioFileWriter::run()
{
    int packetsToSend = 0;
    QMutex mutex;
    while (1)
    {
        mutex.lock();
        // wait for Event requesting an audio buffer
        m_pParent->audio_tx_wait.wait(&mutex);

        mutex.unlock();

        m_pParent->m_audio_packets.lock();

        packetsToSend = m_pParent->m_uAudio.m_PacketsToSend;

        m_pParent->m_audio_packets.unlock();

        while (packetsToSend > (int) m_pParent->m_uAudio.m_PacketsSent)
        {
          if (m_pParent->m_uAudio.m_BytesPerPacket)
          {
              SendNextData(&m_pParent->m_uAudio, m_pParent->m_uAudio.m_BytesPerPacket);
          }
          m_pParent->m_uAudio.m_PacketsSent++;
        }


    }
}

void MainWindow::on_btnHelpAVSRC_clicked()
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
