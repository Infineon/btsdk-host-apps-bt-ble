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
 * Sample MCU application for A2DP Audio Source using WICED HCI protocol.
 */


#include "app_include.h"
extern "C"
{
#include "app_host.h"
}

const char * audio_freq[] =
{
    "16 kHz",
    "32 kHz",
    "44.1 kHz",
    "48 kHz"
};

WaveFileWriter * pWaveFileWriter=NULL;

// Initialize app
void MainWindow::InitAudioSrc()
{
    m_audio_connected = false;
    m_audio_started = false;
    m_audio_i2s_input_enable = false;
    m_bPortOpen = false;
    m_fpAudioFile = NULL;
    memset(&m_uAudio, 0, sizeof(m_uAudio));

    // setup signals/slots
    connect(ui->btnStartAudio, SIGNAL(clicked()), this, SLOT(onStartAudio()));
    connect(ui->btnStopAudio, SIGNAL(clicked()), this, SLOT(onStopAudio()));
    connect(ui->btnConnectAudio, SIGNAL(clicked()), this, SLOT(onConnectAudioSrc()));
    connect(ui->btnDisconnectAudio, SIGNAL(clicked()), this, SLOT(onDisconnectAudioSrc()));
    connect(ui->btnFindAudioFile, SIGNAL(clicked()), this, SLOT(onFindAudioFile()));
    connect(ui->rbAudioSrcFile, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcFile(bool)));
    connect(ui->rbAudioSrcSine, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcSine(bool)));
    connect(ui->rbAudioSrcI2S, SIGNAL(clicked(bool)), this, SLOT(onAudioSrcI2S(bool)));
    ui->edAudioFile->setText( m_settings.value("AudioFile","").toString());
    ui->rbAudioSrcFile->setChecked(m_settings.value("AudioSrcFile",true).toBool());
    ui->rbAudioSrcI2S->setChecked(m_settings.value("AudioSrcI2S",true).toBool());
    ui->rbAudioSrcSine->setChecked(!m_settings.value("AudioSrcFile",false).toBool() &&
                                   !m_settings.value("AudioSrcI2S",false).toBool());
    ui->cbSineFreq->clear();
    for (int i = 0; i < 4; i++)
    {
        ui->cbSineFreq->addItem(audio_freq[i]);
    }
    setAudioSrcUI();

    ui->rbAudioModeMono->setChecked(false);
    ui->rbAudioModeSterio->setChecked(true);
    ui->cbSineFreq->setCurrentIndex(3); // 48.1 KHz

    // Create a thread to send .wav file data to embedded app
    pWaveFileWriter=new WaveFileWriter (this);
    pWaveFileWriter->start(QThread::TimeCriticalPriority);

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
        if (ui->rbAudioSrcFile->isChecked() && (m_uAudio.m_pWavData == NULL))
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
        if (m_uAudio.m_pWavData != NULL)
            HandleA2DPAudioRequestEvent(p_data, len);
        break;

    case HCI_CONTROL_AUDIO_EVENT_COMMAND_COMPLETE:
        Log("Audio event command complete");
        break;

    case HCI_CONTROL_AUDIO_EVENT_COMMAND_STATUS:
        Log("Audio event command status");
        break;

    case HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED:
        Log("Audio event connection attempt failed (0x%X)", opcode);
        break;

    case HCI_CONTROL_AUDIO_EVENT_SUPPORT_FEATURES:
    {
        uint8_t features = p_data[0];

        if (features & AUDIO_SRC_FEATURE_I2S_INPUT)
        {
            Log("AV Source: Feature I2S INPUT enabled");
            m_audio_i2s_input_enable = true;
        }

        setAudioSrcUI();
    }
        break;

    default:
        Log("Rcvd cmd: %d (0x%X)", opcode, opcode);
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


    if (pWaveFileWriter == NULL)
    {
        Log("thread not running\n");
        return;
    }

    if (!m_uAudio.m_pWavData)
    {
        Log("Setup the wave file to send using wavefile <wavfile.wav>\n");
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

// Select audio .wav file
void MainWindow::onFindAudioFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Audio File"), "", tr("Audio Files (*.wav)"));
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

// Initialize audio stream from wav file
#define MAX_PATH          260
bool MainWindow::InitializeAudioFile()
{
    char audioFile[MAX_PATH] = { 0 };
    strcpy(audioFile, ui->edAudioFile->text().toStdString().c_str());

    if (!ExecuteSetWavFile(audioFile)){
        return false;
    }

    return true;
}

void MainWindow::onAudioSrcSine(bool)
{
    m_settings.setValue("AudioSrcFile",false);
    m_settings.setValue("AudioSrcI2S",false);
}

void MainWindow::onAudioSrcFile(bool)
{
#ifdef __MACH__
    Log("!!! WARNING !!! Audio streaming from a file on MacOS is unreliable because of serial port issues. Use sine wave instead.");
    ui->rbAudioSrcSine->setChecked(true);
    return;
#endif
    m_settings.setValue("AudioSrcFile",true);
}

void MainWindow::onAudioSrcI2S(bool)
{
    m_settings.setValue("AudioSrcI2S",true);
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
}

void MainWindow::closeEventAudioSrc(QCloseEvent *event)
{
    UNUSED(event);
    onDisconnectAudioSrc();
    m_settings.setValue("AudioSrcFile",ui->rbAudioSrcFile->isChecked());
    m_settings.setValue("AudioSrcI2S",ui->rbAudioSrcI2S->isChecked());
}

// Read audio .wav file
BYTE* MainWindow::ReadFile(const char* FilePathName, DWORD *pdwWavDataLen)
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
        res = (BYTE*)malloc(size);
        if (res)
        {
            if (fread(res, 1, size, fp) != (size_t)size)
    {
                free(res);
                res = NULL;
            }
            else
                *pdwWavDataLen = size;
        }
        fclose(fp);
    }
    return res;
}

// Get audio data chunks
BYTE* MainWindow::GetWavDataDataChunk(BYTE *pWavData, DWORD dwWavDataLen, DWORD *pdwDataLen)
{
    BYTE* pData = NULL;
    DWORD dwChunkLen;

    do
    {
        // skip "RIFF", check RIFF chunk lenght and skip type ID (WAVE)
        if (dwWavDataLen < 12)
            break;
        if (0 != memcmp(pWavData, "RIFF", 4))
            break;
        dwChunkLen = LE_DWORD(pWavData + 4);
        if (dwChunkLen + 8 > dwWavDataLen)
            break;
        if (0 != memcmp(pWavData + 8, "WAVE", 4))
            break;
        pWavData += 12;
        dwWavDataLen -= 12;
        // find data chunk and return it to caller
        while (dwWavDataLen > 8)
        {
            dwChunkLen = LE_DWORD(pWavData + 4);
            if (dwChunkLen + 8 > dwWavDataLen)
                break;
            if (0 == memcmp(pWavData, "data", 4))
            {
                pData = pWavData + 8;
                *pdwDataLen = dwChunkLen;
                break;
            }
            pWavData += 8 + dwChunkLen;
            dwWavDataLen -= 8 + dwChunkLen;
        }
    } while (false);
    return pData;
}


BYTE * MainWindow::ExecuteSetWavFile(char *pcFileName)
{
    // Read file with WAV data

    if (m_uAudio.m_pWavData)
    {
        free(m_uAudio.m_pWavData);

        m_uAudio.m_pWavData = NULL;
        m_uAudio.m_pData = NULL;
        m_uAudio.m_dwWavDataLen = 0;
        m_uAudio.m_dwChunkLen = 0;
        m_uAudio.m_dwWavSent = 0;
    }

    if (NULL == (m_uAudio.m_pWavData = ReadFile(pcFileName, &m_uAudio.m_dwWavDataLen)))
    {
        Log("Could not open audio file %s", pcFileName);
        return NULL;
    }

    //get Data chunk pointer and length
    if (NULL == (m_uAudio.m_pData = GetWavDataDataChunk(m_uAudio.m_pWavData, m_uAudio.m_dwWavDataLen, &m_uAudio.m_dwChunkLen)))
    {
        Log("Error: could not get the data section of the wav file %s ", pcFileName);
        return NULL;
    }


    return m_uAudio.m_pData;
}

/******************************************************************/
// thread for reading WAV file
WaveFileWriter::WaveFileWriter(MainWindow * pParent) : QThread()
{
    m_pParent = pParent;
    moveToThread(this);
}


uint8_t *gu8AudioBuffer = NULL;
int gs32AudioBufferSize = 0;

// Send wav packets to serial port
void WaveFileWriter::SendNextWav(hci_audio_sample_t * puHci, int bytesPerPacket)
{
    int remaining = puHci->m_dwChunkLen - puHci->m_dwWavSent;

    BYTE au8Hdr[5] =
        { HCI_WICED_PKT, (BYTE)(HCI_CONTROL_AUDIO_DATA & 0xff), (BYTE)((HCI_CONTROL_AUDIO_DATA >> 8) & 0xff), (BYTE)(bytesPerPacket & 0xff), (BYTE)((bytesPerPacket >> 8) & 0xff) };

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
        memcpy(gu8AudioBuffer + written, puHci->m_pData + puHci->m_dwWavSent, bytesPerPacket);
        written += bytesPerPacket;

        puHci->m_dwWavSent += bytesPerPacket;
    }
    else{
        memcpy(gu8AudioBuffer + written, puHci->m_pData + puHci->m_dwWavSent, remaining);
        written += remaining;

        // resetting the wav file to the origin.
        puHci->m_dwWavSent = 0;

        memcpy(gu8AudioBuffer + written, puHci->m_pData + puHci->m_dwWavSent, bytesPerPacket - remaining);
        written += bytesPerPacket - remaining;

        puHci->m_dwWavSent += bytesPerPacket - remaining;
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

    if (puHci->m_dwWavSent >= puHci->m_dwChunkLen){
        puHci->m_dwWavSent = 0;
    }
}

// Loop till the embedded app asks for audio data
void WaveFileWriter::run()
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
              SendNextWav(&m_pParent->m_uAudio, m_pParent->m_uAudio.m_BytesPerPacket);
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
