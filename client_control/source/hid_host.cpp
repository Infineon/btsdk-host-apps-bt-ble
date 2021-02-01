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
 * Sample MCU application for HID Host (BREDR or BLE) profile using WICED HCI protocol.
 */

#include "wiced_types.h"
#include "app_include.h"
#define WICED_BT_BLE_HIDH_HANDLE_OFFSET             20

#ifdef PCM_ALSA // HID Host Audio based on Linux ALSA API
#include "alsa/asoundlib.h"
#endif

#ifdef Q_OS_WIN32 // HID Host Audio based on Win32 media player API
#include <windows.h>
#include <mmsystem.h>
HWAVEOUT hWaveOut;
WAVEHDR WaveOutHeader[WAVE_HDR_NB];
void CALLBACK WaveOutCallbackVerner(HWAVEOUT hwo, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);
#endif

extern "C"
{
#include "app_host_hidh.h"
}

DWORD qtmin(DWORD len, DWORD bufLen);

void MainWindow::setRadioHIDH_BLE(int ble)
{
    ui->radioHIDHBLE->setChecked(ble);
    // The HID Channel (for Get/Set Report) is not used for BLE
    ui->cbHIDHChannel->setEnabled(!ble);

    ui->radioHIDHBREDR->setChecked(!ble);
}

// Initialize app
void MainWindow::InitHIDH()
{
    ui->cbHIDHProtocol->addItem("Report Mode", 0);
    ui->cbHIDHProtocol->addItem("Boot Mode", 1);
    ui->cbHIDHProtocol->setCurrentIndex(0);

    ui->cbHIDHReportType->addItem("Input", 1);
    ui->cbHIDHReportType->addItem("Output", 2);
    ui->cbHIDHReportType->addItem("Feature", 3);
    ui->cbHIDHReportType->setCurrentIndex(0);

    ui->cbHIDHChannel->addItem("Control", 0);
    ui->cbHIDHChannel->addItem("Interrupt", 1);
    ui->cbHIDHChannel->setCurrentIndex(0);

    ui->editHIDHGetSetReportID->setText("01");
    ui->editHIDHSetGetReportData->setText("55");

    // Default to BLE
    setRadioHIDH_BLE(true);

    m_hidh_wakeup_state = false;
    m_hidh_audio_started = false;
    m_hidh_audio_configured = false;
#ifdef PCM_ALSA
    m_alsa_handle = NULL;
#endif
}

// connect to peer deivce
void MainWindow::on_btnHIDHConnect_clicked()
{
    BYTE    cmd[6];
    uint8_t *p_cmd = cmd;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedHIDDevice();
    if (NULL == pDev)
        return;

    if(pDev->m_hidh_handle != NULL_HANDLE)
    {
        Log("HIDH already connected for selected device with handle %d", pDev->m_hidh_handle);
        return;
    }

    BDADDR_TO_STREAM(p_cmd, pDev->m_address);

    Log("HIDH Connect Command, BDA: %02x:%02x:%02x:%02x:%02x:%02x",
           pDev->m_address[0], pDev->m_address[1], pDev->m_address[2], pDev->m_address[3], pDev->m_address[4], pDev->m_address[5]);

    app_host_hidh_connect(pDev->m_address);
}

void MainWindow::on_btnHIDHSetReport_clicked()
{
    CBtDevice *pDev;
    USHORT nHandle;
    int channel;
    int report_type;
    int report_id;
    QString str;
    char string[1000];
    char *p;
    int value;

    if (m_CommPort == nullptr)
        return;

    pDev = GetConnectedHIDHDevice();
    if (pDev == nullptr)
        return;

    nHandle = pDev->m_hidh_handle;

    // Get Channel (Control/Interupt from ComboBox
    channel = ui->cbHIDHChannel->currentIndex();

    // Get ReportType from ComboBox (ReportType 0 is not allowed)
    report_type = ui->cbHIDHReportType->currentIndex() + 1;

    // Get The Report Id to Set
    str = ui->editHIDHGetSetReportID->text();
    strncpy(string, str.toLocal8Bit().data(), 1000-1);
    p = string;
    if (*p != 0)
    {
        sscanf(p, "%02x", &value);
        report_id = value;
    }
    else
    {
        report_id = 0;
    }

    // Get the Report Data
    str = ui->editHIDHSetGetReportData->text();
    strncpy(string, str.toLocal8Bit().data(), 1000-1);
    app_host_hidh_set_report(nHandle, static_cast<uint8_t>(channel), static_cast<uint8_t>(report_type),
        static_cast<uint8_t>(report_id), string, static_cast<uint32_t>(str.length()));
}

void MainWindow::on_btnHIDHGetReport_clicked()
{
    CBtDevice *pDev;
    uint16_t nHandle;
    int report_type;
    int report_id;
    QString str;
    char string[1000];
    char *p;
    int value;

    if (m_CommPort == NULL)
        return;

    pDev = GetConnectedHIDHDevice();
    if (pDev == NULL)
        return;

    nHandle = pDev->m_hidh_handle;

    // Get ReportType from ComboBox (ReportType 0 is not allowed)
    report_type = ui->cbHIDHReportType->currentIndex() + 1;

    // Get The Report Id to Set
    str = ui->editHIDHGetSetReportID->text();
    strncpy(string, str.toLocal8Bit().data(), 1000-1);
    p = string;
    if (*p != 0)
    {
        sscanf(p, "%02x", &value);
        report_id = value;
    }
    else
    {
        report_id = 0;
    }

    Log("Sending HID GetReport Handle:%d Type:%d Id:%d",
        nHandle, report_type, report_id);

    app_host_hidh_get_report(nHandle, report_type, report_id);
}

// disconnect from peer deivce
void MainWindow::on_btnHIDHDisconnect_clicked()
{
    CBtDevice *pDev;

    pDev = GetConnectedHIDHDevice();
    if (pDev == NULL)
        return;

    Log("Sending HIDH Disconnect Command");
    app_host_hidh_disconnect(pDev->m_address);
}

// Get device descriptor
void MainWindow::on_btnHIDHGetDesc_clicked()
{
    uint16_t nHandle;
    CBtDevice *pDev;

    pDev = GetConnectedHIDHDevice();
    if (pDev == NULL)
        return;

    nHandle = pDev->m_hidh_handle;

    Log("Sending Get HID Descriptor Handle: %d", nHandle);
    app_host_hidh_get_desc(nHandle);
}

// Set HID protocol
void MainWindow::on_cbHIDHProtocol_currentIndexChanged(int index)
{
    uint16_t nHandle;
    uint8_t protocol;
    CBtDevice *pDev;

    UNUSED(index);        // Unused parameter

    if (m_CommPort == NULL)
        return;

    pDev = GetConnectedHIDHDevice();
    if (pDev == NULL)
        return;

    nHandle = pDev->m_hidh_handle;
    protocol = (uint8_t)(ui->cbHIDHProtocol->currentIndex());

    Log("Sending HID Protocol Handle:%d Protocol:%d", nHandle, protocol);
    app_host_hidh_set_proto(nHandle, protocol);
}

// Virtual cable unplug
void MainWindow::HidhVirtualUnplug(uint16_t handle)
{
    CBtDevice *pDev;

    pDev = FindInList(CONNECTION_TYPE_HIDH, handle, ui->cbDeviceList);
    if (pDev == NULL  || (pDev->m_hidh_handle != handle))
        return;

    app_host_hidh_virtual_unplug(pDev->m_address);
    VirtualUnplug(pDev);
}



// Handle WICED HCI events
void MainWindow::onHandleWicedEventHIDH(unsigned int opcode, unsigned char *p_data, unsigned int len)
{

    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_HIDH:
        HandleHIDHEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for HID Host
void MainWindow::HandleHIDHEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char      trace[1024];
    BYTE       bda[6];
    CBtDevice *device;
    UINT16  handle;
    QComboBox * pCB;

    app_host_hidh_event(opcode, p_data, len);
    switch (opcode)
    {
    case HCI_CONTROL_HIDH_EVENT_CONNECTED:
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i + 1];

        handle = p_data[7] + (p_data[8] << 8);
        pCB = handle >= WICED_BT_BLE_HIDH_HANDLE_OFFSET ? ui->cbBLEDeviceList : ui->cbDeviceList;

        device = FindInList(bda, pCB);
        // if this is a new device and successful connection, add to the list
        if ((device == NULL) && (p_data[0] == 0))
        {
            device = AddDeviceToList(bda, pCB, NULL);
        }

        if (device)
        {
            // if succuess
            if (p_data[0] == 0)
            {
                device->m_hidh_handle = handle;
                device->m_conn_type |= CONNECTION_TYPE_HIDH;
                HidHostDeviceAdd(bda);
                Log("HIDH connected: %02x:%02x:%02x:%02x:%02x:%02x handle:%d",
                        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], handle);
                SelectDevice(pCB, bda);
            }
            else
            {
                Log("HIDH connect to %02x:%02x:%02x:%02x:%02x:%02x failed with status:%d ",
                        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], p_data[0] );
                device->m_hidh_handle = NULL_HANDLE;
            }
        }
        break;

    case HCI_CONTROL_HIDH_EVENT_DISCONNECTED:
        handle = p_data[0] | (p_data[1] << 8);
        Log("HIDH Connection Closed handle:%d reason:%d ", handle, p_data[2]);
        CBtDevice * pDev;

        if(handle >= WICED_BT_BLE_HIDH_HANDLE_OFFSET)
            pDev = FindInList(CONNECTION_TYPE_HIDH, handle, ui->cbBLEDeviceList);
        else
            pDev = FindInList(CONNECTION_TYPE_HIDH, handle, ui->cbDeviceList);

        if (pDev)
        {
            pDev->m_hidh_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_HIDH;
        }
        ui->cbHIDHProtocol->setCurrentIndex(0);
        break;

    case HCI_CONTROL_HIDH_EVENT_REPORT:
        sprintf(trace, "HIDH Report handle:%d ReportId:%02X data:", p_data[0] + (p_data[1] << 8), p_data[2]);
        for (uint i = 0; i < len - 3; i++)
            sprintf(&trace[strlen(trace)], "%02x ", p_data[i + 3]);
        Log(trace);
        break;

    case HCI_CONTROL_HIDH_EVENT_STATUS:
        sprintf(trace, "HIDH Cmd %02X%02X Status:%d ", p_data[2], p_data[1], p_data[0]);
        Log(trace);
        break;

    case HCI_CONTROL_HIDH_EVENT_DESCRIPTOR:
        sprintf(trace, "HIDH Descriptor handle:%d status:%d length:%d", p_data[0] + (p_data[1] << 8), p_data[2], (int)len - 3);
        Log(trace);
        DumpMemory(&p_data[3], len - 3);
        break;

    case HCI_CONTROL_HIDH_EVENT_VIRTUAL_UNPLUG:
        sprintf(trace, "HIDH Virtual Unplug handle:%d ", p_data[0] + (p_data[1] << 8));
        Log(trace);
        HidhVirtualUnplug(p_data[0] + (p_data[1] << 8));
        break;

    case HCI_CONTROL_HIDH_EVENT_SET_PROTOCOL:
        sprintf(trace, "HIDH Set Prococol handle:%d status:%d", p_data[0] + (p_data[1] << 8), p_data[2]);
        Log(trace);
        break;

    case HCI_CONTROL_HIDH_EVENT_AUDIO_START:
        m_hidh_audio_started = true;
        DisableAppTraces();
        Log("HIDH Audio Start handle:%d format:%d nb_channel:%d freq:%d",
            p_data[0] + (p_data[1] << 8), p_data[2], p_data[3], p_data[4] + (p_data[5] << 8));

        HandleHidHAudioStart(p_data, len);
        break;

    case HCI_CONTROL_HIDH_EVENT_AUDIO_STOP:
        m_hidh_audio_started = false;
        m_hidh_audio_configured = false;
        if(m_audio_started == false)
            EnableAppTraces();
        Log("HIDH Audio Stop handle:%d", p_data[0] + (p_data[1] << 8));
        HandleHidHAudioStop(p_data, len);
        break;

    case HCI_CONTROL_HIDH_EVENT_AUDIO_DATA:
        HandleHidHAudioRxData(&p_data[2], len - 2);
        break;

    case HCI_CONTROL_HIDH_EVENT_SET_REPORT:
        Log("HIDH SetReport Event handle:%d status:%d", p_data[0] + (p_data[1] << 8), p_data[2]);
        break;

    case HCI_CONTROL_HIDH_EVENT_GET_REPORT:
        sprintf(trace, "HIDH GetReport Event handle:%d status:%d data:", p_data[0] + (p_data[1] << 8), p_data[2]);
        for (uint i = 0; i < len - 3; i++)
            sprintf(&trace[strlen(trace)], "%02x ", p_data[i + 3]);
        Log(trace);
        break;

    default:
        sprintf(trace, "Rcvd Unknown HIDH OpCode: %d", (int)opcode);
        Log(trace);
        break;
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedHIDHDevice()
{
    CBtDevice * pDev = GetSelectedHIDDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_hidh_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as HIDH");
        return NULL;
    }

    return pDev;
}


void MainWindow::on_btnHIDHWakeAdd_clicked()
{
    char string[1000] = { 0 };
    uint8_t report_id;
    uint8_t report_pattern[255] = { 0 };
    int value;
    uint8_t report_len = 0;
    char *p;
    int rv;

    QString str = ui->editHIDHReportID->text();
    strncpy(string, str.toLocal8Bit().data(), 1000-1);
    p = string;
    if (*p != 0)
    {
        sscanf(p, "%02x", &value);
        report_id = value;
    }
    else
    {
        report_id = 0;
    }

    str = ui->editHIDHReportPatern->text();
    strncpy(string, str.toLocal8Bit().data(), 1000-1);

    p = string;
    while (*p != 0)
    {
        rv = sscanf(p, "%02x", &value);
        if (rv == 1)
        {
            report_pattern[report_len++] = value;
            p += 2;
            if (*p == ' ')
                p++;
        }
        else
            break;
    }


    CBtDevice * pDev = GetSelectedHIDDevice();
    if (NULL == pDev)
    {
        return;
    }
    Log("Sending HIDH WakeUp Pattern Add address %02x:%02x:%02x:%02x:%02x:%02x ReportId:0x%x Pattern:",
        pDev->m_address[0], pDev->m_address[1], pDev->m_address[2], pDev->m_address[3], pDev->m_address[4], pDev->m_address[5], report_id);

    DumpMemory(report_pattern, report_len);

    app_host_hidh_set_wakeup_pattern(pDev->m_address, report_id, report_pattern, report_len);
}

void MainWindow::on_btnHIDHWakeEnable_clicked()
{
    uint8_t wakeup_gpio;
    uint8_t wakeup_polarity;

    if (!m_hidh_wakeup_state)
    {
        m_hidh_wakeup_state = 1;
    }
    else
    {
        m_hidh_wakeup_state = 0;
    }

    ui->btnHIDHWakeEnable->setText((m_hidh_wakeup_state == 0) ? "Wake Up Enable" : "Wake Up Disable");

    QString str = ui->editHIDHGPIO->text();
    wakeup_gpio = str.toInt();

    str = ui->editHIDHPolarity->text();
    wakeup_polarity = str.toInt();

    Log("Sending HIDH WakeUp Control Enable:%d GPIO:%d Polarity:%d",
        m_hidh_wakeup_state, wakeup_gpio, wakeup_polarity);
    app_host_hidh_set_wakeup_control(wakeup_gpio, wakeup_polarity, m_hidh_wakeup_state)   ;
}

void MainWindow::on_radioHIDHBLE_clicked()
{
    // The HID Channel (for Get/Set Report) is not used for BLE
    ui->cbHIDHChannel->setEnabled(FALSE);
}

void MainWindow::on_radioHIDHBREDR_clicked()
{
    // The HID Channel (for Get/Set Report) is used for BR/EDR
    ui->cbHIDHChannel->setEnabled(TRUE);
}

CBtDevice* MainWindow::GetSelectedHIDDevice()
{
    CBtDevice * pDev = ui->radioHIDHBLE->isChecked() ?
                (CBtDevice *)GetSelectedLEDevice() : (CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
    {
        Log("No %s device selected", ui->radioHIDHBLE->isChecked() ? "BLE" : "BR-EDR");
    }

    return pDev;
}

// HID Host Audio based on Linux ALSA API
#ifdef PCM_ALSA
void MainWindow::HandleHidHAudioStart(LPBYTE p_data, DWORD len)
{
    uint16_t handle;
    uint8_t format;
    uint16_t frequency;
    int status;
    snd_pcm_format_t alsa_format = SND_PCM_FORMAT_S16_LE;
    UNUSED(len);

    handle = (p_data[0] + (p_data[1] << 8));
    format = p_data[2];
    m_nb_channel = p_data[3];
    frequency = (p_data[4] + (p_data[5] << 8));

    if (format != 0)
    {
        Log("Unsupported audio Format:%d, handle %d", format, handle);
        return;
    }

    if (m_nb_channel != 1)
    {
        Log("Unsupported NbChannel:%d", m_nb_channel);
        return;
    }

    status = snd_pcm_open(&m_alsa_handle, "default",
                SND_PCM_STREAM_PLAYBACK, 0);

    if (status < 0)
    {
        Log("snd_pcm_open failed: %s", snd_strerror(status));
    }



    status = snd_pcm_set_params(m_alsa_handle, alsa_format,
        SND_PCM_ACCESS_RW_INTERLEAVED, m_nb_channel,
        frequency, 1, 500000);/* 0.5sec */

}

void MainWindow::HandleHidHAudioStop(LPBYTE p_data, DWORD len)
{
    UNUSED(p_data);
    UNUSED(len);
    snd_pcm_close(m_alsa_handle);
    m_alsa_handle = NULL;
}

void MainWindow::HandleHidHAudioRxData(LPBYTE p_data, DWORD len)
{

    if (m_alsa_handle != NULL)
    {
        snd_pcm_sframes_t alsa_frames;
        snd_pcm_sframes_t alsa_frames_to_send;


        alsa_frames_to_send = len / m_nb_channel;

        // if (bit_per_sample == 16) //
            alsa_frames_to_send /= 2;


        alsa_frames = snd_pcm_writei(m_alsa_handle, p_data, alsa_frames_to_send);
        if (alsa_frames < 0)
        {
            alsa_frames = snd_pcm_recover(m_alsa_handle, alsa_frames, 0);
            Log("snd_pcm_recoverd");
        }
        if (alsa_frames < 0)
        {
            Log("HandleHidHAudioRxData %s", snd_strerror(alsa_frames));
        }
        else
        {
             //Log("snd_pcm_writei %d, len %d", alsa_frames, len);
        }
    }
}

#endif

// HID Host Audio based on Win32 media player API
#ifdef Q_OS_WIN32
void MainWindow::HandleHidHAudioStart(LPBYTE p_data, DWORD len)
{
    uint16_t handle;
    uint8_t format;
    uint8_t nb_channel;
    uint16_t frequency;
    WAVEFORMATEX wfx;
    MMRESULT mm_result;

    UNUSED(len);      // Unused Parameter

    handle = (p_data[0] + (p_data[1] << 8));
    UNUSED(handle);   // Unused variable
    format = p_data[2];
    nb_channel = p_data[3];
    frequency = (p_data[4] + (p_data[5] << 8));

    if (format != 0)
    {
        Log("Unsupported audio Format:%d", format);
        return;
    }

    if (nb_channel != 1)
    {
        Log("Unsupported NbChannel:%d", m_nb_channel);
        return;
    }

    if (m_hidh_audio_configured == false)
    {
        m_hidh_audio_configured = true;
        wfx.wFormatTag = WAVE_FORMAT_PCM;
        wfx.nChannels = nb_channel; // Mono

        wfx.nSamplesPerSec = frequency;

        wfx.nAvgBytesPerSec = frequency;
        wfx.nBlockAlign = 2;
        wfx.wBitsPerSample = 16;     /* number of bits per sample of mono data */
        wfx.cbSize = 0;             /* the count in bytes of the size of extra information (after cbSize) */

        Log("waveOutOpen nSamplesPerSec:%d nAvgBytesPerSec:%d", wfx.nSamplesPerSec, wfx.nAvgBytesPerSec);
        mm_result = waveOutOpen(&hWaveOut, WAVE_MAPPER, &wfx, (DWORD_PTR)WaveOutCallbackVerner, (DWORD_PTR)this, CALLBACK_FUNCTION);
        if (mm_result != MMSYSERR_NOERROR)
        {
            Log("waveOutOpen failed mm_result:%d", mm_result);
            return;
        }

        memset(&WaveOutHeader, 0, sizeof(WaveOutHeader));
        memset(&WaveOutHeaderBuffer, 0, sizeof(WaveOutHeaderBuffer));
        memset(&WaveOutBuffer, 0, sizeof(WaveOutBuffer));
        WaveOutBufferIn = 0;
        WaveOutBufferOut = 0;
        WaveOutBufferNb = 0;

        for (int i = 0; i < WAVE_HDR_NB; i++)
        {
            WaveOutHeader[i].lpData = (LPSTR)&WaveOutHeaderBuffer[i][0];
            WaveOutHeader[i].dwBufferLength = WAVE_HDR_BUFFER_SIZE;
            mm_result = waveOutPrepareHeader(hWaveOut, &WaveOutHeader[i], sizeof(WAVEHDR));
            if (mm_result != MMSYSERR_NOERROR)
            {
                Log("waveOutPrepareHeader failed mm_result:%d", mm_result);
                return;
            }
            mm_result = waveOutWrite(hWaveOut, &WaveOutHeader[i], sizeof(WAVEHDR));
            if (mm_result != MMSYSERR_NOERROR)
            {
                Log("waveOutWrite failed mm_result:%d", mm_result);
            }
        }
    }
}

void CALLBACK WaveOutCallbackVerner(HWAVEOUT hwo, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2)
{
    MainWindow *pDlg = (MainWindow *)dwInstance;
    UNUSED(hwo);      // Unused Parmeter
    UNUSED(dwParam2); // Unused Parmeter
    pDlg->WaveOutCallback(uMsg, dwParam1);
}

void MainWindow::WaveOutCallback(UINT uMsg, DWORD_PTR dwParam1)
{
    WAVEHDR *p_WaveHeader;
    MMRESULT mm_result;
    int cpy_len, byte_to_copy;
    uint8_t *p;

    if (uMsg == WOM_OPEN)
        Log("MM Opened");
    else if (uMsg == WOM_CLOSE)
        Log("MM Closed");
    else if (uMsg == MM_WOM_DONE)
    {
        p_WaveHeader = (WAVEHDR *)dwParam1;

        /* If data received */
        if (WaveOutBufferNb)
        {
            /* Retreive the audio buffer associated with this Wave Header */
            p = (uint8_t *)p_WaveHeader->lpData;

            /* The Audio buffer an contain up to WAVE_HDR_BUFFER_SIZE bytes */
            byte_to_copy = qtmin(WaveOutBufferNb, WAVE_HDR_BUFFER_SIZE);

            /* Copy data from the receive cycular buffer */
            cpy_len = qtmin(((int)sizeof(WaveOutBuffer) - WaveOutBufferOut), byte_to_copy);
            memcpy(p, WaveOutBuffer + WaveOutBufferOut, cpy_len);
            WaveOutBufferOut += cpy_len;
            p += cpy_len;
            WaveOutBufferNb -= cpy_len;
            if (WaveOutBufferOut >= (int)sizeof(WaveOutBuffer))
                WaveOutBufferOut = 0;
            if (cpy_len < byte_to_copy)
            {
                cpy_len = byte_to_copy - cpy_len;
                memcpy(p, WaveOutBuffer + WaveOutBufferOut, cpy_len);
                WaveOutBufferNb -= cpy_len;
                WaveOutBufferOut += cpy_len;
                if (WaveOutBufferOut >= (int)sizeof(WaveOutBuffer))
                    WaveOutBufferOut = 0;
            }
            /* Resubmit the audio buffer */
            mm_result = waveOutWrite(hWaveOut, p_WaveHeader, sizeof(WAVEHDR));
            if (mm_result != MMSYSERR_NOERROR)
            {
                Log("waveOutWrite failed mm_result:%d", mm_result);
            }
        }
        else
        {
            /* Play silense */
            p = (uint8_t *)p_WaveHeader->lpData;
            memset(p, 0, WAVE_HDR_BUFFER_SIZE);
            mm_result = waveOutWrite(hWaveOut, p_WaveHeader, sizeof(WAVEHDR));
            if (mm_result != MMSYSERR_NOERROR)
            {
                Log("waveOutWrite failed mm_result:%d", mm_result);
            }
        }
    }
}

void MainWindow::HandleHidHAudioStop(LPBYTE p_data, DWORD len)
{
    UNUSED(p_data);   // Unused parameter
    UNUSED(len);      // Unused parameter
}

void MainWindow::HandleHidHAudioRxData(LPBYTE p_data, DWORD len)
{
    int payload_len = len;
    int cpy_len;
    uint8_t *p;

    p =  p_data;

    if (m_hidh_audio_started == 0)
        return;

    if ((WaveOutBufferNb + payload_len) > (int)sizeof(WaveOutBuffer))
    {
        Log("Buffer Overrun in:%d out:%d nb:%d", WaveOutBufferIn, WaveOutBufferOut, WaveOutBufferNb);
        return;
    }

    cpy_len = qtmin(payload_len, (int)sizeof(WaveOutBuffer) - WaveOutBufferIn);
    memcpy(WaveOutBuffer + WaveOutBufferIn, p, cpy_len);
    p += cpy_len;
    WaveOutBufferIn += cpy_len;
    if (WaveOutBufferIn >= (int)sizeof(WaveOutBuffer))
        WaveOutBufferIn = 0;
    if (cpy_len != payload_len)
    {
        cpy_len = payload_len - cpy_len;
        memcpy(WaveOutBuffer + WaveOutBufferIn, p, cpy_len);
        p += cpy_len;
        WaveOutBufferIn += cpy_len;
        if (WaveOutBufferIn == sizeof(WaveOutBuffer))
            WaveOutBufferIn = 0;
        if (WaveOutBufferIn > (int)sizeof(WaveOutBuffer))
        {
            WaveOutBufferIn = 0;
            Log("WaveOutBufferIn overrun");
        }
    }
    WaveOutBufferNb += payload_len;
}
#endif

void MainWindow::on_btnHelpHIDH_clicked()
{
    onClear();
    Log("HID Host help topic:");
    Log("");
    Log("Apps : hci_hid_host for BR-EDR or 'hci_ble_hid_host' for BLE HOGP");
    Log("Peer device - BT/BLE keyboard or mouse");
    Log("");
    Log("- Connect");
    Log("  Connect to a HID device such as a keyboard or mouse. After connection, the");
    Log("  input from the HID device will be displayed in BTSpy.exe");
    Log("- Disconnect");
    Log("  Disconnect from a peer device");
    Log("- Get Desc");
    Log("  Get Descriptor from the peer device. The output is displayed in BTSpy.exe.");
    Log("- HID Protocol");
    Log("  Set the protocol as Report mode or Boot mode");
    ScrollToTop();
}
