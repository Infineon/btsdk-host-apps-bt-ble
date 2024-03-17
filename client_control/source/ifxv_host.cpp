/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Sample MCU application for IFV-Voice Host using WICED HCI protocol.
 */

#include <QIODevice>
#include <QAudioOutput>
#include "app_include.h"
#include "usb_kb_usage.h"
#include "hci_control_api.h"
#include "rpc.pb.h"

#define BIT(n) (1<<n)

typedef enum
{
    IFX_CAP_PROFILE_IFXV  = BIT(0),
    IFX_CAP_PROFILE_ATV = BIT(1),
} ifxv_cap_profile_e;

typedef enum
{
    IFX_CAP_CODEC_MSBC  = BIT(0),
    IFX_CAP_CODEC_ADPCM = BIT(1),
    IFX_CAP_CODEC_OPUS  = BIT(2),
} ifxv_cap_codec_e;

typedef enum
{
    IFX_CAP_8_BIT_DATA  = BIT(0),
    IFX_CAP_16_BIT_DATA = BIT(1),
} ifxv_cap_data_unit_e;

typedef enum
{
    IFX_CAP_8KHZ        = BIT(0),
    IFX_CAP_16KHZ       = BIT(1),
} ifxv_cap_sampling_rate_e;

typedef enum
{
    IFX_AUDIO_CODEC_PCM       = 0,
    IFX_AUDIO_CODEC_MSBC      = 1,
    IFX_AUDIO_CODEC_ADPCM     = 2,
    IFX_AUDIO_CODEC_OPUS      = 3,
    IFX_AUDIO_CODEC_MAX
} ifxv_audio_codec_e;

typedef enum
{
    IFX_8_BIT_DATA      = 0,
    IFX_16_BIT_DATA     = 1,
    IFX_SUPPORTED_DATA
} ifxv_data_unit_e;

typedef enum
{
    IFX_8KHZ            = 0,
    IFX_16KHZ           = 1,
    IFX_SUPPORTED_SAMPLE_RATE
} ifxv_sample_rate_e;

#pragma pack(1)
typedef struct
{
    uint8_t     profile;
    uint8_t     major;
    uint8_t     minor;
    uint8_t     codec;
    uint8_t     data_unit;
    uint8_t     sampling_rate;
    uint8_t     resp_time_limit_in_sec;
} server_info_t;

typedef struct
{
    uint8_t         codec;
    uint8_t         data_unit;
    uint8_t         sampling_rate;
    uint8_t         streaming_time_limit_in_sec;
    uint16_t        len;
} ifxv_audio_cfg_t;
#pragma pack()

static ifxv_audio_cfg_t ifxv_cfg = {IFX_AUDIO_CODEC_ADPCM, IFX_16_BIT_DATA, IFX_16KHZ, 10, 134};

// Initialize app
void MainWindow::InitIFXVH()
{
    m_ifxv_AudioRaw_fp = NULL;
    m_ifxv_audioRawDataWrSize = 0;
    m_ifxv_connected = 0;
}

void MainWindow::on_cbAlertLevel_activated(int index)
{
    uint8_t cmd[3] = {1};

    cmd[0] = 0; // Group Findme
    cmd[1] = 0; // Set Immediate Alert Level
    cmd[2] = index; // Alert Level
    Log("Send Immediate Alert Level %d", index);
    SendWicedCommand(HCI_CONTROL_IFXVH_COMMAND_CUSTOM_DEFINED, cmd, 3);
}

// Handle WICED HCI events for BLE/BR HID device
void MainWindow::onHandleWicedEventIFXVH(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    char   buf[100];

    switch (opcode)
    {
        case HCI_CONTROL_IFXVH_EVENT_HOST_CAPABILITIES:
            Log("Event Host Capabilities, len=%d", len);
            if (len == sizeof(server_info_t))
            {
                server_info_t * ptr = (server_info_t *) p_data;
                sprintf(buf, "IFX-Voice Host V%d.%d Capabilities", (int) ptr->major,(int) ptr->minor);
                ui->ifxv_host_capabilities->setTitle(buf);
                sprintf(buf, "Profile: %s %s", ptr->profile & IFX_CAP_PROFILE_IFXV ? "IFXV" : "",
                                                ptr->profile & IFX_CAP_PROFILE_ATV ? "ATV" : "");
                ui->ifxvh_profile->setText(buf);
                sprintf(buf, "Codec: %s %s %s", ptr->codec & IFX_CAP_CODEC_MSBC ? "mSBC" : "",
                                                ptr->codec & IFX_CAP_CODEC_ADPCM ? "ADPCM" : "",
                                                ptr->codec & IFX_CAP_CODEC_OPUS ? "OPUS" : "");
                ui->ifxvh_codec->setText(buf);
                sprintf(buf, "Sampling Data: %s %s", ptr->data_unit & IFX_CAP_16_BIT_DATA ? "16-bit" : "",
                                                ptr->data_unit & IFX_CAP_8_BIT_DATA ? "8-bit" : "");
                ui->ifxvh_data->setText(buf);
                sprintf(buf, "Sampling Rate: %s %s", ptr->sampling_rate & IFX_CAP_16KHZ ? "16kHz" : "",
                                                ptr->sampling_rate & IFX_CAP_8KHZ ? "8kHz" : "");
                ui->ifxvh_sampling_rate->setText(buf);
            }
            else
            {
                ui->ifxv_host_capabilities->setTitle("IFX-Voice Host Capabilities");
                ui->ifxvh_profile->setText("Profile:");
                ui->ifxvh_codec->setText("Codec:");
                ui->ifxvh_data->setText("Sampling Data:");
                ui->ifxvh_sampling_rate->setText("Sampling Rate:");
            }
            break;

        case HCI_CONTROL_IFXVH_EVENT_DEVICE_CAPABILITIES:
            Log("Event Device Capabilities, len=%d", len);
            m_ifxv_connected = len;
            if (m_ifxv_connected)
            {
                ui->ifxvh_status->setText("Connected");
                server_info_t * ptr = (server_info_t *) p_data;
                ui->ifxvh_record_button->setText("Record");
                sprintf(buf, "IFX-Voice Device V%d.%d Capabilities", (int) ptr->major,(int) ptr->minor);
                ui->ifxv_device_capabilities->setTitle(buf);
                sprintf(buf, "Profile: %s %s", ptr->profile & IFX_CAP_PROFILE_IFXV ? "IFXV" : "",
                                               ptr->profile & IFX_CAP_PROFILE_ATV ? "ATV" : "");
                ui->ifxvd_profile->setText(buf);
                if (!ptr->codec)
                {
                    sprintf(buf, "Codec: None");
                    ui->ifxvh_record_button->setEnabled(false);
                }
                else
                {
                    ui->ifxvh_record_button->setEnabled(true);
                    sprintf(buf, "Codec: %s %s %s", ptr->codec & IFX_CAP_CODEC_MSBC ? "mSBC" : "",
                                                    ptr->codec & IFX_CAP_CODEC_ADPCM ? "ADPCM" : "",
                                                    ptr->codec & IFX_CAP_CODEC_OPUS ? "OPUS" : "");
                }
                ui->ifxvd_codec->setText(buf);
                sprintf(buf, "Sampling Data: %s %s", ptr->data_unit & IFX_CAP_16_BIT_DATA ? "16-bit" : "",
                                                     ptr->data_unit & IFX_CAP_8_BIT_DATA ? "8-bit" : "");
                ui->ifxvd_data->setText(buf);
                sprintf(buf, "Sampling Rate: %s %s", ptr->sampling_rate & IFX_CAP_16KHZ ? "16kHz" : "",
                                                     ptr->sampling_rate & IFX_CAP_8KHZ ? "8kHz" : "");
                ui->ifxvd_sampling_rate->setText(buf);
            }
            else
            {
                ui->ifxvh_status->setText("Disconnected");
                ui->ifxvh_record_button->setText("Connect");
                ui->ifxv_device_capabilities->setTitle("IFX-Voice Device not connected");
                ui->ifxvd_profile->setText("Profile:");
                ui->ifxvd_codec->setText("Codec:");
                ui->ifxvd_data->setText("Sampling Data:");
                ui->ifxvd_sampling_rate->setText("Sampling Rate:");
            };
            break;

        case HCI_CONTROL_IFXVH_EVENT_AUDIO_CFG:
            Log("Event Audio Config, len=%d", len);
            if (len == sizeof(ifxv_audio_cfg_t))
            {
                memcpy(&ifxv_cfg, p_data, len);
                ifxv_audio_cfg_t * ptr = (ifxv_audio_cfg_t *) p_data;
                sprintf(buf, "Settings: %s, Data %s, Sample Rate %s, Duration %d %s",
                    ptr->codec ==IFX_AUDIO_CODEC_PCM ? "PCM Raw" :
                    ptr->codec ==IFX_AUDIO_CODEC_MSBC ? "Codec mSBC" :
                    ptr->codec ==IFX_AUDIO_CODEC_ADPCM ? "Codec ADPCM" :
                    ptr->codec ==IFX_AUDIO_CODEC_OPUS ? "Codec OPUS" : "Codec Unknown",
                    ptr->data_unit ==IFX_8_BIT_DATA ? "8 bit data":
                    ptr->data_unit ==IFX_16_BIT_DATA ? "16 bit data":"Unknown",
                    ptr->sampling_rate ==IFX_8KHZ ? "8 kHz":
                    ptr->sampling_rate ==IFX_16KHZ ? "16 kHz":"Unknown",
                    ptr->streaming_time_limit_in_sec, ptr->streaming_time_limit_in_sec ? "sec" : "(No limit)");
                ui->ifxvh_status->setText(buf);
            }
            break;
        case HCI_CONTROL_IFXVH_EVENT_AUDIO_START:
            Log("Event Audio Start");
            onHandleWicedEventIFXVH(HCI_CONTROL_IFXVH_EVENT_AUDIO_CFG, p_data, len);
            m_ifxv_AudioRaw_fp = fopen(ui->edAudioRawFile_ifxvh->text().toStdString().c_str(), "wb");
            if (m_ifxv_AudioRaw_fp == NULL)
            {
                QMessageBox(QMessageBox::Information, "Audio data output", "File Open Error", QMessageBox::Ok).exec();
                return;
            }
            Log("File %s opened", ui->edAudioRawFile_ifxvh->text().toStdString().c_str());
            m_ifxv_audioRawDataWrSize = 0;
            break;

        case HCI_CONTROL_IFXVH_EVENT_AUDIO_STOP:
            Log("Event Audio Stop");
            fclose(m_ifxv_AudioRaw_fp);
            m_ifxv_AudioRaw_fp = NULL;
            Log("File %s closed, size=%d", ui->edAudioRawFile_ifxvh->text().toStdString().c_str(), m_ifxv_audioRawDataWrSize);
            play_audio_file(ui->edAudioRawFile_ifxvh->text().toStdString().c_str(), ifxv_cfg.sampling_rate ? 16000 : 8000);
            break;

        case HCI_CONTROL_IFXVH_EVENT_AUDIO_DATA:
            if (m_ifxv_AudioRaw_fp)
            {
                fwrite(p_data, sizeof(uint16_t), len/2, m_ifxv_AudioRaw_fp);
                m_ifxv_audioRawDataWrSize += len;
                char s[20];
                sprintf(s, "%d bytes", m_ifxv_audioRawDataWrSize);
                ui->ifxvh_file_size->setText(s);
            }
            break;
        case HCI_CONTROL_IFXVH_EVENT_MSG:
            Log((const char *)p_data);
            break;

        case HCI_CONTROL_IFXVH_EVENT_STATUS_CHANGED:
            if (len)
            {
                switch (p_data[0]) {
                case HCI_CONTROL_IFXV_STATUS_IDLE:
                    ui->ifxvh_status->setText("Idle");
                    break;
                case HCI_CONTROL_IFXV_STATUS_SCANNING:
                    ui->ifxvh_status->setText("Scanning...");
                    break;
                case HCI_CONTROL_IFXV_STATUS_DISCOVERY:
                    ui->ifxvh_status->setText("Service Discovery...");
                    break;
                case HCI_CONTROL_IFXV_STATUS_CONNECTED:
                    ui->ifxvh_status->setText("Connected");
                    break;
                }
                Log("Event Status Change: %d (%s)", (p_data[0]), ui->ifxvh_status->text().toStdString().c_str());
            }
            break;
    }
}

void MainWindow::on_ifxvh_play_button_pressed()
{
    play_audio_file(ui->edAudioRawFile_ifxvh->text().toStdString().c_str(), ifxv_cfg.sampling_rate ? 16000 : 8000);
}

void MainWindow::on_ifxvh_record_button_pressed()
{
    if (m_ifxv_connected)
    {
        Log("Sending start audio command");
        SendWicedCommand(HCI_CONTROL_IFXVH_COMMAND_AUDIO_START, (uint8_t*)&ifxv_cfg, sizeof(ifxv_audio_cfg_t));
    }
    else
    {
        SendWicedCommand(HCI_CONTROL_IFXVH_COMMAND_CONNECT, NULL, 0);
    }
}

void MainWindow::on_ifxvh_record_button_released()
{
    if (m_ifxv_connected)
    {
        Log("Sending stop audio command");
        SendWicedCommand(HCI_CONTROL_IFXVH_COMMAND_AUDIO_STOP, NULL, 0);
    }
}

void MainWindow::on_ifxvh_browse_button_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Audio Data File"), "", tr("Raw Files (*.raw);;Audio Data Files (*.raw *.bin *.opus *.adpcm *.msbc);;All (*.*)"));
    if (!fileName.isEmpty())
    {
        FILE * tempFile = fopen(fileName.toStdString().c_str(), "rb");
        if (tempFile)
        {
            fclose(tempFile);

            if (QMessageBox::Yes != QMessageBox(QMessageBox::Information, "File Exists", "Overwrite?", QMessageBox::Yes|QMessageBox::No).exec())
            {
                return;
            }
        }
        ui->edAudioRawFile_ifxvh->setText(fileName);
        ui->ifxvh_file_size->setText("0 byte");
    }
}
