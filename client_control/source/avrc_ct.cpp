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
 * Sample MCU application for AVRC Controller (AVRC-CT), AMS and ANCS using WICED HCI protocol.
 */


#include "app_include.h"
#include "avrc.h"
extern "C"
{
#include "app_host.h"
}

static tAPP_AVRC_META_ATTRIB     repeat_ct;
static tAPP_AVRC_META_ATTRIB     shuffle_ct;

static const char *szAvrcPlayStatus[] = { "Stopped", "Playing", "Paused", "Fwd Seek", "Rev Seek" };
static const char *repeatStr[] = { "Off", "Single", "All", "Group"};
static const char *shuffleStr[] = { "Off", "All", "Group"};

// Initialize app
void MainWindow::InitAVRCCT()
{
    // setup signals/slots
    connect(ui->btnCTPlay, SIGNAL(clicked()), this, SLOT(onCTPlay()));
    connect(ui->btnCTStop, SIGNAL(clicked()), this, SLOT(onCTStop()));
    connect(ui->btnCTPause, SIGNAL(clicked()), this, SLOT(onCTPause()));
    connect(ui->btnCTPrev, SIGNAL(clicked()), this, SLOT(onCTPrevious()));
    connect(ui->btnCTNext, SIGNAL(clicked()), this, SLOT(onCTNext()));
    connect(ui->btnCTSeekBack, SIGNAL(pressed()), this, SLOT(onCTSkipBackwardPressed()));
    connect(ui->btnCTSeekBack, SIGNAL(released()), this, SLOT(onCTSkipBackwardReleased()));
    connect(ui->btnCTSeekFwd, SIGNAL(pressed()), this, SLOT(onCTSkipForwardPressed()));
    connect(ui->btnCTSeekFwd, SIGNAL(released()), this, SLOT(onCTSkipForwardReleased()));
    connect(ui->btnCTMute, SIGNAL(clicked()), this, SLOT(onCTMute()));
    connect(ui->btnCTVolumeUp, SIGNAL(clicked()), this, SLOT(onCTVolumeUp()));
    connect(ui->btnCTVolumeDown, SIGNAL(clicked()), this, SLOT(onCTVolumeDown()));
    connect(ui->cbCTRepeat, SIGNAL(currentIndexChanged(int)), this, SLOT(onCTRepeatMode(int)));
    connect(ui->cbCTShuffle, SIGNAL(currentIndexChanged(int)), this, SLOT(onCTShuffleMode(int)));

    connect(ui->cbCTVolume, SIGNAL(currentIndexChanged(int)), this, SLOT(cbCTVolumeChanged(int)));
    connect(ui->btnCTConnect, SIGNAL(clicked()), this, SLOT(onCTConnect()));
    connect(ui->btnCTDisconnect, SIGNAL(clicked()), this, SLOT(onCTDisconnect()));
    connect(ui->btnCTANCSPositive, SIGNAL(clicked()), this, SLOT(OnBnClickedAncsPositive()));
    connect(ui->btnCTANCSNegative, SIGNAL(clicked()), this, SLOT(OnBnClickedAncsNegative()));
    connect(ui->btnCTUnitInfo, SIGNAL(clicked()), this, SLOT(onCTUnitInfo()));
    connect(ui->btnCTSubUnitInfo, SIGNAL(clicked()), this, SLOT(onCTSubUnitInfo()));

    ui->btnCTPlay->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    ui->btnCTPrev->setIcon(style()->standardIcon(QStyle::SP_MediaSkipBackward));
    ui->btnCTStop->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
    ui->btnCTPause->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    ui->btnCTNext->setIcon(style()->standardIcon(QStyle::SP_MediaSkipForward));
    ui->btnCTSeekFwd->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
    ui->btnCTSeekBack->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
    ui->btnCTMute->setIcon(style()->standardIcon(QStyle::SP_MediaVolumeMuted));


    // Table for Music media player elements
    ui->tblMetaPlayerCT->horizontalHeader()->setStretchLastSection(true);
    ui->tblMetaPlayerCT->verticalHeader()->setVisible(true);
    ui->tblMetaPlayerCT->setRowCount(7);
    ui->tblMetaPlayerCT->setColumnCount(2);
    ui->tblMetaPlayerCT->setColumnWidth(0,120);
    ui->tblMetaPlayerCT->setColumnWidth(1,330);

    QStringList list;

    m_volMute = true;

    list.clear();
    list<<"Media Element"<<"Value";
    ui->tblMetaPlayerCT->setHorizontalHeaderLabels(list);

    // Mute is not available in stack
    ui->btnCTMute->setVisible(false);

    ui->cbCTRepeat->clear();
    ui->cbCTRepeat->addItem(repeatStr[APP_AVRC_PLAYER_VAL_OFF-1], APP_AVRC_PLAYER_VAL_OFF);
    ui->cbCTRepeat->addItem(repeatStr[APP_AVRC_PLAYER_VAL_SINGLE_REPEAT-1], APP_AVRC_PLAYER_VAL_SINGLE_REPEAT);
    ui->cbCTRepeat->addItem(repeatStr[APP_AVRC_PLAYER_VAL_ALL_REPEAT-1], APP_AVRC_PLAYER_VAL_ALL_REPEAT);
    //ui->cbCTRepeat->addItem(repeatStr[APP_AVRC_PLAYER_VAL_GROUP_REPEAT-1], APP_AVRC_PLAYER_VAL_GROUP_REPEAT);

    ui->cbCTShuffle->clear();
    ui->cbCTShuffle->addItem(shuffleStr[APP_AVRC_PLAYER_VAL_OFF-1], APP_AVRC_PLAYER_VAL_OFF);
    ui->cbCTShuffle->addItem(shuffleStr[APP_AVRC_PLAYER_VAL_ALL_SHUFFLE-1], APP_AVRC_PLAYER_VAL_ALL_SHUFFLE);
    //ui->cbCTShuffle->addItem(shuffleStr[APP_AVRC_PLAYER_VAL_GROUP_SHUFFLE-1], APP_AVRC_PLAYER_VAL_GROUP_SHUFFLE);

    m_current_volume_pct = 50;
    ui->cbCTVolume->clear();
    for(int i = 0; i < 101; i++)
    {
        char s[100];
        sprintf(s, "%d", i);
        ui->cbCTVolume->addItem(s);
    }
    ui->cbTGVolume->setCurrentIndex(m_current_volume_pct);

}

// Connect to peer device
void MainWindow::onCTConnect()
{
    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
         return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();

    if (pDev == NULL)
        return;

    app_host_avrc_ct_connect(pDev->m_address);
}

// Disconnect from peer device
void MainWindow::onCTDisconnect()
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    app_host_avrc_ct_disconnect(pDev->m_address);
}

// Send play command
void MainWindow::onCTPlay()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_PLAY_CMD);
}

// Send stop command
void MainWindow::onCTStop()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_STOP_CMD);
}

// Send pause command
void MainWindow::onCTPause()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_PAUSE_CMD);
}

// Send UnitInfo command
void MainWindow::onCTUnitInfo()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_unit_info(bda);
}

// Send UnitInfo command
void MainWindow::onCTSubUnitInfo()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_sub_unit_info(bda);
}

// Send next command
void MainWindow::onCTNext()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_NEXT_CMD);

}

// Send previous command
void MainWindow::onCTPrevious()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_PREVIOUS_CMD);
}

// Send vol up command
void MainWindow::onCTVolumeUp()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_VOL_UP_CMD);
}

// Send vol down command
void MainWindow::onCTVolumeDown()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_VOL_DOWN_CMD);
}

// Send mute command
void MainWindow::onCTMute()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_MUTE_CMD);
    m_volMute = !m_volMute;

    ui->btnCTMute->setIcon(style()->standardIcon(m_volMute ? QStyle::SP_MediaVolumeMuted : QStyle::SP_MediaVolume));
}

// Change repeat option
void MainWindow::onCTRepeatMode(int index)
{
    uint8_t bda[6] = {0};
    if(index == (repeat_ct.curr_value -1))
        return;

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }

    QVariant val = ui->cbCTRepeat->itemData(index);
    repeat_ct.curr_value = val.toUInt();

    app_host_avrc_ct_repeat(bda, repeat_ct.curr_value);
}

// Change shuffle option
void MainWindow::onCTShuffleMode(int index)
{
    uint8_t bda[6] = {0};

    if(index == (shuffle_ct.curr_value -1))
        return;

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }

    QVariant val = ui->cbCTShuffle->itemData(index);
    shuffle_ct.curr_value = val.toUInt();

    app_host_avrc_ct_shuffle(bda, shuffle_ct.curr_value);
}

// Change volume
void MainWindow::cbCTVolumeChanged(int index)
{
    uint8_t bda[6] = {0};

    if(m_current_volume_pct == index)
        return;

    m_current_volume_pct = index;

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }

    app_host_avrc_ct_volume_level(bda, m_current_volume_pct);
}

// Send skip forward press command
void MainWindow::onCTSkipForwardPressed()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_FF_CMD_PRESS);
}

// Send skip forward release command
void MainWindow::onCTSkipForwardReleased()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
    }
    app_host_avrc_ct_command(bda, WICED_AVRCP_CT_FF_CMD_RELEASE);
}

// Send skip backward press command
void MainWindow::onCTSkipBackwardPressed()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
        app_host_avrc_ct_command(pDev->m_address, WICED_AVRCP_CT_REV_CMD_PRESS);
    }
}

// Send skip backward release command
void MainWindow::onCTSkipBackwardReleased()
{
    uint8_t bda[6] = {0};
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if(pDev)
    {
        memcpy(bda, pDev->m_address, BDA_LEN);
        app_host_avrc_ct_command(pDev->m_address, WICED_AVRCP_CT_REV_CMD_RELEASE);
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventAVRCCT(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_DEVICE:
        HandleDeviceEventsAVRCCT(opcode, p_data, len);
        break;

    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
        HandleAVRCControllerEvents(opcode, p_data, len);
        break;

    }
}

// Handle WICED HCI local device events
void MainWindow::HandleDeviceEventsAVRCCT(DWORD opcode, LPBYTE p_data, DWORD len)
{
    UNUSED(p_data);
    UNUSED(len);

    switch (opcode)
    {
        case HCI_CONTROL_EVENT_DEVICE_STARTED:
            break;
    }
}

// Handle WICED HCI events for AVRC CT, AMS, ANCS
void MainWindow::HandleAVRCControllerEvents(DWORD opcode, BYTE *p_data, DWORD len)
{
    uint16_t maxStrLen;

    app_host_avrc_ct_event(opcode, p_data, len);

    switch (opcode)
    {
        // connected with peer
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED:
        {
            break;
        }

        // diconnected from peer
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED:
        {

        }
        break;

        // Peer changed player
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE:
        {
        }
        break;

        // Peer indicated its available settings
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_AVAILABLE:
        {
            Log("AVRCP Controller Player settings available handle: %04x, settings: 0x%x 0x%x 0x%x 0x%x",
                        p_data[0] + (p_data[1] << 8), p_data[2], p_data[3], p_data[4], p_data[5]);

            BYTE *p_setting = &p_data[1];
            const int MAX_RC_APP_SETTING_VALUE   = 4;

            int k;

            for (int i=1; i<=MAX_RC_APP_SETTING_VALUE; i++)
            {
                if (p_setting[i])
                {
                    int value = (int) p_setting[i];

                    if(i == APP_AVRC_PLAYER_SETTING_REPEAT)
                    {
                        ui->cbCTRepeat->clear();
                        k = 1;
                        while(value && k < 5)
                        {
                            value -= 1 << k;
                            ui->cbCTRepeat->addItem(repeatStr[k-1], k);
                            k++;
                        }
                    }

                    if(i == APP_AVRC_PLAYER_SETTING_SHUFFLE)
                    {
                        ui->cbCTShuffle->clear();
                        k = 1;
                        while(value && k < 4)
                        {
                            value -= 1 << k;
                            ui->cbCTShuffle->addItem(shuffleStr[k-1], k);
                            k++;
                        }
                    }
                }
            }
        }

        break;

        // Peer changed settings
    case HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE:               /* AVRCP Controller Player setting changed */
        Log("AVRCP Controller Player setting change handle: %04x ",
                        p_data[0] + (p_data[1] << 8));

        for (int i = 0, j = 3; i < p_data[2]; i++)
        {
            Log("    attr: %02x setting: %02x ", p_data[j], p_data[j + 1]);

            switch (p_data[j++])
            {
            case 2: // Repeat Mode
                repeat_ct.curr_value = p_data[j++];
                Log("Repeat %s", repeatStr[repeat_ct.curr_value-1]);
                ui->cbCTRepeat->setCurrentIndex(repeat_ct.curr_value-1);
             break;

            case 3: // Shuffle Mode
                shuffle_ct.curr_value = p_data[j++];
                Log("Shuffle %s", shuffleStr[shuffle_ct.curr_value-1]);
                ui->cbCTShuffle->setCurrentIndex(shuffle_ct.curr_value-1);
                break;
            }
        }

        break;

        // Peer changed play status
    case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS:
        Log("AVRCP Controller Play status handle: %04x status:%d (%s)",
            p_data[0] + (p_data[1] << 8), p_data[2], szAvrcPlayStatus[p_data[2]]);
        break;

        // Peer indicated current playing track info
    case HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO:           /* AVRCP Controller Track Info event */
      {
        Log("AVRCP Controller Track Info handle: %04x status: %d Attr: %d",
            p_data[0] + (p_data[1] << 8), p_data[2], p_data[3]);

        maxStrLen = p_data[4] + (p_data[5] << 8);

        if (maxStrLen > 60)
            maxStrLen = 60;

        QString mediaName;
        QString mediaType;
        QTableWidgetItem *colType;
        QTableWidgetItem *colVal;
        char name[256] = {0};
        memset(name, 0, 256);

        switch (p_data[3])
        {
        case 1: /* Title */
            mediaType = "Title";
            break;
        case 2: /* Artist */
            mediaType = "Artist";
            break;
        case 3: /* Album */
            mediaType = "Album";
            break;
        case 4: /* track number */
            mediaType = "Track Num";
            break;
        case 5: /* number of tracks */
            mediaType = "Total Tracks";
            break;
        case 6: /* Genre */
            mediaType = "Genre";
            break;
        case 7: /* playing time/track duration*/
            mediaType = "Duration (sec)";
            break;
        default:
            /* Unhandled */
            break;
        }

        if(!mediaType.length())
            break;

        if(maxStrLen)
        {
            memcpy(name, &p_data[6],  maxStrLen);
        }
        else
            sprintf(name, "%s", "<not available>");

        Log("%s - %s", mediaType.toLocal8Bit().data(), name);

        mediaName = name;

        colType = new QTableWidgetItem(mediaType);
        colVal = new QTableWidgetItem(mediaName);

        ui->tblMetaPlayerCT->setItem(p_data[3] -1,0, colType);
        ui->tblMetaPlayerCT->setItem(p_data[3] -1,1, colVal);


    }
        break;

    default:
        /* Unhandled */
        break;
    }

}

// Send ANCS action command for acknowledgement
void MainWindow::OnBnClickedAncsPositive()
{
    BYTE command[5] = { 0, 0, 0, 0, 0 };
    command[0] = m_notification_uid & 0xff;
    command[1] = (m_notification_uid >> 8) & 0xff;
    command[2] = (m_notification_uid >> 16) & 0xff;
    command[3] = (m_notification_uid >> 24) & 0xff;
    Log("Sent command %d for notification uid %ld", 0, m_notification_uid);

    SendWicedCommand(HCI_CONTROL_ANCS_COMMAND_ACTION, command, 5);
}

// Send ANCS action command for rejection
void MainWindow::OnBnClickedAncsNegative()
{
    BYTE command[5] = { 0, 0, 0, 0, 1 };
    command[0] = m_notification_uid & 0xff;
    command[1] = (m_notification_uid >> 8) & 0xff;
    command[2] = (m_notification_uid >> 16) & 0xff;
    command[3] = (m_notification_uid >> 24) & 0xff;

    Log("Sent command %d for notification uid %ld", 1, m_notification_uid);

    SendWicedCommand(HCI_CONTROL_ANCS_COMMAND_ACTION, command, 5);
}

// Simulate button press on stero headphone
// (implementation is embedded application dependent)
void MainWindow::on_btnAVRCTBtnPress_clicked()
{
    wiced_hci_avrc_ct_button_press();
}

// Simulate long button press on stero headphone (press and hold)
// (implementation is embedded application dependent)
void MainWindow::on_btnAVRCTLongBtnPress_clicked()
{
    wiced_hci_avrc_ct_long_button_press();
}


void MainWindow::on_btnHelpAVRC_CT_clicked()
{
    onClear();
    Log("AVRC Controller help topic:");
    Log("");
    Log("Apps : watch");
    Log("Peer device - iPhone or Android phone");
    Log("");
    Log("Discover the phone from the 'BR/EDR Discovery' control and pair the phone");
    Log("- Connect");
    Log("  Create an AVRC connection with a peer device. Note that some peer devices may");
    Log("  not support AVRC CT connection without the A2DP Sink profile.");
    Log("- Disconnect");
    Log("  Disconnect an AVRCP connection with a peer device.");
    Log("- Repeat, Shuffle");
    Log("  These controls change the AVRCP player settings and are used when a peer");
    Log("  device supports AVRCP Controller 1.3 or higher.");
    Log("- Volume");
    Log("  This control displays the absolute volume. (Used when a peer device supports");
    Log("  AVRCP Controller 1.4 or higher).");
    Log("- AMS and ANCS");
    Log("  For AMS and ANCS, reset the board and download the 'watch' app. From the 'GATT'");
    Log("  tab click 'Start Advertisements'. From an iPhone app such as 'Light Blue',");
    Log("  discover the 'watch' app running on the board and pair. Play media on the");
    Log("  iPhone and control it with the AVRC CT controls. Incoming calls and messages");
    Log("  will be displayed on the ANCS controls.");

    ScrollToTop();
}
