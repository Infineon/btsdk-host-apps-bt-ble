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
 * Sample MCU application for AVRC Target (AVRC-TG) using WICED HCI protocol.
 */


#include "app_include.h"
#include "avrc.h"


const char *table_rows[] =
{
    "Title",
    "Artist",
    "Album",
    "Genre",
    "Track num",
    "Num Tracks",
    "Playing Time",
};

/* Sample song, first song */
tAPP_AVRC_ITEM_MEDIA song1 =
{
    //.attr_count =
    APP_AVRC_MAX_NUM_MEDIA_ATTR_ID,
    {
        //.p_attr_list[0] =
        {   APP_AVRC_MEDIA_ATTR_ID_TITLE,
            {   /* The player name, name length and character set id.*/
                8,
                (uint8_t *)"Beat It ",
            },
        },

        //.p_attr_list[1] =
        {
            APP_AVRC_MEDIA_ATTR_ID_ARTIST,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"Michael Jackson "
            },
        },

        //.p_attr_list[2] =
        {
            APP_AVRC_MEDIA_ATTR_ID_ALBUM,
            {   /* The player name, name length and character set id.*/
                8,
                (uint8_t *)"Thriller"
            },
        },

        //.p_attr_list[3] =
        {
            APP_AVRC_MEDIA_ATTR_ID_GENRE,
            {   /* The player name, name length and character set id.*/
                8,
                (uint8_t *)"SoftRock"
            },
        },

        //.p_attr_list[4] =
        {
            APP_AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
            {   /* The player name, name length and character set id.*/
                1,
                (uint8_t *)"3"
            },
        },

        //.p_attr_list[5] =
        {
            APP_AVRC_MEDIA_ATTR_ID_TRACK_NUM,
            {   /* The player name, name length and character set id.*/
                1,
                (uint8_t *)"1"
            },
        },

        //.p_attr_list[6] =
        {
            APP_AVRC_MEDIA_ATTR_ID_PLAYING_TIME,
            {   /* The player name, name length and character set id.*/
                6,
                (uint8_t *)"269000"
            },
        },
    },

};

/* Sample song, second song */
tAPP_AVRC_ITEM_MEDIA song2 =
{
    //.attr_count =
    APP_AVRC_MAX_NUM_MEDIA_ATTR_ID,

     {
        //.p_attr_list[0] =
        {
            APP_AVRC_MEDIA_ATTR_ID_TITLE,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"Happy Nation    "
            },
        },

        //.p_attr_list[1] =
        {
            APP_AVRC_MEDIA_ATTR_ID_ARTIST,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"Jonas, Jenny    "
            },
        },


        //.p_attr_list[2] =
        {
            APP_AVRC_MEDIA_ATTR_ID_ALBUM,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"Ace of Base     "
            },
        },

        //.p_attr_list[3] =
        {
            APP_AVRC_MEDIA_ATTR_ID_GENRE,
            {   /* The player name, name length and character set id.*/
                4,
                (uint8_t *)"Pop "
            },
        },

        //.p_attr_list[4] =
        {
            APP_AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
            {   /* The player name, name length and character set id.*/
                1,
                (uint8_t *)"3"
            },
        },

        //.p_attr_list[5] =
        {
            APP_AVRC_MEDIA_ATTR_ID_TRACK_NUM,
            {   /* The player name, name length and character set id.*/
                1,
                (uint8_t *)"2"
            },
        },

        //.p_attr_list[6] =
        {
            APP_AVRC_MEDIA_ATTR_ID_PLAYING_TIME,
            {   /* The player name, name length and character set id.*/
                6,
                (uint8_t *)"255000"
            },
        },
     },


};

/* Sample song, third song */
tAPP_AVRC_ITEM_MEDIA song3 =
{

    //.attr_count =
    APP_AVRC_MAX_NUM_MEDIA_ATTR_ID,

    {
        // .p_attr_list[0] =
        {
            APP_AVRC_MEDIA_ATTR_ID_TITLE,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"River of Dreams "
            },
        },


    //.p_attr_list[1] =

        {
           APP_AVRC_MEDIA_ATTR_ID_ARTIST,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"Billy Joel      "
            },
        },


        //.p_attr_list[2] =
        {
            APP_AVRC_MEDIA_ATTR_ID_ALBUM,
            {   /* The player name, name length and character set id.*/
                16,
                (uint8_t *)"River of Dreams "
            },
        },

        //.p_attr_list[3] =
        {
            APP_AVRC_MEDIA_ATTR_ID_GENRE,
            {   /* The player name, name length and character set id.*/
                4,
                (uint8_t *)"Soul"
            },
        },

        //.p_attr_list[4] =
        {
            APP_AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
            {   /* The player name, name length and character set id.*/
                1,
                (uint8_t *)"3"
            },
        },

        //.p_attr_list[5] =
        {
            APP_AVRC_MEDIA_ATTR_ID_TRACK_NUM,
            {   /* The player name, name length and character set id.*/
                1,
                (uint8_t *)"3"
            },
        },

        //.p_attr_list[6] =
        {
            APP_AVRC_MEDIA_ATTR_ID_PLAYING_TIME,
            {   /* The player name, name length and character set id.*/
                6,
                (uint8_t *)"211000"
            },
        },
     },
};

/* list of songs */
tAPP_AVRC_ITEM_MEDIA * app_avrc_songs_list[] =
{
    &song1,
    &song2,
    &song3
};


/* current play state */
uint8_t avrc_app_play_state = APP_AVRC_PLAYSTATE_STOPPED;

tAPP_AVRC_META_ATTRIB     repeat_tg;
tAPP_AVRC_META_ATTRIB     shuffle_tg;

/* Currently playing song */
uint8_t app_avrc_cur_play_index = 0;
//*************************************************************************************/

// Initialize app
void MainWindow::InitAVRCTG()
{
    ui->btnTGPlay->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    ui->btnTGPrev->setIcon(style()->standardIcon(QStyle::SP_MediaSkipBackward));
    ui->btnTGStop->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
    ui->btnTGPause->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    ui->btnTGNext->setIcon(style()->standardIcon(QStyle::SP_MediaSkipForward));

    // setup signals/slots
    connect(ui->btnTGPlay, SIGNAL(clicked()), this, SLOT(onTGPlay()));
    connect(ui->btnTGStop, SIGNAL(clicked()), this, SLOT(onTGStop()));
    connect(ui->btnTGPause, SIGNAL(clicked()), this, SLOT(onTGPause()));
    connect(ui->btnTGPrev, SIGNAL(clicked()), this, SLOT(onTGPrevious()));
    connect(ui->btnTGNext, SIGNAL(clicked()), this, SLOT(onTGNext()));
    connect(ui->btnTGConnect, SIGNAL(clicked()), this, SLOT(onTGConnect()));
    connect(ui->btnTGDisconnect, SIGNAL(clicked()), this, SLOT(onTGDisconnect()));
    connect(ui->btnTGRegister_nfy, SIGNAL(clicked()), this, SLOT(onTGRegisterNotification()));
    connect(ui->btnTGMute, SIGNAL(clicked()), this, SLOT(onTGMute()));

    connect(ui->cbTGRepeat, SIGNAL(currentIndexChanged(int)), this, SLOT(oncbTGRepeatCurrentIndexChanged(int)));
    connect(ui->cbTGShuffle, SIGNAL(currentIndexChanged(int)), this, SLOT(oncbTGShuffleCurrentIndexChanged(int)));

    // Table for Music media player elements
    ui->tblMetaPlayerTG->horizontalHeader()->setStretchLastSection(true);
    ui->tblMetaPlayerTG->verticalHeader()->setVisible(true);
    ui->tblMetaPlayerTG->setRowCount(7);
    ui->tblMetaPlayerTG->setColumnCount(2);
    ui->tblMetaPlayerTG->setColumnWidth(0,120);
    ui->tblMetaPlayerTG->setColumnWidth(1,330);


    QStringList list;

    list.clear();
    list<<"Media Element"<<"Value";
    ui->tblMetaPlayerTG->setHorizontalHeaderLabels(list);

    ui->cbTGRepeat->clear();
    ui->cbTGRepeat->addItem("OFF", APP_AVRC_PLAYER_VAL_OFF);
    ui->cbTGRepeat->addItem("Single", APP_AVRC_PLAYER_VAL_SINGLE_REPEAT);
    ui->cbTGRepeat->addItem("All", APP_AVRC_PLAYER_VAL_ALL_REPEAT);
    ui->cbTGRepeat->addItem("Group", APP_AVRC_PLAYER_VAL_GROUP_REPEAT);
    ui->cbTGRepeat->setCurrentIndex(0);

    ui->cbTGShuffle->clear();
    ui->cbTGShuffle->addItem("OFF", APP_AVRC_PLAYER_VAL_OFF);
    ui->cbTGShuffle->addItem("All", APP_AVRC_PLAYER_VAL_ALL_SHUFFLE);
    ui->cbTGShuffle->addItem("Group", APP_AVRC_PLAYER_VAL_GROUP_SHUFFLE);
    ui->cbTGShuffle->setCurrentIndex(0);

    m_current_volume_pct = 50;
    m_current_song_pos = 0; // Setting song position to zero on start.
    m_tg_play_status_timeout_ms = 1000; // Setting play status timeout to 1 second.

    ui->cbTGVolume->clear();
    for(int i = 0; i < 101; i++)
    {
        char s[100];
        sprintf(s, "%d", i);
        ui->cbTGVolume->addItem(s);
    }
    ui->cbTGVolume->setCurrentIndex(m_current_volume_pct);

    SetTrack();


    setAVRCTGUI();
}

// Set button states
void MainWindow::setAVRCTGUI()
{
    ui->btnTGPlay->setEnabled(true);
    ui->btnTGStop->setEnabled(true);
    ui->btnTGPause->setEnabled(true);
    ui->btnTGPrev->setEnabled(true);
    ui->btnTGNext->setEnabled(true);
    ui->btnTGConnect->setEnabled(true);
    ui->btnTGDisconnect->setEnabled(true);
}

// Connect to peer
void MainWindow::onTGConnect()
{
    BYTE    cmd[60];
    int     commandBytes = 0;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
         return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();

    if (pDev == NULL)
        return;

    for (int i = 0; i < 6; i++)
        cmd[commandBytes++] = pDev->m_address[5 - i];

    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_CONNECT, cmd, commandBytes);
}

// Disconnect from peer device
void MainWindow::onTGDisconnect()
{
    BYTE    cmd[60];
    int     commandBytes = 0;

    CBtDevice * pDev =(CBtDevice *)GetConnectedAVRCDevice();
    if (NULL == pDev)
        return;

    for (int i = 0; i < 6; i++)
        cmd[commandBytes++] = pDev->m_address[5 - i];

    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_DISCONNECT, cmd, commandBytes);

    pDev->m_avrc_handle = NULL_HANDLE;
    pDev->m_conn_type &= ~CONNECTION_TYPE_AVRC;
}

// Register for notification
void MainWindow::onTGRegisterNotification()
{
    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_REGISTER_NOTIFICATION, NULL, 0);
}

void MainWindow::TGPlay()
{
    avrc_app_play_state = APP_AVRC_PLAYSTATE_PLAYING;
    TrackInfo();
    PlayerStatus();
}

// Send play command
void MainWindow::onTGPlay()
{
    TGPlay();
    onStartAudio();
    Log("Starting to play");
}

void MainWindow::TGStop()
{
    avrc_app_play_state = APP_AVRC_PLAYSTATE_STOPPED;
    m_current_song_pos = 0; // reset the current song position.
    m_audio_play_status_send_limit_counter = 0; // reset play status send limit conunter.
    PlayerStatus();
}

// Send stop command
void MainWindow::onTGStop()
{
//    if (!m_audio_started)
//        return;
    TGStop();
    onStopAudio();
    Log("Audio stop");


}

// Send pause command
void MainWindow::onTGPause()
{
    Log("Pausing playback");
    avrc_app_play_state = APP_AVRC_PLAYSTATE_PAUSED;
    PlayerStatus();
    onStopAudio();

}

// User clicked Previous button
void MainWindow::onTGPrevious()
{
    Log("Setting PREV TRACK");

    /* Song is updated so start with beginning */
    m_current_song_pos = 0;
    m_audio_play_status_send_limit_counter = 0;

    if(app_avrc_cur_play_index == 0)
        app_avrc_cur_play_index = APP_AVRC_NUMSONGS-1;
    else
        app_avrc_cur_play_index--;
    /* show track info on UI, and send the updated track info to the embedded app */
    SetTrack();
    TrackInfo();

}

// User clicked Next button
void MainWindow::onTGNext()
{
    Log("Setting NEXT TRACK");

    /* Song is updated so start with beginning */
    m_current_song_pos = 0;
    m_audio_play_status_send_limit_counter = 0;

    app_avrc_cur_play_index++;
    if(app_avrc_cur_play_index > APP_AVRC_NUMSONGS-1)
        app_avrc_cur_play_index = 0;
    /* show track info on UI, and send the updated track info to the embedded app */
    SetTrack();
    TrackInfo();
}

// User clicked Mute button
void MainWindow::onTGMute()
{
    Log("Mute Pressed");

    SendWicedCommand(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_MUTE, NULL, 0);
}

// Change repeat option
void MainWindow::oncbTGRepeatCurrentIndexChanged(int index)
{
    if (m_CommPort == NULL)
        return;

    BYTE cmd[1];
    int  commandBytes = 0;

    QVariant val = ui->cbTGRepeat->itemData(index);

    repeat_tg.curr_value = val.toUInt();
    cmd[commandBytes++] = (BYTE)repeat_tg.curr_value;

    Log("Sending  AVRCP target Repeat setting change : %d", repeat_tg.curr_value);

    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_REPEAT_MODE_CHANGE, cmd, commandBytes);

}

// Change shuffle option
void MainWindow::oncbTGShuffleCurrentIndexChanged(int index)
{
    if (m_CommPort == NULL)
        return;

    BYTE cmd[1];
    int    commandBytes = 0;

    QVariant val = ui->cbTGShuffle->itemData(index);
    shuffle_tg.curr_value = val.toUInt();
    cmd[commandBytes++] = (BYTE)shuffle_tg.curr_value;


    Log("Sending AVRCP target Shuffle setting change  : %d", shuffle_tg.curr_value);
    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_SHUFFLE_MODE_CHANGE, cmd, commandBytes);
}

// user changed volume
void MainWindow::on_cbTGVolume_currentIndexChanged(int index)
{
    if (m_CommPort == NULL)
        return;

    if(m_current_volume_pct == index)
        return;

    m_current_volume_pct = index;

    BYTE   cmd[60];
    int    commandBytes = 0;
    CBtDevice * pDev = GetConnectedAVRCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_avrc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;
    cmd[commandBytes++] = m_current_volume_pct;


    Log("Sending Audio volume. Handle: 0x%04x", nHandle);
    SendWicedCommand(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_LEVEL, cmd, commandBytes);
}

/* Send the current track info to the embedded app */
void MainWindow::TrackInfo()
{
    BYTE     cmd[1024];
    uint16_t commandBytes = 0;

    memset(&cmd[0], 0, sizeof(cmd));

    /* send the track information for currently playing song */
    for (int xx=0; xx < app_avrc_songs_list[app_avrc_cur_play_index]->attr_count; xx++)
    {
        cmd[commandBytes++] = (uint8_t)app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[xx].attr_id;;
        cmd[commandBytes++] = (uint8_t)app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[xx].name.str_len;;

        memcpy( &cmd[commandBytes],
                app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[xx].name.p_str,
                app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[xx].name.str_len);
        commandBytes += app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[xx].name.str_len;
    }

    Log("TrackInfo : [%x] commandBytes: %d", HCI_CONTROL_AVRC_TARGET_COMMAND_TRACK_INFO, commandBytes);

    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_TRACK_INFO, cmd, commandBytes);
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventAVRCTG(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_DEVICE:
        HandleDeviceEventsAVRCTG(opcode, p_data, len);
        break;

    case HCI_CONTROL_GROUP_AVRC_TARGET:
        HandleAVRCTargetEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI local device events
void MainWindow::HandleDeviceEventsAVRCTG(DWORD opcode, LPBYTE p_data, DWORD len)
{
    UNUSED(p_data);
    UNUSED(len);

    switch (opcode)
    {
        case HCI_CONTROL_EVENT_DEVICE_STARTED:
//            m_audio_connected = false;
//            m_audio_started = false;
            break;
    }
    setAVRCTGUI();
}

// Handle WICED HCI AVRC-TG events
void MainWindow::HandleAVRCTargetEvents(DWORD opcode, BYTE* p_data, DWORD len)
{
    BYTE     bda[6];
    UINT16 handle;
    CBtDevice *device;
    char   trace[1024];

    Log("HandleAVRCTargetEvents len %ld", len);

    switch (opcode)
    {
    // connected with peer
    case HCI_CONTROL_AVRC_TARGET_EVENT_CONNECTED:
        {
            for (int i = 0; i < 6; i++)
                bda[5 - i] = p_data[i];
            p_data += 6;
            handle = p_data[0] + (p_data[1] << 8);

            // find device in the list with received address and save the connection handle
            if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
                device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

            device->m_avrc_handle = handle;
            device->m_conn_type |= CONNECTION_TYPE_AVRC;

            Log("AVRC connected %02x:%02x:%02x:%02x:%02x:%02x handle %04x",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], handle);

            /* When AVRC is connected, send current application info to embedded app */
            PlayerStatus();
        }
        break;

         // diconnected from peer
    case HCI_CONTROL_AVRC_TARGET_EVENT_DISCONNECTED:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x12 - HCI_CONTROL_AVRC_TARGET_EVENT_DISCONNECTED", handle);
        Log(trace);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_AVRC, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_avrc_handle == handle))
        {
            pDev->m_avrc_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_AVRC;
        }

        break;
    }

        // Peer indicated play
    case HCI_CONTROL_AVRC_TARGET_EVENT_PLAY:
        Log("AVRC Play handle:%04x", p_data[0] + (p_data[1] << 8));
        //m_trace->SetCurSel(m_trace->AddString(trace));

        avrc_app_play_state = APP_AVRC_PLAYSTATE_PLAYING;

        TrackInfo();
        // If audio streaming is not started send a start
        onTGPlay();

        break;

        // Peer indicated pause
    case HCI_CONTROL_AVRC_TARGET_EVENT_PAUSE:
        Log("AVRC Pause handle:%04x", p_data[0] + (p_data[1] << 8));

        avrc_app_play_state = APP_AVRC_PLAYSTATE_PAUSED;

        // If audio streaming is started send a stop
        onTGStop();
        break;

        // Peer indicated stop
    case HCI_CONTROL_AVRC_TARGET_EVENT_STOP:
        Log("AVRC Stop handle:%04x", p_data[0] + (p_data[1] << 8));

        avrc_app_play_state = APP_AVRC_PLAYSTATE_STOPPED;

        // If audio streaming is started send a stop
        onTGStop();
        break;


        // Peer indicated next track
    case HCI_CONTROL_AVRC_TARGET_EVENT_NEXT_TRACK:
        Log("AVRC Next handle:%04x", p_data[0] + (p_data[1] << 8));

        app_avrc_cur_play_index++;
        if(app_avrc_cur_play_index > (APP_AVRC_NUMSONGS-1))
            app_avrc_cur_play_index = 0;
        /* show track info on UI, and send the updated track info to the embedded app */
        SetTrack();
        TrackInfo();

        /* Song is updated so start with beginning */
        m_current_song_pos = 0;
        m_audio_play_status_send_limit_counter = 0;

        break;

        // Peer indicated previous track
    case HCI_CONTROL_AVRC_TARGET_EVENT_PREVIOUS_TRACK:
        Log("AVRC Previous handle:%04x", p_data[0] + (p_data[1] << 8));

        if(app_avrc_cur_play_index == 0)
            app_avrc_cur_play_index = APP_AVRC_NUMSONGS-1;
        else
            app_avrc_cur_play_index--;
        /* show track info on UI, and send the updated track info to the embedded app */
        SetTrack();
        TrackInfo();

        /* Song is updated so start with beginning */
        m_current_song_pos = 0;
        m_audio_play_status_send_limit_counter = 0;
        break;

        // Peer indicated volume changed
    case HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL:
    {
        Log("AVRC Volume handle:%04x Level: %d", p_data[0] + (p_data[1] << 8), p_data[2]);

        int vol = p_data[2];
        if(vol != m_current_volume_pct)
        {
            m_current_volume_pct = vol;
            ui->cbTGVolume->setCurrentIndex(m_current_volume_pct);

            ui->cbCTVolume->setCurrentIndex(m_current_volume_pct);
        }
        break;
    }

        // Peer indicated repeat settings changed
    case HCI_CONTROL_AVRC_TARGET_EVENT_REPEAT_SETTINGS:
        {
            /* Peer device changed repeat settings*/
            repeat_tg.curr_value = p_data[0];

            // Peer changed Player Application Settings. Update the UI to reflect the new value.
            // find the combo box item corresponding to the new value and set that item as selected
            int icnt ;
            icnt = ui->cbTGRepeat->count();

            for(int i = 0; i < icnt; i++)
            {
                QVariant val = ui->cbTGRepeat->itemData(i);
                if(val.toUInt() == repeat_tg.curr_value)
                {
                    ui->cbTGRepeat->setCurrentIndex(i);
                    break;
                }
            }
            Log("Repeat value changed by peer, %d", repeat_tg.curr_value);

       }
       break;

        // Peer indicated shuffle settings changed
    case HCI_CONTROL_AVRC_TARGET_EVENT_SHUFFLE_SETTINGS:
        {
            /* Peer device changed shuffle settings*/
            shuffle_tg.curr_value = p_data[0];

            // Peer changed Player Application Settings. Update the UI to reflect the new value.
            // find the combo box item corresponding to the new value and set that item as selected
            int icnt ;
            icnt = ui->cbTGShuffle->count();

            for(int i = 0; i < icnt; i++)
            {
                QVariant val = ui->cbTGShuffle->itemData(i);
                if(val.toUInt() == shuffle_tg.curr_value)
                {
                    ui->cbTGShuffle->setCurrentIndex(i);
                    break;
                }
            }
            Log("Shuffle value changed by peer, %d", shuffle_tg.curr_value);
        }
        break;

    default:
        /* Unhandled */
        break;
    }

    setAVRCTGUI();
}

/* Show the track info on the UI */
void MainWindow::SetTrack()
{
    QString strTitle = (char *)app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[0].name.p_str;

    for(int i = 0; i < 7; i++)
    {
        QString strCol1 = table_rows[i];
        QTableWidgetItem *col1 = new QTableWidgetItem(strCol1);
        QString strCol2 = (char *)app_avrc_songs_list[app_avrc_cur_play_index]->p_attr_list[i].name.p_str;
        QTableWidgetItem *col2 = new QTableWidgetItem(strCol2);
        ui->tblMetaPlayerTG->setItem(i, 0, col1);
        ui->tblMetaPlayerTG->setItem(i, 1, col2);
    }

    Log("Song: %s", strTitle.toLocal8Bit().data());

}

/* Send player status info to the embedded app */
void MainWindow::PlayerStatus()
{
    /* send play status info to the application */
    BYTE     cmd[16];
    uint16_t commandBytes = 0;
    uint32_t songlen, songpos;
    bool     send_track_change = false;

    uint32_t sample_freq = GetSamplingFrequencyValue(ui->cbSineFreq->currentIndex());

    songlen = m_uAudio.m_dwChunkLen / ((sample_freq*2*2)/1000);
    songpos = m_uAudio.m_dwAudioSent / ((sample_freq*2*2)/1000);

    if (m_current_song_pos > songpos)
    {
        send_track_change = true;
    }
    m_current_song_pos = songpos;

    Log("PlayStatus : [%x] state: %d posn: %d len %d",
        HCI_CONTROL_AVRC_TARGET_COMMAND_PLAYER_STATUS, avrc_app_play_state, songpos, songlen);

    cmd[commandBytes++] = avrc_app_play_state;

    uint32_t *p_cmdbyte = (uint32_t *)&cmd[commandBytes];

    p_cmdbyte[0] = songlen;
    p_cmdbyte[1] = songpos;

    commandBytes += 8;

    SendWicedCommand(HCI_CONTROL_AVRC_TARGET_COMMAND_PLAYER_STATUS, cmd, commandBytes);
    if (send_track_change)
    {
        if(ui->cbTGRepeat->currentIndex() != 1)
        {
            app_avrc_cur_play_index++;
            if(app_avrc_cur_play_index > (APP_AVRC_NUMSONGS-1))
                app_avrc_cur_play_index = 0;
        }
        /* show track info on UI, and send the updated track info to the embedded app */
        SetTrack();
        TrackInfo();
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedAVRCDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_avrc_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as AVRC");
        return NULL;
    }

    return pDev;
}


void MainWindow::on_btnHelpAVRC_TG_clicked()
{
    onClear();
    Log("AVRC Target help topic:");
    Log("");
    Log("Apps : watch");
    Log("Peer device - headset, speaker, car-kit");
    Log("");
    Log("- Connect");
    Log("  Connect to the selected BR/EDR device to create an AVRCP connection. The peer");
    Log("  selected device should support the AVRCP Controller role, such as a headset,");
    Log("  speaker, or car-kit. After creating the A2DP Source connection, the peer");
    Log("  device may automatically create an AVRCP connection. This option is available");
    Log("  only when the 'PTS_TEST_ONLY' compiler option is enabled in the embedded app.");
    Log("- Disconnect");
    Log("  Disconnect an existing AVRCP connection from the selected BR/EDR. This option");
    Log("  is available only when the 'PTS_TEST_ONLY' compiler option is enabled in the");
    Log("  embedded app.");
    Log("- Play, Pause, Stop, Forward, Back");
    Log("  These buttons send 'player status' and 'track info' attribute notifications to");
    Log("  the peer device.  The peer device should support AVRCP 1.3 or higher. Note that");
    Log("  these buttons are not controlling the media in the AV Source UI, and are meant");
    Log("  for demonstrating the API usage. The forward/back buttons will change the");
    Log("  current track of the built in the play list displayed in the AVRCP UI.");
    Log("  Note that the built-in playlist is for display only and does not match the");
    Log("  streaming media in the A2DP Source. The current track is displayed by peer");
    Log("  devices supporting AVRCP Controller 1.3 or higher.");
    Log("- Shuffle/Repeat");
    Log("  These controls change the AVRCP player settings and are used when a peer");
    Log("  device supports AVRCP Controller 1.3 or higher. These controls do not change");
    Log("  the media in the A2DP source UI.");
    Log("- Volume");
    Log("  This control sets or displays the absolute volume. (Used when a peer device");
    Log("  supports AVRCP Controller 1.4 or higher).");

    ScrollToTop();
}
