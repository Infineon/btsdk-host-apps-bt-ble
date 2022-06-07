/*
 * Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Sample MCU application for LE Audio sink using WICED HCI protocol.
 */
#include "app_include.h"
extern "C"
{
#include "app_host.h"
#include "app_host_le_audio.h"
}
#include "mainwindow.h"
#include <QMessageBox>

Q_DECLARE_METATYPE( CBtDevice* )

#define TAG "LE Audio"


// Initialize app
void MainWindow::InitUnicastSink()
{
    ui->tabLEAudio->setEnabled(TRUE);
    //ui->btn_LeAudioConnect->setEnabled(TRUE);
    ui->btn_LeAudioConnect->setText("Reconnect");
}

void MainWindow::incoming_Call_Popup(uint16_t conn_id, uint8_t call_id , char* uri)
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("Incoming Call...");
    msgBox.setIconPixmap(QPixmap(":/IconCall.png"));
    msgBox.setStyleSheet("QLabel{width: 140px; min-width: 50px; max-width: 200px;}");

    msgBox.setText(uri);
    QPushButton *AcceptButton = msgBox.addButton(tr("Accept Call"), QMessageBox::AcceptRole);
    QPushButton *terminateButton = msgBox.addButton(tr("Terminate Call"), QMessageBox::AcceptRole);
    msgBox.exec();

    if (msgBox.clickedButton() == AcceptButton)
    {
        app_host_le_audio_accept_call(conn_id, call_id);
    }
    else if (msgBox.clickedButton() == terminateButton)
    {
        app_host_le_audio_terminate_call(conn_id, call_id);
    }
}

void MainWindow::handle_incoming_Call(uint16_t conn_id, uint8_t *p_data)
{
    uint8_t call_id, uri_len;
    char call_uri[50];

    STREAM_TO_UINT8(call_id, p_data);
    memset(call_uri,0,50);
    STREAM_TO_UINT8(uri_len, p_data);
    memcpy(call_uri, p_data, uri_len);
    Log("[%s] call uri %s Incoming call %x uri_len %d", TAG,call_uri, call_id, uri_len);

    incoming_Call_Popup(conn_id, call_id, call_uri);
}

void MainWindow::on_btn_LeAudioConnect_clicked()
{
    QString text = ui->btn_LeAudioConnect->text();

    int item =  ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    Log("LeConnect BtDevice : %02x:%02x:%02x:%02x:%02x:%02x",
        p_device->m_address[0], p_device->m_address[1], p_device->m_address[2], p_device->m_address[3],
            p_device->m_address[4], p_device->m_address[5]);

    if(text == "Reconnect")
    {
        app_host_gatt_connect(p_device->address_type, p_device->m_address);
        ui->btn_LeAudioConnect->setText("Disconnect");
    }
    else if(text == "Disconnect")
    {
        app_host_gatt_cancel_connect(p_device->address_type, p_device->m_address);
        ui->btn_LeAudioConnect->setText("Reconnect");
    }
}

void MainWindow::on_btn_startLeAdv_clicked()
{
    QString text = ui->btn_startLeAdv->text();

    if(text == "Start Adv")
    {
        app_host_gatt_start_stop_advert(TRUE);
        ui->btn_startLeAdv->setText("Stop Adv");
    }
    else if(text == "Stop Adv")
    {
        app_host_gatt_start_stop_advert(FALSE);
        ui->btn_startLeAdv->setText("Start Adv");
    }
}

void MainWindow::on_btn_setMediaPlayer_clicked()
{
    int item =  ui->scoll_devlist->currentIndex();
    uint8_t player_name[50];
    memset(player_name, 0, 50);
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    QString player = ui->playerlist->currentText();

    if (player.isNull())
        return;

    memcpy( &player_name, player.toStdString().c_str() ,player.size());
    int len = player.size();
    Log("[%s] player %s", TAG, player_name);
    app_host_le_audio_set_media_player(p_device->con_handle, len, player_name);
}

void MainWindow::on_btn_playPause_clicked()
{
    QString text = ui->btn_playPause->text();
    int item =  ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    if(text == "Play")
    {
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_PLAY_CMD);
        ui->btn_playPause->setText("Pause");
    }
    else if(text == "Pause")
    {
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_PAUSE_CMD);
        ui->btn_playPause->setText("Play");
    }
}

void MainWindow::on_btn_muteUnmute_clicked()
{
    QString text = ui->btn_muteUnmute->text();
    int item =  ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    if(text == "Mute")
    {
        ui->btn_muteUnmute->setText("Unmute");
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_MUTE_CMD);
    }
    else if(text == "Unmute")
    {
        ui->btn_muteUnmute->setText("Mute");
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_UNMUTE_CMD);
    }
}

void MainWindow::on_btn_volDown_clicked()
{
    int item =  ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    if(ui->btn_muteUnmute->text() == "Unmute")
    {
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_DOWN_CMD);
    }
    else
    {
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_VOL_DOWN_CMD);
    }

}

void MainWindow::on_btn_volUp_clicked()
{
    int item =  ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    if(ui->btn_muteUnmute->text() == "Unmute")
    {
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_UP_CMD);
    }
    else
    {
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_VOL_UP_CMD);
    }
}

void MainWindow::on_btn_absVol_clicked()
{
    int item =  ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    QString absval =  ui->absVal->text();
    int absvol = absval.toInt();

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    app_host_le_audio_set_abs_volume(p_device->con_handle, absvol > 255 ? 255 : absvol);
}

void MainWindow::update_media_player_list(uint8_t len, uint8_t *p_data)
{
    int currlen = 0;
    int players_num;
    STREAM_TO_UINT8(players_num, p_data);
    for(int i=0; i<players_num; i++)
    {
        char player[50];
        memset(player,0,50);
        STREAM_TO_UINT8(currlen, p_data);
        memcpy(player, p_data, currlen);
        Log("[%s] player %s", TAG, player);
        p_data = p_data + currlen;

        int index = ui->playerlist->findText( player,Qt::MatchStartsWith);
        if (len != 0 && index == -1)
        {
            ui->playerlist->addItem(player);
        }
    }
}


void MainWindow::UpdateLEAudioRole(uint8_t role)
{
    QString text;
    switch(role)
    {
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE:
             text = "UNICAST SOURCE";
        break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK:
             text = "UNICAST SINK";
        break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER:
             text = "CALL CONTROL SERVER (Phone)";
        break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT:
             text = "CALL CONTROL CLIENT (EB)";
        break;
    }

    ui->deviceRoleTxt->setText(text);
    ui->scoll_devlist->clear();
    ui->btn_startLeAdv->setText("Start Adv");
    ui->playerlist->clear();
    ui->btn_playPause->setText("Play");
    ui->btn_muteUnmute->setText("Mute");
    ui->absVal->setValidator( new QIntValidator(0, 255, this) );
    ui->mediaControlGroupBox->setEnabled(false);
    ui->VolGroupBox->setEnabled(false);

}

void MainWindow::getMediaState(int state)
{
   switch(state)
   {
   case 0: Log("[%s] Media status inactive", TAG); break;
   case 1: Log("[%s] Media status playing", TAG); break;
   case 2: Log("[%s] Media status paused", TAG); break;
   case 3: Log("[%s] Media status seeking", TAG); break;
   }
}

void MainWindow::update_mute_state(int state)
{
    if(state == 1)
    {
        ui->btn_muteUnmute->setText("Unmute");
    }
    else
    {
        ui->btn_muteUnmute->setText("Mute");
    }

}

// Handle WICED HCI events for AV sink
void MainWindow::Handle_LE_Audio_Events(DWORD opcode, BYTE *rx_buf, DWORD len)
{
    uint8_t     status;
    uint8_t     role;
    uint16_t    conn_id;
    uint8_t     mute;
    uint8_t*    p_data = rx_buf;
    if(len <= 0)
        return;

    if (opcode == HCI_CONTROL_LE_AUDIO_DEVICE_ROLE_EVT)
    {
        EnableTabs(HCI_CONTROL_GROUP_LE_AUDIO, true);
        STREAM_TO_UINT8(role, p_data);
        Log("[%s] device role %x", TAG, role);
        UpdateLEAudioRole(role);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    switch (opcode)
    {
    case HCI_CONTROL_LE_AUDIO_MEDIA_PLAYER_EVT:
        update_media_player_list(len, p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_PLAY_STATUS_EVT:
        STREAM_TO_UINT8(status, p_data);
        getMediaState(status);
        if(status == 1)
        {
            //playing
            ui->btn_playPause->setText("Pause");
        }
        else if (status == 2)
        {
            //Paused
            ui->btn_playPause->setText("Play");
        }
        break;
    case HCI_CONTROL_LE_AUDIO_VOLUME_STATUS_EVT:
        STREAM_TO_UINT8(status, p_data);
        Log("[%s] Volume status %d", TAG, status);
        break;
    case HCI_CONTROL_LE_AUDIO_MUTE_STATUS_EVT:
        STREAM_TO_UINT8(mute, p_data);
        Log("[%s] Mute status %x", TAG, mute);
        update_mute_state(mute);
        break;
    case HCI_CONTROL_LE_AUDIO_INCOMING_CALL_EVT:
        handle_incoming_Call(conn_id , p_data);
        break;
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventLeAudio(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_LE_AUDIO:
        Handle_LE_Audio_Events(opcode, p_data, len);
        break;
    }
}

// Add new device to combo box
CBtDevice *MainWindow::AddDeviceToListLeAudio(BYTE *addr, QComboBox * pCb, char * bd_name, uint16_t conn_id)
{
    CBtDevice *device = nullptr;
    QString abuffer;

    if (bd_name)
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x (%s)",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],bd_name);
    else
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Check if device is already present
    int i = pCb->findText( abuffer,Qt::MatchStartsWith);
    if (i == -1)
    {
        QVariant qv;
        device = new CBtDevice();
        device->con_handle = conn_id;
        if (bd_name && strlen(bd_name))
            strncpy(device->m_name, bd_name,sizeof(device->m_name)-1);

        qv.setValue<CBtDevice *>(device);

        pCb->addItem( abuffer, qv );
        i = pCb->findText( abuffer, Qt::MatchStartsWith);

        memcpy(device->m_address, addr, 6);
    }
    return device;
}

 void MainWindow::UpdateLEAudioDevice(BOOL is_connected, BYTE *address, uint16_t conn_id)
 {
     if (is_connected)
     {
         AddDeviceToListLeAudio(address, ui->scoll_devlist, NULL, conn_id);
         ui->mediaControlGroupBox->setEnabled(true);
         ui->VolGroupBox->setEnabled(true);
     }
     else
     {
          ui->btn_playPause->setText("Play");
          ui->btn_startLeAdv->setText("Start Adv");
          ui->mediaControlGroupBox->setEnabled(false);
          ui->VolGroupBox->setEnabled(false);
          ui->playerlist->clear();
     }

 }

 void MainWindow::on_btn_connectToPeer_clicked()
 {
     OnBnClickedLeConnect();
 }

 void MainWindow::on_btnCall_clicked()
 {
     app_host_le_audio_generate_call(0x8000, ui->callUri->text().toStdString().length(), (uint8_t*)ui->callUri->text().toStdString().c_str());
 }
