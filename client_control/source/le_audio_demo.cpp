/*
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "wiced_hci_le_audio.h"
}
#include "QDebug"
#include "mainwindow.h"
#include <QByteArray>
#include <QMessageBox>

Q_DECLARE_METATYPE(CBtDevice *)

#define TAG "LE Audio"

#define START 1
#define STOP 0
#define SYNC 1
#define TERMINATE 0
#define ADD 1
#define REMOVE 0
#define CALL_ID 1 //Initial call

QMap<uint32_t, char*> broadcastSourceList;
//QList<uint32_t> broadcastSourceList;
uint8_t le_audio_dev_role = HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE;
QString status_val[] = {"", "PA Sync established", "PA Sync lost", "BIG Sync established", "BIG Sync terminated"};
uint8_t incoming_call_cnt = 0;
char g_friendly_name[MAX_CALL_FRIENDLY_NAME];

enum
{
    BAP_CODEC_CONFIG_8_1_1,
    BAP_CODEC_CONFIG_8_1_2,
    BAP_CODEC_CONFIG_8_2_1,
    BAP_CODEC_CONFIG_8_2_2,
    BAP_CODEC_CONFIG_16_1_1,
    BAP_CODEC_CONFIG_16_1_2,
    BAP_CODEC_CONFIG_16_2_1,
    BAP_CODEC_CONFIG_16_2_2,
    BAP_CODEC_CONFIG_24_1_1,
    BAP_CODEC_CONFIG_24_1_2,
    BAP_CODEC_CONFIG_24_2_1,
    BAP_CODEC_CONFIG_24_2_2,
    BAP_CODEC_CONFIG_32_1_1,
    BAP_CODEC_CONFIG_32_1_2,
    BAP_CODEC_CONFIG_32_2_1,
    BAP_CODEC_CONFIG_32_2_2,
    BAP_CODEC_CONFIG_441_1_1,
    BAP_CODEC_CONFIG_441_1_2,
    BAP_CODEC_CONFIG_441_2_1,
    BAP_CODEC_CONFIG_441_2_2,
    BAP_CODEC_CONFIG_48_1_1,
    BAP_CODEC_CONFIG_48_1_2,
    BAP_CODEC_CONFIG_48_2_1,
    BAP_CODEC_CONFIG_48_2_2,
    BAP_CODEC_CONFIG_48_3_1,
    BAP_CODEC_CONFIG_48_3_2,
    BAP_CODEC_CONFIG_48_4_1,
    BAP_CODEC_CONFIG_48_4_2,
    BAP_CODEC_CONFIG_48_5_1,
    BAP_CODEC_CONFIG_48_5_2,
    BAP_CODEC_CONFIG_48_6_1,
    BAP_CODEC_CONFIG_48_6_2,
};

uint8_t broadcast_supported_config[] = {BAP_CODEC_CONFIG_16_2_2, BAP_CODEC_CONFIG_48_2_2, BAP_CODEC_CONFIG_48_4_2, BAP_CODEC_CONFIG_48_6_2};

// Helper function
uint16_t le_audio_update_conn_id(CBtDevice *p_device)
{
    uint16_t conn_id;

    switch (le_audio_dev_role)
    {
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK:
        conn_id = 0; // local update
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE:
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK:
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER:
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT:
        conn_id = p_device->con_handle;
        break;
    default:
        conn_id = p_device->con_handle;
        break;
    }

    return conn_id;
}

void MainWindow::clear_streams_list()
{
    broadcastSourceList.clear();
}

// Initialize app
void MainWindow::InitUnicastSink()
{
    ui->tabLEAudio->setEnabled(TRUE);
    ui->groupBox_generateCall->setEnabled(FALSE);
    ui->btn_LeAudioConnect->setText("Reconnect");
}

void MainWindow::retrieve_call_on_hold_popup(uint16_t conn_id, uint8_t call_id)
{
    QMessageBox retrieveBox(this);

    retrieveBox.setIconPixmap(QPixmap(":/callhold.png"));
    retrieveBox.setStyleSheet("QLabel{width: 140px; min-width: 50px; max-width: 200px;}");

    retrieveBox.setText("Call on hold.....");

    QPushButton *retrieveButton = retrieveBox.addButton(tr("Retrieve Call"), QMessageBox::AcceptRole);
    QPushButton *termintateButton = retrieveBox.addButton(tr("Terminate Call"), QMessageBox::AcceptRole);
    retrieveBox.exec();

    if (retrieveBox.clickedButton() == termintateButton)
    {
        incoming_call_cnt = 0; // reset counter
        app_host_le_audio_terminate_call(conn_id, call_id, FALSE);
    }
    else if(retrieveBox.clickedButton() == retrieveButton)
    {
        app_host_le_audio_handle_call_action(conn_id, call_id, WICED_BT_GA_CCP_ACTION_RETRIEVE_CALL);
        ongoing_call_terminate_popup(conn_id, call_id);
    }
}

void MainWindow::ongoing_call_terminate_popup(uint16_t conn_id, uint8_t call_id)
{
    QMessageBox popupBox(this);

    popupBox.setIconPixmap(QPixmap(":/IconCall.png"));
    popupBox.setStyleSheet("QLabel{width: 140px; min-width: 50px; max-width: 200px;}");

    popupBox.setText("Onging call.....");

    QPushButton *holdButton = popupBox.addButton(tr("Hold Call"), QMessageBox::AcceptRole);
    QPushButton *termintateButton = popupBox.addButton(tr("Terminate Call"), QMessageBox::AcceptRole);
    popupBox.exec();

    if (popupBox.clickedButton() == termintateButton)
    {
        incoming_call_cnt = 0; // reset counter
        app_host_le_audio_terminate_call(conn_id, call_id, FALSE);
    }
    else if(popupBox.clickedButton() == holdButton)
    {
        app_host_le_audio_handle_call_action(conn_id, call_id, WICED_BT_GA_CCP_ACTION_HOLD_CALL);
        retrieve_call_on_hold_popup(conn_id, call_id);
    }
}

void MainWindow::incoming_call_popup(uint16_t conn_id, uint8_t call_id, char *uri)
{
    QMessageBox msgBox(this);

    msgBox.setWindowTitle("Incoming Call...");
    msgBox.setIconPixmap(QPixmap(":/IconCall.png"));
    msgBox.setStyleSheet("QLabel{width: 140px; min-width: 50px; max-width: 200px;}");

    strcat(strcat(uri,"\n" ), g_friendly_name);
    msgBox.setText(uri);
    QPushButton *acceptButton = msgBox.addButton(tr("Accept Call"), QMessageBox::AcceptRole);
    QPushButton *rejectButton = msgBox.addButton(tr("Reject Call"), QMessageBox::AcceptRole);
    msgBox.exec();

    if (msgBox.clickedButton() == acceptButton)
    {
        incoming_call_cnt++;
        app_host_le_audio_handle_call_action(conn_id, call_id, WICED_BT_GA_CCP_ACTION_ACCEPT_CALL);
        ongoing_call_terminate_popup(conn_id, call_id);
    }
    else if (msgBox.clickedButton() == rejectButton)
    {
        incoming_call_cnt = 0; // reset counter
        app_host_le_audio_terminate_call(conn_id, call_id, TRUE);
    }
}

void MainWindow::join_call_popup(uint16_t conn_id, uint8_t call_id, char *uri)
{
    QMessageBox joinPopup(this);

    joinPopup.setWindowTitle("Incoming Call...");
    joinPopup.setIconPixmap(QPixmap(":/IconCall.png"));
    joinPopup.setStyleSheet("QLabel{width: 140px; min-width: 50px; max-width: 200px;}");

    joinPopup.setText(uri);
    QPushButton *joinButton = joinPopup.addButton(tr("Join Call"), QMessageBox::AcceptRole);
    QPushButton *rejectButton = joinPopup.addButton(tr("Reject Call"), QMessageBox::AcceptRole);
    joinPopup.exec();

    if (joinPopup.clickedButton() == joinButton)
    {
        app_host_le_audio_handle_call_action(conn_id, call_id, WICED_BT_GA_CCP_ACTION_JOIN_CALL);
    }
    else if (joinPopup.clickedButton() == rejectButton)
    {
        incoming_call_cnt = 0; // reset counter
        app_host_le_audio_terminate_call(conn_id, call_id, TRUE);
    }
}

void MainWindow::handle_incoming_call(uint16_t conn_id, uint8_t *p_data)
{
    uint8_t call_id,uri_len;
    char call_uri[MAX_URI_LEN];

    STREAM_TO_UINT8(call_id, p_data);
    memset(call_uri, 0, MAX_URI_LEN);
    STREAM_TO_UINT8(uri_len, p_data);
    memcpy(call_uri, p_data, uri_len);
    Log("[%s] call uri %s Incoming call %x uri_len %d ", TAG, call_uri, call_id, uri_len);

    if(incoming_call_cnt == 1)
        incoming_call_popup(conn_id, call_id, call_uri);
    else if (incoming_call_cnt >= 3) //only after call accepted
        join_call_popup(conn_id, call_id, call_uri);
}

void MainWindow::update_call_friendly_name(uint8_t *p_data)
{
    uint8_t f_name_len;

    memset(g_friendly_name, 0, MAX_CALL_FRIENDLY_NAME);
    STREAM_TO_UINT8(f_name_len, p_data);
    if(f_name_len >= MAX_CALL_FRIENDLY_NAME)
    {
       f_name_len = (MAX_CALL_FRIENDLY_NAME-1);
    }
    memcpy(g_friendly_name, p_data, f_name_len);

    if(!f_name_len)
        Log("[%s] friendly name %s", TAG,g_friendly_name);
}

void MainWindow::update_call_remote_hold_state(uint8_t *p_data)
{
    uint8_t call_id;
    STREAM_TO_UINT8(call_id, p_data);
    Log("[%s] remote hold, call_id %d", TAG, call_id);
}

void MainWindow::on_btn_LeAudioConnect_clicked()
{
    QString text = ui->btn_LeAudioConnect->text();

    int item = ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    Log("LeConnect BtDevice : %02x:%02x:%02x:%02x:%02x:%02x", p_device->m_address[0], p_device->m_address[1],
        p_device->m_address[2], p_device->m_address[3], p_device->m_address[4], p_device->m_address[5]);

    if (text == "Reconnect")
    {
        app_host_gatt_connect(p_device->address_type, p_device->m_address);
        ui->btn_LeAudioConnect->setText("Disconnect");
    }
    else if (text == "Disconnect")
    {
        app_host_gatt_cancel_connect(p_device->address_type, p_device->m_address);
        ui->btn_LeAudioConnect->setText("Reconnect");
    }
}

void MainWindow::on_btn_startLeAdv_clicked()
{
    QString text = ui->btn_startLeAdv->text();

    if (text == "Start Adv")
    {
        app_host_gatt_start_stop_advert(TRUE);
        ui->btn_startLeAdv->setText("Stop Adv");
    }
    else if (text == "Stop Adv")
    {
        app_host_gatt_start_stop_advert(FALSE);
        ui->btn_startLeAdv->setText("Start Adv");
    }
}

void MainWindow::on_btn_setMediaPlayer_clicked()
{
    int item = ui->scoll_devlist->currentIndex();
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

    memcpy(&player_name, player.toStdString().c_str(), player.size());
    int len = player.size();
    Log("[%s] player %s", TAG, player_name);
    app_host_le_audio_set_media_player(p_device->con_handle, len, player_name);
}

void MainWindow::on_btn_playPause_clicked()
{
    uint16_t conn_id = 0;
    QString text = ui->btn_playPause->text();
    int item = ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;
    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
    {
        if (text == "Play")
        {
            app_host_le_audio_command(1, WICED_LE_AUDIO_BROADCAST_SINK_PLAY_PAUSE_CMD, NULL);
            ui->btn_playPause->setText("Pause");
        }
        else if (text == "Pause")
        {
            app_host_le_audio_command(0, WICED_LE_AUDIO_BROADCAST_SINK_PLAY_PAUSE_CMD, NULL);
            ui->btn_playPause->setText("Play");
        }
    }
    else
    {
        if(le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK)
            conn_id = p_device->con_handle;
        else if(le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE)
            conn_id = 0;

        if (text == "Play")
        {
            uint32_t codec_config = ui->codec_config->currentIndex();
            app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_PLAY_CMD, &codec_config);
            ui->btn_playPause->setText("Pause");
        }
        else if (text == "Pause")
        {
            app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_PAUSE_CMD, NULL);
            ui->btn_playPause->setText("Play");
        }

    }

    // remove warning
    UNUSED(conn_id);
}

void MainWindow::on_btn_muteUnmute_clicked()
{
    QString text = ui->btn_muteUnmute->text();
    int item = ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    if (text == "Mute")
    {
        ui->btn_muteUnmute->setText("Unmute");
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_MUTE_CMD, NULL);
    }
    else if (text == "Unmute")
    {
        ui->btn_muteUnmute->setText("Mute");
        app_host_le_audio_command(p_device->con_handle, WICED_LE_AUDIO_UNMUTE_CMD, NULL);
    }
}

void MainWindow::on_btn_volDown_clicked()
{
    int item = ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    uint16_t conn_id = le_audio_update_conn_id(p_device);

    if (ui->btn_muteUnmute->text() == "Unmute")
    {
        app_host_le_audio_command(conn_id, WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_DOWN_CMD, NULL);
    }
    else
    {
        app_host_le_audio_command(conn_id, WICED_LE_AUDIO_VOL_DOWN_CMD, NULL);
    }
}

void MainWindow::on_btn_volUp_clicked()
{
    int item = ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    uint16_t conn_id = le_audio_update_conn_id(p_device);

    if (ui->btn_muteUnmute->text() == "Unmute")
    {
        app_host_le_audio_command(conn_id, WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_UP_CMD, NULL);
    }
    else
    {
        app_host_le_audio_command(conn_id, WICED_LE_AUDIO_VOL_UP_CMD, NULL);
    }
}

void MainWindow::on_btn_absVol_clicked()
{
    int item = ui->scoll_devlist->currentIndex();
    if (item < 0)
        return;

    QString absval = ui->absVal->text();
    int absvol = absval.toInt();

    CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    uint16_t conn_id = le_audio_update_conn_id(p_device);

    app_host_le_audio_set_abs_volume(conn_id, absvol > 255 ? 255 : absvol);
}

void MainWindow::update_media_player_list(uint8_t len, uint8_t *p_data)
{
    int currlen = 0;
    int players_num;
    STREAM_TO_UINT8(players_num, p_data);
    for (int i = 0; i < players_num; i++)
    {
        char player[50];
        memset(player, 0, 50);
        STREAM_TO_UINT8(currlen, p_data);
        memcpy(player, p_data, currlen);
        Log("[%s] player %s", TAG, player);
        p_data = p_data + currlen;

        int index = ui->playerlist->findText(player, Qt::MatchStartsWith);
        if (len != 0 && index == -1)
        {
            ui->playerlist->addItem(player);
        }
    }
}

void MainWindow::update_media_player_status(uint8_t *p_data)
{
    uint8_t status;

    STREAM_TO_UINT8(status, p_data);
    getMediaState(status);
    if (status == 1)
    {
        // playing
        ui->btn_playPause->setText("Pause");
    }
    else if (status == 2)
    {
        // Paused
        ui->btn_playPause->setText("Play");
    }
}

void MainWindow::UpdateLEAudioRole()
{
    QString text;
    switch (le_audio_dev_role)
    {
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE:
        text = "UNICAST SOURCE";
        ui->groupBox_connect->setEnabled(TRUE);
        ui->VolGroupBox->setEnabled(TRUE);
        ui->mediaControlGroupBox->setEnabled(TRUE);
        ui->btn_setMediaPlayer->setEnabled(TRUE);
        ui->groupBox_adv->setEnabled(FALSE);
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->streamSelectionGroupBox->setEnabled(FALSE);
        ui->StreamConfigControlGroupBox->setEnabled(FALSE);
        ui->past->setEnabled(FALSE);
        // supported codec configurations per TMAP spec.
        ui->codec_config->addItem("8_1_1 (7.5ms - 16 kbps-LL)");
        ui->codec_config->addItem("8_1_2 (7.5ms - 16 kbps-HR)");
        ui->codec_config->addItem("8_2_1 (10ms - 16 kbps-LL)");
        ui->codec_config->addItem("8_2_2 (10ms - 16 kbps-HR)");
        ui->codec_config->addItem("16_1_1 (7.5ms - 32 kbps-LL)");
        ui->codec_config->addItem("16_1_2 (7.5ms - 32 kbps-HR)");
        ui->codec_config->addItem("16_2_1 (10ms - 32 kbps-LL)");
        ui->codec_config->addItem("16_2_2 (10ms - 32 kbps-HR)");
        ui->codec_config->addItem("24_1_1 (7.5ms - 48 kbps-LL)");
        ui->codec_config->addItem("24_1_2 (7.5ms - 48 kbps)-HR");
        ui->codec_config->addItem("24_2_1 (10ms - 48 kbps-LL)");
        ui->codec_config->addItem("24_2_2 (10ms - 48 kbps-HR)");
        ui->codec_config->addItem("32_1_1 (7.5ms - 64 kbps-LL)");
        ui->codec_config->addItem("32_1_2 (7.5ms - 64 kbps-HR)");
        ui->codec_config->addItem("32_2_1 (10ms - 64 kbps-LL)");
        ui->codec_config->addItem("32_2_2 (10ms - 64 kbps-HR)");
        ui->codec_config->addItem("441_1_1 (7.5ms - 16 kbps-LL)");
        ui->codec_config->addItem("441_1_2 (7.5ms - 16 kbps-HR)");
        ui->codec_config->addItem("441_2_1 (10ms - 16 kbps-LL)");
        ui->codec_config->addItem("441_2_2 (10ms - 16 kbps-HR)");
        ui->codec_config->addItem("48_1_1 (7.5ms - 80 kbps-LL)");
        ui->codec_config->addItem("48_1_2 (7.5ms - 80 kbps-HR)");
        ui->codec_config->addItem("48_2_1 (10ms - 80 kbps-LL)");
        ui->codec_config->addItem("48_2_2 (10ms - 80 kbps-HR)");
        ui->codec_config->addItem("48_3_1 (7.5ms - 96 kbps-LL)");
        ui->codec_config->addItem("48_3_2 (7.5ms - 96 kbps-HR)");
        ui->codec_config->addItem("48_4_1 (10ms - 96 kbps-LL)");
        ui->codec_config->addItem("48_4_2 (10ms - 96 kbps-HR)");
        ui->codec_config->addItem("48_5_1 (7.5ms - 96 kbps-LL)");
        ui->codec_config->addItem("48_5_2 (7.5ms - 96 kbps-HR)");
        ui->codec_config->addItem("48_6_1 (10ms - 96 kbps-LL)");
        ui->codec_config->addItem("48_6_2 (10ms - 96 kbps-HR)");
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK:
        text = "UNICAST SINK";
        ui->groupBox_adv->setEnabled(TRUE);
        ui->VolGroupBox->setEnabled(TRUE);
        ui->mediaControlGroupBox->setEnabled(TRUE);
        ui->btn_setMediaPlayer->setEnabled(TRUE);
        ui->groupBox_connect->setEnabled(FALSE);
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->streamSelectionGroupBox->setEnabled(FALSE);
        ui->StreamConfigControlGroupBox->setEnabled(FALSE);
        ui->past->setEnabled(FALSE);
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER:
        text = "CALL CONTROL SERVER (Phone)";
        ui->groupBox_connect->setEnabled(TRUE);
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->groupBox_adv->setEnabled(FALSE);
        ui->streamSelectionGroupBox->setEnabled(FALSE);
        ui->StreamConfigControlGroupBox->setEnabled(FALSE);
        ui->VolGroupBox->setEnabled(FALSE);
        ui->mediaControlGroupBox->setEnabled(FALSE);
        ui->past->setEnabled(FALSE);
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT:
        text = "CALL CONTROL CLIENT (EB)";
        ui->groupBox_adv->setEnabled(TRUE);
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->groupBox_connect->setEnabled(FALSE);
        ui->streamSelectionGroupBox->setEnabled(FALSE);
        ui->StreamConfigControlGroupBox->setEnabled(FALSE);
        ui->VolGroupBox->setEnabled(FALSE);
        ui->mediaControlGroupBox->setEnabled(FALSE);
        ui->past->setEnabled(FALSE);
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE:
        text = "BROADCAST SOURCE";
        ui->StreamConfigControlGroupBox->setEnabled(TRUE);
        ui->VolGroupBox->setEnabled(FALSE);
        ui->mediaControlGroupBox->setEnabled(FALSE);
        ui->streamSelectionGroupBox->setEnabled(FALSE);
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->groupBox_adv->setEnabled(FALSE);
        ui->groupBox_connect->setEnabled(FALSE);

        // supported codec configurations per TMAP spec.
        ui->codecConfigComboBox->addItem("16_2 (10ms - 32 kbps)");
        // ui->codecConfigComboBox->addItem("48_1 (7.5ms - 80 kbps)");
        ui->codecConfigComboBox->addItem("48_2 (10ms - 80 kbps)");
        // ui->codecConfigComboBox->addItem("48_3 (7.5ms - 96 kbps)");
        ui->codecConfigComboBox->addItem("48_4 (10ms - 96 kbps)");
        // ui->codecConfigComboBox->addItem("48_5 (7.5ms - 124.8 kbps)");
        ui->codecConfigComboBox->addItem("48_6 (10ms - 124 kbps)");
        ui->past->setEnabled(FALSE);
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK:
        text = "BROADCAST SINK";
        ui->streamSelectionGroupBox->setEnabled(TRUE);
        ui->pushButton->setEnabled(TRUE);
        ui->groupBox_adv->setEnabled(TRUE);
        ui->groupBox_connect->setEnabled(FALSE);
        ui->VolGroupBox->setEnabled(FALSE);
        ui->mediaControlGroupBox->setEnabled(FALSE);
        ui->StreamConfigControlGroupBox->setEnabled(FALSE);
        ui->btn_playPause->setText("Pause");
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->past->setEnabled(FALSE);
        break;
    case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT:
        text = "BROADCAST ASSISTANT";
        ui->streamSelectionGroupBox->setEnabled(TRUE);
        ui->groupBox_connect->setEnabled(TRUE);
        ui->pushButton->setEnabled(TRUE);
        ui->pushButton->setText("Add Source to Delegator");
        ui->VolGroupBox->setEnabled(FALSE);
        ui->mediaControlGroupBox->setEnabled(FALSE);
        ui->groupBox_generateCall->setEnabled(FALSE);
        ui->groupBox_adv->setEnabled(FALSE);
        ui->past->setEnabled(TRUE);
        break;
    }

    ui->deviceRoleTxt->setText(text);
    ui->scoll_devlist->clear();
    ui->btn_startLeAdv->setText("Start Adv");
    ui->playerlist->clear();
    ui->btn_playPause->setText("Play");
    ui->btn_muteUnmute->setText("Mute");
    ui->absVal->setValidator(new QIntValidator(0, 255, this));
}

void MainWindow::getMediaState(int state)
{
    switch (state)
    {
    case 0:
        Log("[%s] Media status inactive", TAG);
        break;
    case 1:
        Log("[%s] Media status playing", TAG);
        break;
    case 2:
        Log("[%s] Media status paused", TAG);
        break;
    case 3:
        Log("[%s] Media status seeking", TAG);
        break;
    }
}

void MainWindow::update_mute_state(int state)
{
    if (state == 1)
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
    uint8_t status;
    uint16_t conn_id;
    uint8_t mute;
    uint8_t *p_data = rx_buf;
    if (len <= 0)
        return;

    if (opcode == HCI_CONTROL_LE_AUDIO_DEVICE_ROLE_EVT)
    {
        EnableTabs(HCI_CONTROL_GROUP_LE_AUDIO, true);
        STREAM_TO_UINT8(le_audio_dev_role, p_data);
        Log("[%s] device role %x", TAG, le_audio_dev_role);
        UpdateLEAudioRole();
        return;
    }

    switch (opcode)
    {
    case HCI_CONTROL_LE_AUDIO_MEDIA_PLAYER_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        update_media_player_list(len, p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_PLAY_STATUS_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        update_media_player_status(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_VOLUME_STATUS_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        STREAM_TO_UINT8(status, p_data);
        Log("[%s] Volume status %d", TAG, status);
        break;
    case HCI_CONTROL_LE_AUDIO_MUTE_STATUS_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        STREAM_TO_UINT8(mute, p_data);
        Log("[%s] Mute status %x", TAG, mute);
        update_mute_state(mute);
        break;
    case HCI_CONTROL_LE_AUDIO_MUTE_AND_VOLUME_STATUS_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        STREAM_TO_UINT8(status, p_data);
        STREAM_TO_UINT8(mute, p_data);
        Log("[%s] Volume status %d", TAG, status);
        Log("[%s] Mute status %x", TAG, mute);
        update_mute_state(mute);
        break;
    case HCI_CONTROL_LE_AUDIO_INCOMING_CALL_EVT:
        incoming_call_cnt++;
        STREAM_TO_UINT16(conn_id, p_data);
        handle_incoming_call(conn_id, p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_BROADCAST_SINK_STREAM_RSP_EVT:
        handle_stream_response_data(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_STATUS_UPDATE_EVT:
        handle_status_update(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_CALL_FRINDLY_NAME_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        update_call_friendly_name(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_REMOTE_HOLD_CALL_EVT:
        STREAM_TO_UINT16(conn_id, p_data);
        update_call_remote_hold_state(p_data);
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
CBtDevice *MainWindow::AddDeviceToListLeAudio(BYTE *addr, QComboBox *pCb, char *bd_name, uint16_t conn_id)
{
    CBtDevice *device = nullptr;
    QString abuffer;

    if (bd_name)
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x (%s)", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                        bd_name);
    else
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Check if device is already present
    int i = pCb->findText(abuffer, Qt::MatchStartsWith);
    if (i == -1)
    {
        QVariant qv;
        device = new CBtDevice();
        device->con_handle = conn_id;
        if (bd_name && strlen(bd_name))
            strncpy(device->m_name, bd_name, sizeof(device->m_name) - 1);

        qv.setValue<CBtDevice *>(device);

        pCb->addItem(abuffer, qv);
        i = pCb->findText(abuffer, Qt::MatchStartsWith);

        memcpy(device->m_address, addr, 6);
    }
    return device;
}

void MainWindow::UpdateLEAudioDevice(BOOL is_connected, BYTE *address, uint16_t conn_id)
{
    if (is_connected)
    {
        AddDeviceToListLeAudio(address, ui->scoll_devlist, NULL, conn_id);
        if(le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER)
        {
            ui->groupBox_generateCall->setEnabled(TRUE);
            ui->rmtHoldBtn->setEnabled(FALSE);
        }
        else if(le_audio_dev_role != HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT)
        {
            ui->mediaControlGroupBox->setEnabled(TRUE);
            ui->VolGroupBox->setEnabled(TRUE);
        }
    }
    else
    {
        ui->btn_playPause->setText("Play");
        ui->btn_startLeAdv->setText("Start Adv");
        ui->mediaControlGroupBox->setEnabled(FALSE);
        ui->VolGroupBox->setEnabled(FALSE);
        ui->playerlist->clear();
    }
}

void MainWindow::on_btn_connectToPeer_clicked()
{
    int item =  ui->cbBLEDeviceList->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->cbBLEDeviceList->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    QString connText = ui->btn_connectToPeer->text();

    if (connText == "Connect")
    {
        Log("LeConnect BtDevice : %02x:%02x:%02x:%02x:%02x:%02x",
            p_device->m_address[0], p_device->m_address[1], p_device->m_address[2], p_device->m_address[3],
                p_device->m_address[4], p_device->m_address[5]);

        app_host_gatt_connect(p_device->address_type, p_device->m_address);

        ui->btn_connectToPeer->setText("Disconnect");
    }
    else
    {
        Log("LeDisConnect BtDevice : %02x:%02x:%02x:%02x:%02x:%02x",
            p_device->m_address[0], p_device->m_address[1], p_device->m_address[2], p_device->m_address[3],
                p_device->m_address[4], p_device->m_address[5]);

        app_host_gatt_cancel_connect(p_device->address_type, p_device->m_address);
        reset_le_audio_ui();
    }
}

void MainWindow::on_btnCall_clicked()
{
    ui->rmtHoldBtn->setEnabled(TRUE);
    app_host_le_audio_generate_call(0x8000, ui->callUri->text().toStdString().length(),
                                    (uint8_t *)ui->callUri->text().toStdString().c_str(),
                                    ui->friendlyName->text().toStdString().length(),
                                    (uint8_t *)ui->friendlyName->text().toStdString().c_str());

}

void MainWindow::on_rmtHoldBtn_clicked()
{
    QString state = ui->rmtHoldBtn->text();

    if (state == "Remote Hold")
    {
        wiced_hci_set_rmt_call_hold(CALL_ID);
        ui->rmtHoldBtn->setText("Remote Retrieve");
    }
    else if(state == "Remote Retrieve")
    {
       wiced_hci_set_rmt_hold_retrieve(CALL_ID);
       ui->rmtHoldBtn->setText("Remote Hold");
    }
}

void MainWindow::on_startBroadcastPushButton_clicked()
{
    QByteArray br_code = QByteArray::fromHex(ui->broadcastCode->text().rightJustified(32, '0').toLatin1());
    uint32_t broadcast_id = 0;
    uint32_t encryption = 0;
    uint32_t num_channels = 0;
    uint8_t bis_count =0;
    uint32_t codec_config = 0;
    bool status = FALSE;

    if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE)
    {
        if (ui->startBroadcastPushButton->text() == "Start Broadcast")
        {
            // get sampling frequencey, num of channels, encryption and start broadcast stream
            encryption = ("Unencrypted BIS" == ui->encryptionComboBox->currentText()) ? 0 : 1;
            num_channels = ui->channelCountComboBox->currentText().toInt();
            bis_count = ui->bisCntcomboBox->currentText().toInt();
            codec_config = broadcast_supported_config[ui->codecConfigComboBox->currentIndex()];
            broadcast_id = ui->BroadcastID->text().rightJustified(6, '0').toInt(&status, 16);
            if(!status)
                return;

            app_host_le_audio_broadcast_source_start_streaming(1, codec_config, bis_count,
                                                               num_channels, encryption, broadcast_id, (uint8_t *)br_code.data());
            Log("Starting Streaming...");
            ui->startBroadcastPushButton->setText(QStringLiteral("Stop Broadcast"));
        }
        else
        {
            app_host_le_audio_broadcast_source_start_streaming(0, codec_config, bis_count, num_channels, encryption, broadcast_id, NULL);
            Log("Stopping Streaming...");
            ui->startBroadcastPushButton->setText(QStringLiteral("Start Broadcast"));
        }
    }
}

void MainWindow::on_scan_streams_clicked()
{
    QString text = ui->scan_streams->text();

    if (text == ("Discover Sources"))
    {
        ui->streams->clear();
        clear_streams_list();
        if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
        {
            Log("Stand-alone broadcast sink role");
            Log("Looking for broadcast source streams...");
            ui->mediaControlGroupBox->setEnabled(false);
            ui->pushButton->setEnabled(true);
            app_host_le_audio_broadcast_sink_find_sources(START);
        }

        else if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT)
        {
            Log("Looking for broadcast source streams...");
            app_host_le_audio_broadcast_assistant_scan_source(START);
        }

        else
            return;

        ui->scan_streams->setText("Stop Discovery");
    }
    else
    {
        if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
        {
            Log("Stop Looking for broadcast source streams...");
            app_host_le_audio_broadcast_sink_find_sources(STOP);
        }

        else if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT)
        {
            Log("Stop Looking for broadcast source streams...");
            app_host_le_audio_broadcast_assistant_scan_source(STOP);
        }

        else
            return;

        ui->scan_streams->setText("Discover Sources");
    }
}

void MainWindow::on_pushButton_clicked()
{
    QByteArray br_code = QByteArray::fromHex(ui->broadcastCodeEdit->text().rightJustified(32, '0').toLatin1());
    QString src_text = ui->pushButton->text();
    uint8_t use_past = FALSE;

    if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
    {
        if (src_text == "Sync to Stream")
        {
            ui->VolGroupBox->setEnabled(TRUE);
            app_host_le_audio_broadcast_sink_sync_to_stream(SYNC, (uint8_t *)br_code.data(),
                                                            ui->streams->currentText().toUInt(Q_NULLPTR, 16));
            ui->pushButton->setText("Terminate Stream");
        }
        else
        {
            app_host_le_audio_broadcast_sink_sync_to_stream(TERMINATE, (uint8_t *)br_code.data(),
                                                            ui->streams->currentText().toUInt(Q_NULLPTR, 16));
            if (le_audio_dev_role != HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT)
                ui->pushButton->setText("Sync to Stream");

            ui->pushButton->setEnabled(false);
            ui->scan_streams->setText("Discover Sources");
        }
    }
    else if (le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT)
    {
        int item = ui->scoll_devlist->currentIndex();
        if (item < 0)
            return;

        CBtDevice *p_device = (CBtDevice *)ui->scoll_devlist->itemData(item).value<CBtDevice *>();

        if (p_device == NULL)
            return;

        uint16_t conn_id = le_audio_update_conn_id(p_device);

        if (src_text == "Add Source to Delegator")
        {
            ui->VolGroupBox->setEnabled(TRUE);
            use_past = ui->past->isChecked();
            app_host_le_audio_broadcast_assistant_select_source(ADD, conn_id, (uint8_t *)br_code.data(),
                                                            ui->streams->currentText().toUInt(Q_NULLPTR, 16), use_past);
            ui->pushButton->setText("Remove Source");
        }
        else
        {
            app_host_le_audio_broadcast_assistant_select_source(REMOVE, conn_id, (uint8_t *)br_code.data(),
                                                            ui->streams->currentText().toUInt(Q_NULLPTR, 16), 0);

            ui->pushButton->setText("Add Source to Delegator");
        }
    }
}

void MainWindow::handle_stream_response_data(uint8_t *p_data)
{
    uint32_t broadcast_id;
    uint8_t len;
    char br_name[20];

    STREAM_TO_UINT32(broadcast_id, p_data);


    memset(br_name, 0, 20);
    STREAM_TO_UINT8(len, p_data);
    if(len >= 20)len = (20-1);
    memcpy(br_name, p_data, len);

    if (!broadcastSourceList.contains(broadcast_id))
    {
        broadcastSourceList.insert(broadcast_id, br_name);
        ui->streams->addItem(QString::number(broadcast_id, 16));
        ui->br_name->addItem(QString::number(broadcast_id, 16) + " : " + QString::fromLocal8Bit(br_name));
    }
    else
    {
        Log("device already been found...");
    }
}

void MainWindow::handle_status_update(uint8_t *p_data)
{
    uint8_t index;
    STREAM_TO_UINT8(index, p_data);
    if(index == 3)
    {
        //BIG sync established
         if(ui->pushButton->text() == "Sync to Stream" && le_audio_dev_role == HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
         {
             ui->pushButton->setText("Terminate Stream");
         }
    }
    Log("status : %s", status_val[index].toLatin1().data());
}

bool wiced_hci_le_audio_broadcast_sink_play_pause_broadcast_code(uint16_t listen)
{
    bool result;
    QByteArray br_code =
        QByteArray::fromHex(g_pMainWindow->ui->broadcastCodeEdit->text().rightJustified(32, '0').toLatin1());

    result = wiced_hci_le_audio_broadcast_sink_play_pause(listen, (uint8_t *)br_code.data());
    return result;
}
