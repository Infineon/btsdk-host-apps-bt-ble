/*
 * Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include <QFileInfo>

Q_DECLARE_METATYPE(CBtDevice *)

#define TAG "LE Audio"

#define START 1
#define STOP 0
#define SYNC 1
#define TERMINATE 0
#define ADD 1
#define REMOVE 0
#define CALL_ID 1 //Initial call

//QMap<uint32_t, char*> broadcastSourceList;
//QList<uint32_t> broadcastSourceList;
uint32_t le_audio_dev_role = HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE;
uint8_t incoming_call_cnt = 0;
char g_friendly_name[MAX_CALL_FRIENDLY_NAME];

extern AudioFileWriter * pAudioFileWriter;

#define STRING_START_BROADCAST "Start Broadcast"
#define STRING_SYNC_TO_STREAM "Sync to Stream"
#define STRING_DISCOVER_SOURCES "Discover Sources"
#define STRING_STOP_DISCOVER "Stop Discovery"
#define STRING_TERMINATE_STREAM "Terminate Stream"
#define STRING_CALL_ON_HOLD "Call on hold....."
#define STRING_ON_GOING_CALL "Ongoing call....."
#define STRING_STOP_ADVERTISEMENT "Stop Adv"
#define STRING_START_ADVERTISEMENT "Start Adv"
#define STRING_PLAY "Play"
#define STRING_PAUSE "Pause"
#define STRING_MUTE "Mute"
#define STRING_UNMUTE "Unmute"
#define STRING_CONNECT "Connect"
#define STRING_DISCONNECT "Disconnect"
#define STRING_REMOTE_RETRIEVE "Remote Retrieve"
#define STRING_REMOTE_HOLD "Remote Hold"

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

//uint8_t broadcast_supported_config[] = {BAP_CODEC_CONFIG_16_2_2, BAP_CODEC_CONFIG_48_2_2, BAP_CODEC_CONFIG_48_4_2, BAP_CODEC_CONFIG_48_6_2};

bool fileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}


// Helper function
uint16_t le_audio_update_conn_id(CBtDevice *p_device)
{
    uint16_t conn_id = 0;

    if((le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE)|
        (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK) |
        (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER)|
        (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT) |
        (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT))
    {
        conn_id = p_device->get_connection_handle();
    }

    return conn_id;
}

// Initialize app
void MainWindow::InitUnicastSink()
{
    ui->tabLEAudio->setEnabled(TRUE);
    le_audio_reset_ui();
}

void MainWindow::retrieve_call_on_hold_popup(uint16_t conn_id, uint8_t call_id)
{
    QMessageBox retrieveBox(this);

    retrieveBox.setIconPixmap(QPixmap(":/callhold.png"));
    retrieveBox.setStyleSheet("QLabel{width: 140px; min-width: 50px; max-width: 200px;}");

    retrieveBox.setText(STRING_CALL_ON_HOLD);

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

    popupBox.setText(STRING_ON_GOING_CALL);

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

void MainWindow::le_audio_handle_incoming_call(uint16_t conn_id, uint8_t *p_data)
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

void MainWindow::le_audio_handle_update_call_friendly_name(uint8_t *p_data)
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

void MainWindow::le_audio_handle_update_call_remote_hold_state(uint8_t *p_data)
{
    uint8_t call_id;
    STREAM_TO_UINT8(call_id, p_data);
    Log("[%s] remote hold, call_id %d", TAG, call_id);
}

void MainWindow::on_btn_le_audio_startLeAdv_clicked()
{
    QString text = ui->btn_le_audio_startLeAdv->text();

    if (text == STRING_START_ADVERTISEMENT)
    {
        app_host_gatt_start_stop_advert(TRUE);
        ui->btn_le_audio_startLeAdv->setText(STRING_STOP_ADVERTISEMENT);
    }
    else if (text == STRING_STOP_ADVERTISEMENT)
    {
        app_host_gatt_start_stop_advert(FALSE);
        ui->btn_le_audio_startLeAdv->setText(STRING_START_ADVERTISEMENT);
    }
}

void MainWindow::on_btn_le_audio_hs_setMediaPlayer_clicked()
{
    CBtDevice * p_device = GetSelectedLEDevice();
    uint8_t player_name[50];
    memset(player_name, 0, 50);

    if (p_device == NULL)
        return;

    QString player = ui->le_audio_hs_playerlist->currentText();

    if (player.isNull())
        return;

    memcpy(&player_name, player.toStdString().c_str(), player.size());
    int len = player.size();
    Log("[%s] player %s", TAG, player_name);
    app_host_le_audio_set_media_player(p_device->get_connection_handle(), len, player_name);
}

void MainWindow::HandleLEAudioRequestEvent(uint8_t *pu8Data, int len)
{
    if(len < 3)
    {
        Log("HandleLEAudioRequestEvent bad length");
        return;
    }

    if(!m_audio_started ){
        Log("[%s] audio stopped",__FUNCTION__);
        return;
    }


    int bytes_per_packet = pu8Data[0] | (pu8Data[1] << 8);
    int num_packets = pu8Data[2];

    //Log("pkt len %d bytes %d packets",bytes_per_packet, num_packets);

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

void MainWindow::HandleLEAudioStartEvent(uint8_t *p_data, int len)
{
    UNUSED(p_data);
    UNUSED(len);

    if (!m_audio_started)
    {
        Log("Audio started");
        m_audio_format  = 0; // wav

        Log("Initializing audio file");
        if (!InitializeAudioFile(ui->le_audio_pl_LeAudioFile->text()))
        {
            return;
        }
        m_uAudio.m_PacketsSent = 0;
        m_uAudio.m_PacketsToSend = 0;
        set_audio_started_status(true);

        Log("Initialized audio file: %s", ui->le_audio_pl_LeAudioFile->text().toStdString().c_str());
        //DisableAppTraces();
    }
}
void MainWindow::HandleLEAudioStopEvent(uint8_t *p_data, int len)
{
    UNUSED(p_data);
    UNUSED(len);

    m_audio_started = false;
    //EnableAppTraces();
    Log("Audio stopped");
}

void MainWindow::on_btn_le_audio_pl_findFile_clicked()
{
    QString fileName;
    fileName = QFileDialog::getOpenFileName(this, tr("Open Audio File"),
        "", tr("Audio Files (*.wav)"));
    ui->le_audio_pl_LeAudioFile->setText(fileName);
    m_settings.setValue("LEAudioFile",fileName);
}

void MainWindow::on_btn_le_audio_hs_playPause_clicked()
{
    QString text = ui->btn_le_audio_hs_playPause->text();

    if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK)
    {
        CBtDevice *p_device = GetSelectedLEDevice();
        if (p_device == NULL){
            Log("[%s] Headset play/pause pressed, no connection...", TAG);
            return;
        }

        Log("[%s] Headset %s pressed ", TAG, text.toStdString().c_str());
        if (text == STRING_PLAY)
        {
            if(p_device->is_ready_to_play()){
                app_host_le_audio_command(p_device->get_connection_handle(), WICED_LE_AUDIO_PLAY_CMD, NULL);
                ui->btn_le_audio_hs_playPause->setText(STRING_PAUSE);
            }else{
                Log("[%s] not ready to play", TAG);
            }
        }
        else if (text == STRING_PAUSE)
        {
            app_host_le_audio_command(p_device->get_connection_handle(), WICED_LE_AUDIO_PAUSE_CMD, NULL);
            ui->btn_le_audio_hs_playPause->setText(STRING_PLAY);
        }
    }
}


void MainWindow::on_btn_le_audio_pl_playPause_clicked()
{
    QString text = ui->btn_le_audio_pl_playPause->text();

    Log("[%s] Player PlayPause pressed",TAG);
    if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE)
    {
        CBtDevice *p_device = GetSelectedLEDevice();
        if (p_device == NULL){
            Log("[%s] Player PlayPause pressed, no device found",TAG);
            return;
        }


        if (text == STRING_PLAY)
        {
            if(le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE){

                if(ui->cbCommport->currentText() != "host-mode"){
                    /* invoke only if !host-mode */
                    if(!fileExists(ui->le_audio_pl_LeAudioFile->text())){
                        on_btn_le_audio_pl_findFile_clicked();
                    }
                }
            }

            if(p_device->is_ready_to_play())
            {
                uint32_t codec_config = ui->le_audio_pl_list_codecConfig->itemData(ui->le_audio_pl_list_codecConfig->currentIndex()).toInt();
                m_settings.setValue("LEAudioCodecConfig",ui->le_audio_pl_list_codecConfig->currentText());

                app_host_le_audio_command(p_device->get_connection_handle(), WICED_LE_AUDIO_PLAY_CMD, &codec_config);
                ui->btn_le_audio_pl_playPause->setText(STRING_PAUSE);
            }
        }
        else if (text == STRING_PAUSE)
        {
            app_host_le_audio_command(p_device->get_connection_handle(), WICED_LE_AUDIO_PAUSE_CMD, NULL);
            ui->btn_le_audio_pl_playPause->setText(STRING_PLAY);
        }
    }
}

void MainWindow::handle_bt_le_audio_muteUnmute_btn(QPushButton* p_btn)
{
    QString text = p_btn->text();
    CBtDevice *p_device = GetSelectedLEDevice();
    uint16_t conn_id = 0;

    if(p_device){
        conn_id =p_device->get_connection_handle();
    }

    if (text == STRING_MUTE)
    {
        p_btn->setText(STRING_UNMUTE);
        app_host_le_audio_command(conn_id, WICED_LE_AUDIO_MUTE_CMD, NULL);
    }
    else if (text == STRING_UNMUTE)
    {
        p_btn->setText(STRING_MUTE);
        app_host_le_audio_command(conn_id, WICED_LE_AUDIO_UNMUTE_CMD, NULL);
    }
}

void MainWindow::on_btn_le_audio_hs_muteUnmute_clicked()
{
    handle_bt_le_audio_muteUnmute_btn(ui->btn_le_audio_hs_muteUnmute);
}

void MainWindow::on_btn_le_audio_pl_muteUnmute_clicked()
{
    handle_bt_le_audio_muteUnmute_btn(ui->btn_le_audio_hs_muteUnmute);
}

void MainWindow::handle_le_audio_volUpDown_clicked_btn(int cmd)
{
    CBtDevice *p_device = GetSelectedLEDevice();
    uint16_t conn_id = 0;

    if(p_device)
        conn_id = le_audio_update_conn_id(p_device);

    app_host_le_audio_command(conn_id, cmd, NULL);
}

void MainWindow::on_btn_le_audio_hs_volDown_clicked()
{
    if (ui->btn_le_audio_hs_muteUnmute->text() == STRING_UNMUTE){
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_DOWN_CMD);
    }else{
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_VOL_DOWN_CMD);
    }

}

void MainWindow::on_btn_le_audio_pl_volDown_clicked()
{
    if (ui->btn_le_audio_pl_muteUnmute->text() == STRING_UNMUTE){
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_DOWN_CMD);
    }else{
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_VOL_DOWN_CMD);
    }

}

void MainWindow::on_btn_le_audio_hs_volUp_clicked()
{
    if (ui->btn_le_audio_hs_muteUnmute->text() == STRING_UNMUTE){
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_UP_CMD);
    }else{
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_VOL_UP_CMD);
    }
}

void MainWindow::on_btn_le_audio_pl_volUp_clicked()
{
    if (ui->btn_le_audio_pl_muteUnmute->text() == STRING_UNMUTE){
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_UNMUTE_RELATIVE_VOL_UP_CMD);
    }else{
        handle_le_audio_volUpDown_clicked_btn(WICED_LE_AUDIO_VOL_UP_CMD);
    }
}

void MainWindow::handle_le_audio_absVol_btn(QString absvol_str)
{
    int absvol = absvol_str.toInt();

    CBtDevice *p_device = GetSelectedLEDevice();
    uint16_t conn_id = 0;

    if(p_device)
        conn_id = le_audio_update_conn_id(p_device);

    app_host_le_audio_set_abs_volume(conn_id, absvol > 255 ? 255 : absvol);
}

void MainWindow::on_btn_le_audio_hs_absVol_clicked()
{
    handle_le_audio_absVol_btn(ui->le_audio_hs_absVal->text());
}

void MainWindow::on_btn_le_audio_pl_absVol_clicked()
{
    handle_le_audio_absVol_btn(ui->le_audio_pl_absVal->text());
}

void MainWindow::le_audio_handle_update_media_player_list(uint8_t len, uint8_t *p_data)
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

        int index = ui->le_audio_hs_playerlist->findText(player, Qt::MatchStartsWith);
        if (len != 0 && index == -1)
        {
            ui->le_audio_hs_playerlist->addItem(player);
        }
    }
}

void MainWindow::le_audio_handle_update_media_player_status(uint8_t *p_data)
{
    uint8_t status;
    QString new_btn_text = STRING_PLAY;

    STREAM_TO_UINT8(status, p_data);
    LogMediaState(status);
    if (status == 1)
    {
        // playing
        new_btn_text = STRING_PAUSE;
    }

    if(is_le_audio_headset()){
        ui->btn_le_audio_hs_playPause->setText(new_btn_text);
    }else{
        ui->btn_le_audio_pl_playPause->setText(new_btn_text);
    }
}

static name_val_t le_audio_options[] =
{
    {"8_1_1 (7.5ms - 16 kbps-LL)",BAP_CODEC_CONFIG_8_1_1},
    {"8_1_2 (7.5ms - 16 kbps-HR)",BAP_CODEC_CONFIG_8_1_2},
    {"8_2_1 (10ms - 16 kbps-LL)",BAP_CODEC_CONFIG_8_2_1},
    {"8_2_2 (10ms - 16 kbps-HR)",BAP_CODEC_CONFIG_8_2_2},
    {"16_1_1 (7.5ms - 32 kbps-LL)",BAP_CODEC_CONFIG_16_1_1},
    {"16_1_2 (7.5ms - 32 kbps-HR)",BAP_CODEC_CONFIG_16_1_2},
    {"16_2_1 (10ms - 32 kbps-LL)",BAP_CODEC_CONFIG_16_2_1},
    {"16_2_2 (10ms - 32 kbps-HR)",BAP_CODEC_CONFIG_16_2_2},
    {"24_1_1 (7.5ms - 48 kbps-LL)",BAP_CODEC_CONFIG_24_1_1},
    {"24_1_2 (7.5ms - 48 kbps)-HR",BAP_CODEC_CONFIG_24_1_2},
    {"24_2_1 (10ms - 48 kbps-LL)",BAP_CODEC_CONFIG_24_2_1},
    {"24_2_2 (10ms - 48 kbps-HR)",BAP_CODEC_CONFIG_24_2_2},
    {"32_1_1 (7.5ms - 64 kbps-LL)",BAP_CODEC_CONFIG_32_1_1},
    {"32_1_2 (7.5ms - 64 kbps-HR)",BAP_CODEC_CONFIG_32_1_2},
    {"32_2_1 (10ms - 64 kbps-LL)",BAP_CODEC_CONFIG_32_2_1},
    {"32_2_2 (10ms - 64 kbps-HR)",BAP_CODEC_CONFIG_32_2_2},
    {"441_1_1 (7.5ms - 16 kbps-LL)",BAP_CODEC_CONFIG_441_1_1},
    {"441_1_2 (7.5ms - 16 kbps-HR)",BAP_CODEC_CONFIG_441_1_2},
    {"441_2_1 (10ms - 16 kbps-LL)",BAP_CODEC_CONFIG_441_2_1},
    {"441_2_2 (10ms - 16 kbps-HR)",BAP_CODEC_CONFIG_441_2_2},
    {"48_1_1 (7.5ms - 80 kbps-LL)",BAP_CODEC_CONFIG_48_1_1},
    {"48_1_2 (7.5ms - 80 kbps-HR)",BAP_CODEC_CONFIG_48_1_2},
    {"48_2_1 (10ms - 80 kbps-LL)",BAP_CODEC_CONFIG_48_2_1},
    {"48_2_2 (10ms - 80 kbps-HR)",BAP_CODEC_CONFIG_48_2_2},
    {"48_3_1 (7.5ms - 96 kbps-LL)",BAP_CODEC_CONFIG_48_3_1},
    {"48_3_2 (7.5ms - 96 kbps-HR)",BAP_CODEC_CONFIG_48_3_2},
    {"48_4_1 (10ms - 96 kbps-LL)",BAP_CODEC_CONFIG_48_4_1},
    {"48_4_2 (10ms - 96 kbps-HR)",BAP_CODEC_CONFIG_48_4_2},
    {"48_5_1 (7.5ms - 96 kbps-LL)",BAP_CODEC_CONFIG_48_5_1},
    {"48_5_2 (7.5ms - 96 kbps-HR)",BAP_CODEC_CONFIG_48_5_2},
    {"48_6_1 (10ms - 96 kbps-LL)",BAP_CODEC_CONFIG_48_6_1},
    {"48_6_2 (10ms - 96 kbps-HR)",BAP_CODEC_CONFIG_48_6_2}
};

void MainWindow::initCodecConfigOptions(QComboBox *p_box, const name_val_t *p_options, int num, int enable)
{
    p_box->clear();
    if(!enable){
        return;
    }

    QString previous = "";
    if(m_settings.value("LEAudioCodecConfig").toString().length()){
        previous = m_settings.value("LeAudioCodecConfig").toString();
    }
    int prev_index = 0;

    for(int index = 0; num--;p_options++, index++){
        p_box->addItem(p_options->name, QVariant::fromValue(p_options->data));
        if(previous == p_options->name){
            prev_index = index;
        }
    }

    p_box->setCurrentIndex(prev_index);
}

void MainWindow::init_le_audio_player_codec_config_options(int enable)
{
    initCodecConfigOptions(ui->le_audio_pl_list_codecConfig, le_audio_options, sizeof(le_audio_options)/sizeof(le_audio_options[0]), enable);
}

void MainWindow::init_le_audio_player_bis_codec_config_options(int enable)
{
    init_le_audio_player_codec_config_options(enable);
}

void MainWindow::initWidgets(QWidget ** p_list, int num, int enable, int hide = 0)
{
    for(;num--;p_list++){
        if(*p_list){
            if(hide == 1){
                (*p_list)->hide();
            }else{
                (*p_list)->show();
                (*p_list)->setEnabled(enable);
            }
        }
    }
}

void MainWindow::InitLeAudioHeadsetMediaControl(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_hs_mediaControl,
        ui->btn_le_audio_hs_setMediaPlayer,
        ui->btn_le_audio_hs_playPause,
        ui->le_audio_hs_playerlist
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);

    ui->btn_le_audio_hs_playPause->setText(STRING_PLAY);
}

void MainWindow::InitLeAudioHeadsetVolumeControl(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_hs_volControl,
        ui->btn_le_audio_hs_muteUnmute,
        ui->btn_le_audio_hs_volDown,
        ui->btn_le_audio_hs_volUp,
        ui->btn_le_audio_hs_absVol
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);
    ui->btn_le_audio_hs_muteUnmute->setText(STRING_MUTE);

    ui->le_audio_hs_absVal->setValidator(new QIntValidator(0, 255, this));
}

void MainWindow::InitLeAudioPlayerMediaControl(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_pl_mediaControl,
        ui->btn_le_audio_pl_playPause
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);
    ui->le_audio_hs_playerlist->clear();
    ui->btn_le_audio_hs_playPause->setText(STRING_PLAY);
}

void MainWindow::InitLeAudioPlayerMediaFile(int enable)
{
    QWidget * p_list_media[] = {
        ui->groupBox_le_audio_pl_media,
        ui->le_audio_pl_list_codecConfig,
    };

    initWidgets(p_list_media, sizeof(p_list_media)/sizeof(p_list_media[0]), enable);

    QWidget *p_list_file[] = {
        ui->label_le_audio_pl_LeAudioFile,
        ui->le_audio_pl_LeAudioFile,
        ui->btn_le_audio_pl_findFile,
    };

    if(ui->cbCommport->currentText() == "host-mode"){
        /* Do not enable for host-mode */
        initWidgets(p_list_file, sizeof(p_list_file)/sizeof(p_list_file[0]), 0, 0);
    }else{
        initWidgets(p_list_file, sizeof(p_list_file)/sizeof(p_list_file[0]), enable);
    }

    init_le_audio_player_codec_config_options(enable);
}

void MainWindow::InitLeAudioPlayerVolumeControl(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_pl_volControl,
        ui->btn_le_audio_pl_muteUnmute,
        ui->btn_le_audio_pl_volDown,
        ui->btn_le_audio_pl_volUp,
        ui->btn_le_audio_pl_absVol
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);
    ui->btn_le_audio_pl_muteUnmute->setText(STRING_MUTE);

    ui->le_audio_pl_absVal->setValidator(new QIntValidator(0, 255, this));
}

void MainWindow::InitLeAudioPlayerCallControlSetup(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_pl_callControlSetup,
        ui->le_audio_pl_call_friendlyName,
        ui->le_audio_pl_callURI
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);

    if(m_settings.value("LEAudioCallFriendlyName").toString().length()){
        ui->le_audio_pl_call_friendlyName->setText(m_settings.value("LEAudioCallFriendlyName").toString());
    }
    if(m_settings.value("LEAudioCallURI").toString().length()){
        ui->le_audio_pl_callURI->setText(m_settings.value("LEAudioCallURI").toString());
    }

}

void MainWindow::InitLeAudioPlayerCallControls(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_pl_callControls,
        ui->btn_le_audio_pl_simulateCall,
        ui->btn_le_audio_pl_simulateHold,
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);
}


void MainWindow::InitLeAudioPlayerBroadcastSource(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_pl_bcastSource,
        ui->le_audio_pl_bis_channelCount,
        ui->le_audio_pl_bisCount,
        ui->le_audio_pl_bisEncrypt,
        ui->le_audio_pl_bcastCode,
        ui->le_audio_pl_bcastID
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);
    init_le_audio_player_bis_codec_config_options(enable);

    ui->le_audio_pl_bcastCode->setText("12345678");
    ui->le_audio_pl_bcastID->setText("1234");
}

void MainWindow::InitLeAudioPlayerBroadcastSourceControls(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_pl_bcastSourceControl,
        ui->btn_le_audio_pl_startBcast
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);
    init_le_audio_player_bis_codec_config_options(enable);
    ui->btn_le_audio_pl_startBcast->setText(STRING_START_BROADCAST);
}

void MainWindow::InitLeAudioBroadcastAssistant(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_bcastAssistant,
        ui->btn_le_audio_ba_scan_bcastStreams,
        ui->le_audio_ba_found_bcastStreams,
        ui->le_audio_ba_bcastCode,
        ui->btn_le_audio_ba_addSource,
        ui->btn_le_audio_ba_addSourceWithPast,
        ui->btn_le_audio_ba_removeSource
    };

    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);

    ui->btn_le_audio_ba_scan_bcastStreams->setText(STRING_DISCOVER_SOURCES);
    ui->le_audio_ba_found_bcastStreams->clear();
}

void MainWindow::InitLeAudioHeadsetBroadcastSink(int enable)
{
    QWidget * p_list[] = {
        ui->groupBox_le_audio_hs_bcastSink,
        ui->btn_le_audio_hs_scan_bcastStreams,
        ui->btn_le_audio_hs_bcast_SyncToStream,
        ui->le_audio_hs_found_bcastStreams
    };
    initWidgets(p_list, sizeof(p_list)/sizeof(p_list[0]), enable);

    if(!enable){
        ui->le_audio_hs_found_bcastStreams->clear();
    }

    ui->btn_le_audio_hs_scan_bcastStreams->setText(STRING_DISCOVER_SOURCES);
    ui->btn_le_audio_hs_bcast_SyncToStream->setText(STRING_SYNC_TO_STREAM);
    ui->le_audio_hs_found_bcastStreams->clear();
}

QString MainWindow::add_text_with_delimiter(QString text, QString add, QString delimiter = ", ")
{
    if(text.length()){
        return delimiter + add;
    }

    return add;
}

void MainWindow::UpdateLEAudioRole()
{
    uint32_t audio_role = le_audio_dev_role;
    uint32_t mask_bit = 0;
    QString text = "";

    if(audio_role){
        ui->groupBox_leaudio_connect->setEnabled(TRUE);
    }

    if(audio_role & (HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE | HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE)){
        InitLeAudioPlayerMediaFile(TRUE);
    }

    do
    {
        if(!(audio_role >> mask_bit)){
            break;
        }

        switch (audio_role & (1 << mask_bit))
        {
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE:
            text += add_text_with_delimiter(text, "Unicast Source");

            InitLeAudioPlayerMediaControl(TRUE);
            InitLeAudioPlayerVolumeControl(TRUE);

            break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK:
            text += add_text_with_delimiter(text, "Unicast Sink");

            InitLeAudioHeadsetMediaControl(TRUE);
            InitLeAudioHeadsetVolumeControl(TRUE);
            break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER:
            text += add_text_with_delimiter(text, "Call Control Server (Phone)");
            InitLeAudioPlayerCallControlSetup(TRUE);
            InitLeAudioPlayerCallControls(TRUE);

            break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT:
            text += add_text_with_delimiter(text, "Call Control Client (EB)");
            break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE:
            text += add_text_with_delimiter(text, "Broadcast Source");
            InitLeAudioPlayerBroadcastSource(TRUE);
            InitLeAudioPlayerBroadcastSourceControls(TRUE);

            break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK:
            text += add_text_with_delimiter(text, "Broadcast Sink");

            InitLeAudioHeadsetBroadcastSink(TRUE);
            InitLeAudioHeadsetVolumeControl(TRUE);
            break;
        case HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT:
            text += add_text_with_delimiter(text, "Broadcast Assistant");
            InitLeAudioBroadcastAssistant(TRUE);
            break;
        }
    }while(++mask_bit < 32);

    if(m_settings.value("LEAudioFile").toString().length()){
        ui->le_audio_pl_LeAudioFile->setText(m_settings.value("LEAudioFile").toString());
    }

    ui->le_audio_deviceRoleTxt->setText("Device Role: " + text);
    ui->btn_le_audio_startLeAdv->setText(STRING_START_ADVERTISEMENT);

    if(le_audio_dev_role & 0xf){
        ui->tabLeAudioRole->setCurrentWidget(ui->tabLEPlayer);
    }else if(le_audio_dev_role & 0x80){
        ui->tabLeAudioRole->setCurrentWidget(ui->tabLEBroadcastAssistant);
    }else {
        ui->tabLeAudioRole->setCurrentWidget(ui->tabLEHeadset);
    }
}

void MainWindow::LogMediaState(int state)
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

bool MainWindow::is_le_audio_headset()
{
    if((le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK)||
            (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK))
    {
        return TRUE;
    }

    return FALSE;
}

bool MainWindow::is_le_audio_player()
{
    if((le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE)||
            (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE))
    {
        return TRUE;
    }

    return FALSE;
}

void MainWindow::le_audio_handle_update_mute_state(int state)
{
    QString state_text = STRING_MUTE;

    if (state == 1)
    {
        state_text = STRING_UNMUTE;
    }

    if(is_le_audio_headset())
    {
        ui->btn_le_audio_hs_muteUnmute->setText(state_text);
    }else{
        ui->btn_le_audio_pl_muteUnmute->setText(state_text);
    }
}

const name_val_t le_audio_app_status[] = {
    {"connected", HCI_CONTROL_LEA_APP_STATE_CONNECTED},
    {"mtu_configured",HCI_CONTROL_LEA_APP_STATE_MTU_CONFIGURED},
    {"discovery_complete",HCI_CONTROL_LEA_APP_STATE_DISCOVERY_COMPLETE},
    {"init", HCI_CONTROL_LEA_APP_STATE_INITING},
    {"ready",HCI_CONTROL_LEA_APP_STATE_READY},
    {"disconnecting", HCI_CONTROL_LEA_APP_STATE_DISCONNECTING},
    {"disconnected", HCI_CONTROL_LEA_APP_STATE_DISCONNECTED}
};

const name_val_t le_audio_app_sub_state[] = {
    {"notifying", HCI_CONTROL_LEA_APP_STATE_INIT_NOTIFYING},
    {"enabling",HCI_CONTROL_LEA_APP_STATE_INIT_ENABLING},
    {"reading",HCI_CONTROL_LEA_APP_STATE_INIT_READING}
};


void MainWindow::le_audio_handle_app_status_events(BYTE *p_data, DWORD len)
{
    uint16_t handle = -1;
    uint32_t app_status = -1;
    uint32_t sub_status = -1;
    uint8_t uuid_len = 0;
    uint32_t uuid_32 = 0;

    const char *app_status_str, *sub_status_str,*device_addr_str = "no_device";

    if(len >= 10){
        STREAM_TO_UINT16(handle, p_data);
        STREAM_TO_UINT32(app_status, p_data);
        STREAM_TO_UINT32(sub_status, p_data);
        // Read UUID
        {
            STREAM_TO_UINT8(uuid_len, p_data);
            if(uuid_len == 2)
            {
                STREAM_TO_UINT16(uuid_32, p_data);
            }
            else if (uuid_len == 4)
            {
                STREAM_TO_UINT32(uuid_32, p_data);
            }
        }
    }

    CBtDevice *p_device = FindInList(CONNECTION_TYPE_LE, handle, ui->cbBLEDeviceList);
    if(p_device)
        device_addr_str = p_device->get_bdaddr_string();
    app_status_str = get_name_for_val(le_audio_app_status, sizeof(le_audio_app_status)/sizeof(le_audio_app_status[0]), app_status, "unknown");

    switch(app_status)
    {
    case HCI_CONTROL_LEA_APP_STATE_CONNECTED:
    case HCI_CONTROL_LEA_APP_STATE_MTU_CONFIGURED:
    case HCI_CONTROL_LEA_APP_STATE_DISCOVERY_COMPLETE:
    case HCI_CONTROL_LEA_APP_STATE_DISCONNECTING:
    case HCI_CONTROL_LEA_APP_STATE_DISCONNECTED:
    case HCI_CONTROL_LEA_APP_STATE_READY:
        Log("%s app status %s %s 0x%x", TAG, device_addr_str, app_status_str, sub_status);
    break;
    case HCI_CONTROL_LEA_APP_STATE_INITING:
        sub_status_str = get_name_for_val(le_audio_app_sub_state, sizeof(le_audio_app_sub_state)/sizeof(le_audio_app_sub_state[0]), sub_status, "unknown");
        Log("%s app status %s %s %s 0x%04x", TAG, device_addr_str, app_status_str, sub_status_str, uuid_32);
        break;
    }

    if(p_device){
        if(app_status == HCI_CONTROL_LEA_APP_STATE_READY){
                p_device->set_ready_to_play(1);
        }else{
            p_device->set_ready_to_play(0);
        }
    }
}

void MainWindow::le_audio_handle_le_events(DWORD opcode, BYTE *rx_buf, DWORD len)
{
    UNUSED(rx_buf);
    UNUSED(len);

    switch(opcode)
    {
    case HCI_CONTROL_LE_EVENT_CONNECTED:
        ui->btn_le_audio_connectToPeer->setText(STRING_DISCONNECT);
        break;
    case HCI_CONTROL_LE_EVENT_DISCONNECTED:
        ui->btn_le_audio_connectToPeer->setText(STRING_CONNECT);
        break;
    }
}

// Handle WICED HCI events for AV sink
void MainWindow::le_audio_handle_audio_events(DWORD opcode, BYTE *rx_buf, DWORD len)
{
    uint8_t status;
    uint16_t conn_id;
    uint8_t mute;
    uint8_t *p_data = rx_buf;

    if (opcode == HCI_CONTROL_LE_AUDIO_EVENT_REQUEST_DATA)
    {
        HandleLEAudioRequestEvent(p_data, len);
        return;
    }else if(opcode == HCI_CONTROL_LE_AUDIO_EVENT_STARTED)
    {
        Log("[%s] audio started event", TAG);
        HandleLEAudioStartEvent(p_data, len);
        return;
    }else if(opcode == HCI_CONTROL_LE_AUDIO_EVENT_STOPPED)
    {
        Log("[%s] audio stopped event", TAG);
        HandleLEAudioStopEvent(p_data, len);
        return;
    }

    if (len <= 0) // HCI_CONTROL_LE_AUDIO_EVENT_STARTED & HCI_CONTROL_LE_AUDIO_EVENT_STOPPED events are supposed to come with len 0
        return;


    if (opcode == HCI_CONTROL_LE_AUDIO_EVENT_DEVICE_ROLE)
    {
        EnableTabs(HCI_CONTROL_GROUP_LE_AUDIO, true);
        STREAM_TO_UINT32(le_audio_dev_role, p_data);
        Log("[%s] device role %x", TAG, le_audio_dev_role);
        UpdateLEAudioRole();
        return;
    }

    switch (opcode)
    {
    case HCI_CONTROL_LE_AUDIO_EVENT_APP_STATUS:
        le_audio_handle_app_status_events(p_data, len);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_MEDIA_PLAYER:
        STREAM_TO_UINT16(conn_id, p_data);
        le_audio_handle_update_media_player_list(len, p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_PLAY_STATUS:
        STREAM_TO_UINT16(conn_id, p_data);
        le_audio_handle_update_media_player_status(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_VOLUME_STATUS:
        STREAM_TO_UINT16(conn_id, p_data);
        STREAM_TO_UINT8(status, p_data);
        Log("[%s] Volume status %d", TAG, status);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_MUTE_STATUS:
        STREAM_TO_UINT16(conn_id, p_data);
        STREAM_TO_UINT8(mute, p_data);
        Log("[%s] Mute status %x", TAG, mute);
        le_audio_handle_update_mute_state(mute);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_MUTE_AND_VOLUME_STATUS:
        STREAM_TO_UINT16(conn_id, p_data);
        STREAM_TO_UINT8(status, p_data);
        STREAM_TO_UINT8(mute, p_data);
        Log("[%s] Volume status %d mute status %d", TAG, status, mute);
        le_audio_handle_update_mute_state(mute);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_INCOMING_CALL:
        incoming_call_cnt++;
        STREAM_TO_UINT16(conn_id, p_data);
        le_audio_handle_incoming_call(conn_id, p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_BROADCAST_STREAM_RSP:
        le_audio_handle_broadcast_stream_response_data(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_BROADCAST_STATUS_UPDATE:
        le_audio_handle_broadcast_status_update(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_CALL_FRIENDLY_NAME:
        STREAM_TO_UINT16(conn_id, p_data);
        le_audio_handle_update_call_friendly_name(p_data);
        break;
    case HCI_CONTROL_LE_AUDIO_EVENT_REMOTE_HOLD_CALL:
        STREAM_TO_UINT16(conn_id, p_data);
        le_audio_handle_update_call_remote_hold_state(p_data);
        break;
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventLeAudio(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if(!ui->tabLEAudio->isEnabled())
    {
        return;
    }

    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_LE_AUDIO:
        le_audio_handle_audio_events(opcode, p_data, len);
        break;
    case HCI_CONTROL_GROUP_LE:
        le_audio_handle_le_events(opcode, p_data, len);
        break;
    }

}

// Add new device to combo box
CBtDevice *MainWindow::AddDeviceToListLeAudio(BYTE *addr, QComboBox *pCb, char *bd_name, uint16_t conn_id)
{
    CBtDevice *device = nullptr;
    char abuffer[100] = {0};
    int used =0;

    used += get_bd_string(addr, abuffer, sizeof(abuffer));
    if (bd_name)
    {
        snprintf(abuffer + used, sizeof(abuffer) - used, ", [%s]", bd_name);
    }

    // Check if device is already present
    int i = pCb->findText(abuffer, Qt::MatchStartsWith);
    if (i == -1)
    {
        QVariant qv;
        device = new CBtDevice(conn_id);
        if (bd_name && strlen(bd_name))
            strncpy(device->m_name, bd_name, sizeof(device->m_name) - 1);

        qv.setValue<CBtDevice *>(device);

        pCb->addItem(abuffer, qv);
        i = pCb->findText(abuffer, Qt::MatchStartsWith);

        Log("Added %s to index %d", device->get_bdaddr_string(), i);

        memcpy(device->m_address, addr, 6);
    }
    return device;
}

void MainWindow::on_btn_le_audio_connectToPeer_clicked()
{
    CBtDevice *p_device = GetSelectedLEDevice();

    if (p_device == NULL)
        return;

    QString connText = ui->btn_le_audio_connectToPeer->text();

    if (connText == STRING_CONNECT)
    {
        if((p_device->get_connection_handle() == 0) && (p_device->connect_in_progress() == 0)){
            Log("LeConnect BtDevice : %s",p_device->get_bdaddr_string());

            IssueConnect(p_device);
        }else{
            Log("LeConnect already issued connect request to BtDevice : %s",p_device->get_bdaddr_string());
            //app_host_gatt_cancel_connect(p_device->address_type, p_device->m_address);
        }
    }else {
        if(p_device->get_connection_handle()){
            app_host_gatt_le_disconnect(p_device->get_connection_handle());
        }
    }

}

void MainWindow::on_btn_le_audio_pl_simulateCall_clicked()
{
    ui->btn_le_audio_pl_simulateHold->setEnabled(TRUE);
    app_host_le_audio_generate_call(0x8000, ui->le_audio_pl_callURI->text().toStdString().length(),
                                    (uint8_t *)ui->le_audio_pl_callURI->text().toStdString().c_str(),
                                    ui->le_audio_pl_call_friendlyName->text().toStdString().length(),
                                    (uint8_t *)ui->le_audio_pl_call_friendlyName->text().toStdString().c_str());
    m_settings.setValue("LEAudioCallFriendlyName", ui->le_audio_pl_call_friendlyName->text());
    m_settings.setValue("LEAudioCallURI", ui->le_audio_pl_callURI->text());
}

void MainWindow::on_btn_le_audio_pl_simulateHold_clicked()
{
    QString state = ui->btn_le_audio_pl_simulateHold->text();

    if (state == STRING_REMOTE_HOLD)
    {
        wiced_hci_set_rmt_call_hold(CALL_ID);
        ui->btn_le_audio_pl_simulateHold->setText(STRING_REMOTE_RETRIEVE);
    }
    else if(state == STRING_REMOTE_RETRIEVE)
    {
       wiced_hci_set_rmt_hold_retrieve(CALL_ID);
       ui->btn_le_audio_pl_simulateHold->setText(STRING_REMOTE_HOLD);
    }
}

void MainWindow::on_btn_le_audio_pl_startBcast_clicked()
{
    QByteArray broadcast_code = QByteArray::fromHex(ui->le_audio_pl_bcastCode->text().rightJustified(32, '0').toLatin1());
    uint32_t broadcast_id = 0;
    uint32_t encryption = 0;
    uint32_t num_channels = 0;
    uint8_t bis_count =0;
    uint32_t codec_config = 0;
    bool status = FALSE;

    if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE)
    {
        if (ui->btn_le_audio_pl_startBcast->text() == STRING_START_BROADCAST)
        {
            if(ui->cbCommport->currentText() != "host-mode"){
                /* invoke only if !host-mode */
                if(!fileExists(ui->le_audio_pl_LeAudioFile->text())){
                    on_btn_le_audio_pl_findFile_clicked();
                }
            }

            // get sampling frequencey, num of channels, encryption and start broadcast stream
            encryption = ui->le_audio_pl_bisEncrypt->isChecked();
            QString audio_type = ui->le_audio_pl_bis_channelCount->currentText();
            if (audio_type == "mono")
                {
                    num_channels = 1;
                    Log("mono stream");
                }
            else
                {
                    num_channels = 2;
                    Log("stereo stream");
                }
            bis_count = ui->le_audio_pl_bisCount->currentText().toInt();

            codec_config = ui->le_audio_pl_list_codecConfig->itemData(ui->le_audio_pl_list_codecConfig->currentIndex()).toInt();
            m_settings.setValue("LEAudioCodecConfig",ui->le_audio_pl_list_codecConfig->currentText());

            broadcast_id = ui->le_audio_pl_bcastID->text().rightJustified(6, '0').toInt(&status, 16);
            if(!status)
                return;

            app_host_le_audio_broadcast_source_start_streaming(1, codec_config, bis_count,
                                                               num_channels, encryption, broadcast_id, (uint8_t *)broadcast_code.data());
            Log("Starting %sencrypted streaming, num channels %d count %d cfg %d id 0x%06x code %s", encryption ? "":"un",
                num_channels, bis_count, codec_config, broadcast_id, ui->le_audio_pl_bcastCode->text().toStdString().c_str());
            ui->btn_le_audio_pl_startBcast->setText(QStringLiteral("Stop Broadcast"));
        }
        else
        {
            app_host_le_audio_broadcast_source_start_streaming(0, codec_config, bis_count, num_channels, encryption, broadcast_id, NULL);
            Log("Stopping Streaming...");
            ui->btn_le_audio_pl_startBcast->setText(QStringLiteral(STRING_START_BROADCAST));
        }
    }
}

void MainWindow::le_audio_handle_scan_bcastStreams_click(QPushButton *p_btn, QComboBox *p_streams)
{
    QString text = p_btn->text();

    if (text == (STRING_DISCOVER_SOURCES))
    {
        p_streams->clear();
        Log("Looking for broadcast source streams...");

        if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
        {
            app_host_le_audio_broadcast_sink_find_sources(START);
        }
        else if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT)
        {
            app_host_le_audio_broadcast_assistant_scan_source(START);
        }
        else
            return;

        p_btn->setText(STRING_STOP_DISCOVER);
    }
    else
    {
        Log("Stop Looking for broadcast source streams...");

        if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
        {
            app_host_le_audio_broadcast_sink_find_sources(STOP);
        }

        else if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT)
        {
            app_host_le_audio_broadcast_assistant_scan_source(STOP);
        }
        p_btn->setText(STRING_DISCOVER_SOURCES);
    }
}

void MainWindow::on_btn_le_audio_hs_scan_bcastStreams_clicked()
{
    le_audio_handle_scan_bcastStreams_click(ui->btn_le_audio_hs_scan_bcastStreams, ui->le_audio_hs_found_bcastStreams);
}

void MainWindow::on_btn_le_audio_hs_bcast_SyncToStream_clicked()
{
    QByteArray broadcast_code = QByteArray::fromHex(ui->le_audio_hs_bcastCode->text().rightJustified(32, '0').toLatin1());
    uint32_t broadcast_id = ui->le_audio_hs_found_bcastStreams->currentData().toUInt();
    QString src_text = ui->btn_le_audio_hs_bcast_SyncToStream->text();

    if (le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK)
    {
        if (src_text == STRING_SYNC_TO_STREAM)
        {
            app_host_le_audio_broadcast_sink_sync_to_stream(SYNC, (uint8_t *)broadcast_code.data(), broadcast_id);
            ui->btn_le_audio_hs_bcast_SyncToStream->setText(STRING_TERMINATE_STREAM);
        }
        else
        {
            app_host_le_audio_broadcast_sink_sync_to_stream(TERMINATE, (uint8_t *)broadcast_code.data(), broadcast_id);
            ui->btn_le_audio_hs_bcast_SyncToStream->setText(STRING_SYNC_TO_STREAM);
            ui->btn_le_audio_hs_scan_bcastStreams->setText(STRING_DISCOVER_SOURCES);
        }
    }
}

void MainWindow::le_audio_ba_handle_add_bcast_source(int add_stream, int use_past)
{
    CBtDevice *p_device = GetSelectedLEDevice();
    QByteArray broadcast_code = QByteArray::fromHex(ui->le_audio_ba_bcastCode->text().rightJustified(32, '0').toLatin1());

    if (p_device == NULL)
        return;

    uint16_t conn_id = le_audio_update_conn_id(p_device);
    uint32_t broadcast_id = ui->le_audio_ba_found_bcastStreams->currentData().toUInt();

    if(add_stream)
    {
        app_host_le_audio_broadcast_assistant_select_source(ADD, conn_id, (uint8_t *)broadcast_code.data(), broadcast_id, use_past);
    }
    else
    {
        app_host_le_audio_broadcast_assistant_select_source(REMOVE, conn_id, (uint8_t *)broadcast_code.data(), broadcast_id, 0);
    }

}

void MainWindow::on_btn_le_audio_ba_addSource_clicked()
{
    le_audio_ba_handle_add_bcast_source(1, 0);
}

void MainWindow::on_btn_le_audio_ba_addSourceWithPast_clicked()
{
    le_audio_ba_handle_add_bcast_source(1, 1);
}

void MainWindow::on_btn_le_audio_ba_removeSource_clicked()
{
    le_audio_ba_handle_add_bcast_source(0, 0);
}

void MainWindow::on_btn_le_audio_ba_scan_bcastStreams_clicked()
{
    le_audio_handle_scan_bcastStreams_click(ui->btn_le_audio_ba_scan_bcastStreams, ui->le_audio_ba_found_bcastStreams);
}

void MainWindow::le_audio_add_broadcast_stream_to_list(QComboBox * cb, uint32_t broadcast_id, QString br_name)
{
    QString br_id_name = QString("0x") + QString::number(broadcast_id, 16) + ", " + br_name;

    if(cb->findData(broadcast_id) == -1)
    {
        cb->addItem(br_id_name, broadcast_id);
        Log("Adding bcast device : %s", br_id_name.toStdString().c_str());
    }
}

void MainWindow::le_audio_handle_broadcast_stream_response_data(uint8_t *p_data)
{
    uint32_t broadcast_id;
    uint8_t len;
    char br_name[20];

    STREAM_TO_UINT32(broadcast_id, p_data);


    memset(br_name, 0, 20);
    STREAM_TO_UINT8(len, p_data);
    if(len >= 20)len = (20-1);
    memcpy(br_name, p_data, len);


    if(le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK){
        le_audio_add_broadcast_stream_to_list(ui->le_audio_hs_found_bcastStreams, broadcast_id, br_name);
    }else if(le_audio_dev_role & HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_ASSISTANT){
        le_audio_add_broadcast_stream_to_list(ui->le_audio_ba_found_bcastStreams, broadcast_id, br_name);
    }
}

static const name_val_t broadcast_status_val[]  = {
    { "PA Sync established", HCI_CONTROL_LEA_BROADCAST_PA_SYNC_ESTABLISHED },
    { "PA Sync lost", HCI_CONTROL_LEA_BROADCAST_PA_SYNC_LOST },
    { "BIG Sync established", HCI_CONTROL_LEA_BROADCAST_BIG_SYNC_ESTABLISHED },
    { "BIG Sync terminated", HCI_CONTROL_LEA_BROADCAST_BIG_SYNC_LOST }
};

const char * MainWindow::get_name_for_val(const name_val_t * p_vals, int num_vals, int value, const char * def = (char *)"unknown")
{
    for(int i = 0; i < num_vals; i++, p_vals++){
        if(QString(p_vals->data) == value){
            return p_vals->name;
        }
    }

    return def;
}

void MainWindow::le_audio_handle_broadcast_status_update(uint8_t *p_data)
{
    uint32_t broadcast_status;

    STREAM_TO_UINT8(broadcast_status, p_data);

    switch(broadcast_status)
    {
    case HCI_CONTROL_LEA_BROADCAST_BIG_SYNC_ESTABLISHED:
    {
        //BIG sync established
        ui->btn_le_audio_hs_bcast_SyncToStream->setText(STRING_TERMINATE_STREAM);
    }break;
    case HCI_CONTROL_LEA_BROADCAST_BIG_SYNC_LOST:
    {
        //BIG sync established
        ui->btn_le_audio_hs_bcast_SyncToStream->setText(STRING_SYNC_TO_STREAM);
    }break;
    }

    if((broadcast_status >= HCI_CONTROL_LEA_BROADCAST_PA_SYNC_ESTABLISHED) &&
       (broadcast_status <= HCI_CONTROL_LEA_BROADCAST_BIG_SYNC_LOST))
    {
        Log("status : %s",
            get_name_for_val(broadcast_status_val, sizeof(broadcast_status_val)/sizeof(broadcast_status_val[0]), broadcast_status));
    }
}

void MainWindow::le_audio_reset_ui()
{
    ui->btn_le_audio_connectToPeer->setText(STRING_CONNECT);

    /* headset */
    InitLeAudioHeadsetMediaControl(FALSE);
    InitLeAudioHeadsetVolumeControl(FALSE);
    InitLeAudioHeadsetBroadcastSink(FALSE);

    /* player */
    InitLeAudioPlayerMediaControl(FALSE);
    InitLeAudioPlayerMediaFile(FALSE);
    InitLeAudioPlayerVolumeControl(FALSE);
    InitLeAudioPlayerCallControlSetup(FALSE);
    InitLeAudioPlayerCallControls(FALSE);
    InitLeAudioPlayerBroadcastSource(FALSE);
    InitLeAudioPlayerBroadcastSourceControls(FALSE);

    /* assistant */
    InitLeAudioBroadcastAssistant(FALSE);

    ui->btn_le_audio_startLeAdv->setText(STRING_START_ADVERTISEMENT);


    //reset_controls();
    ui->le_audio_deviceRoleTxt->setText("Device Role");
}
