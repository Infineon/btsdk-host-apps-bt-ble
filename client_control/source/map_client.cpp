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
 * Sample MCU application for MAP client using WICED HCI protocol.
 */

#include "app_include.h"
#include <stdlib.h>

#define MCE_MAX_LIST_COUNT      50

void MainWindow::InitMAPClient()
{
}

void MainWindow::on_btnMceGetServices_clicked()
{
    Log("MAP Client get services");

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
    {
        Log("No device selected");
        return;
    }

    UINT8 bd_addr[6];
    for (int i = 0; i < 6; i++)
        bd_addr[i] = pDev->m_address[5 - i];

    UINT8 buf[10];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_BDA, bd_addr, 6);

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_GET_MAS_INSTANCES, buf, p - buf);
}

void MainWindow::on_btnMceConnect_clicked()
{
    Log("MAP Client connect");

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
    {
        Log("No device selected");
        return;
    }

    if (pDev->m_mce_handle != NULL_HANDLE)
    {
        Log("A MAP session already exists");
        return;
    }

    UINT8 bd_addr[6];
    for (int i = 0; i < 6; i++)
        bd_addr[i] = pDev->m_address[5 - i];

    QVariant qv = ui->cbMceInstanceList->currentData();
    if (!qv.isValid())
    {
        Log("No MAP service selected");
        return;
    }

    UINT16 mas_value = qv.value<UINT16>();
    UINT8 mas_ins_id = (UINT8)(mas_value & 0xFF);

    UINT8 buf[20];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_BDA, bd_addr, 6);
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_MAS_INS_ID, &mas_ins_id, 1);

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_CONNECT, buf, p - buf);

    m_mce_msg_type = (UINT8)(mas_value >> 8);
}

void MainWindow::on_btnMceDisconnect_clicked()
{
    Log("MAP Client disconnect");

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 buf[10];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_DISCONNECT, buf, p - buf);
}

void MainWindow::on_cbMceFolderList_currentIndexChanged(const QString &text)
{
    Log("MAP Client folder list current index changed");

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 buf[20];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

    UINT8 nav_flag;
    if (m_mce_cur_folder == "msg")
        nav_flag = 2; // Down one level
    else
        nav_flag = 3; // Up one level
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_NAV_FLAG, &nav_flag, 1);

    m_mce_set_folder = text;
    QByteArray array = m_mce_set_folder.toLocal8Bit();
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_FOLDER, (UINT8 *)array.data(), m_mce_set_folder.size());

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_SET_FOLDER, buf, p - buf);
}

void MainWindow::on_listMceMessages_currentItemChanged(QListWidgetItem *item, QListWidgetItem *previous)
{
    Log("MAP Client get message");

    if (NULL == item)
        return;

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 buf[40];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

    QByteArray array = item->text().toLocal8Bit();
    char *msg_handle = array.data();
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_MSG_HANDLE, (UINT8 *)msg_handle, strlen(msg_handle));

    m_mce_rcvd_text.clear();
    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_GET_MESSAGE, buf, p - buf);
}

void MainWindow::on_btnMceDelete_clicked()
{
    Log("MAP Client delete");

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    m_mce_delete_item = ui->listMceMessages->currentItem();
    if (NULL == m_mce_delete_item)
        return;

    UINT8 buf[40];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

    QByteArray array = m_mce_delete_item->text().toLocal8Bit();
    char *msg_handle = array.data();
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_MSG_HANDLE, (UINT8 *)msg_handle, strlen(msg_handle));

    UINT8 msg_status_indic = 1;
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_MSG_STATUS_INDIC, &msg_status_indic, 1);

    UINT8 msg_status_value;
    if (ui->btnMceDelete->text() == "Delete")
        msg_status_value = 1;
    else
        msg_status_value = 0;
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_MSG_STATUS_VALUE, &msg_status_value, 1);

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_SET_MESSAGE_STATUS, buf, p - buf);
}

void MainWindow::on_btnMceReply_clicked()
{
    ui->edMceTo->setText(ui->edMceFrom->text());

    if (m_mce_msg_type & 1)     // is Email
    {
        QString subject = ui->edMceFromSubject->text();
        if (subject.indexOf("RE: ", 0, Qt::CaseInsensitive) != 0)
            subject.prepend("RE: ");
        ui->edMceToSubject->setText(subject);
    }

    ui->edtMceCompose->clear();
}

bool MapIsEmail(QString text)
{
    QRegularExpression re("\\w+@\\w+.\\w+");
    QRegularExpressionMatch match = re.match(text);

    if (match.hasMatch())
        return true;

    return false;
}

bool MapIsPhoneNumber(QString text)
{
    QString valids("0123456789()+- ");
    QChar chr;

    for (int i = 0; i < text.size(); i++)
    {
        chr = text.at(i);
        if (valids.indexOf(chr) < 0)
            return false;
    }
    return true;
}

QString MapTypeToString(int type)
{
    QString str;

    if (type & 1)
        str = QString("EMAIL");
    else if (type & 2)
        str = QString("SMS_GSM");
    else if (type & 4)
        str = QString("SMS_CDMA");
    else
        str = QString("MMS");

    return str;
}

#define MAP_SEND_MESSAGE_TEMPLATE   "BEGIN:BMSG\r\nVERSION:1.0\r\nSTATUS:UNREAD\r\nTYPE:%s\r\nFOLDER:telecom/msg/outbox\r\nBEGIN:BENV\r\nBEGIN:VCARD\r\nVERSION:3.0\r\n%s\r\nEND:VCARD\r\nBEGIN:BBODY\r\nCHARSET:UTF-8\r\nLENGTH:%d\r\nBEGIN:MSG\r\n%s\r\nEND:MSG\r\nEND:BBODY\r\nEND:BENV\r\nEND:BMSG\r\n"

void MainWindow::on_btnMceSend_clicked()
{
    Log("MAP Client send");

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    QString recipient = ui->edMceTo->text();
    if (recipient.size() == 0)
    {
        Log("Recipient not specified");
        return;
    }

    QString content = ui->edtMceCompose->toPlainText();
    if (m_mce_msg_type & 1)     // is Email
    {
        QString temp = "Subject: " + ui->edMceToSubject->text() + "\r\n\r\n";
        content.prepend(temp);
        temp = "To: " + recipient + "\r\n";
        content.prepend(temp);

        if (MapIsEmail(recipient))
            recipient.prepend("EMAIL:");
        else
            recipient.prepend("FN:");
    }
    else
    {
        if (MapIsPhoneNumber(recipient))
            recipient.prepend("TEL:");
        else
            recipient.prepend("FN:");
    }

    int size = content.size() + 22;

    m_mce_push_message = QString::asprintf(MAP_SEND_MESSAGE_TEMPLATE, MapTypeToString(m_mce_msg_type).toLocal8Bit().data(), recipient.toLocal8Bit().data(), size, content.toLocal8Bit().data());

    // Change to outbox
    UINT8 buf[20];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

    UINT8 nav_flag = 3; // Up one level
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_NAV_FLAG, &nav_flag, 1);

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_FOLDER, (UINT8 *)"outbox", 6);

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_SET_FOLDER, buf, p - buf);
}

void MainWindow::onHandleWicedEventMAPClient(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (opcode)
    {
    case HCI_CONTROL_MCE_EVENT_MAS_INSTANCES:
        onHandleMceMasInstances(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_CONNECTED:
        onHandleMceConnected(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_DISCONNECTED:
        onHandleMceDisconnected(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_FOLDER_SET:
        onHandleMceFolderSet(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_FOLDER_LIST:
        onHandleMceFolderList(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_MESSAGE_LIST:
        onHandleMceMessageList(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_MESSAGE:
        onHandleMceMessage(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_MESSAGE_PUSHED:
        onHandleMceMessagePushed(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_MESSAGE_STATUS_SET:
        onHandleMceMessageStatusSet(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_NOTIF_REG:
        onHandleMceNotifReg(p_data, len);
        break;

    case HCI_CONTROL_MCE_EVENT_NOTIF:
        onHandleMceNotif(p_data, len);
        break;
    }
}

void MainWindow::onHandleMceMasInstances(unsigned char *p_data, unsigned int len)
{
    UINT8 *p;
    UINT8 status;
    UINT8 num_mas_inst;
    UINT8 mas_ins_id;
    UINT8 supported_type;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        Log("MAP Client got MAS instances");

        p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_NUM_MAS_INST);
        num_mas_inst = p[2];

        // Setup MAS instance list
        UINT8 *p_search = p_data;
        ui->cbMceInstanceList->clear();
        for (int i = 0; i < num_mas_inst; i++)
        {
            char name[100];
            QVariant qv;

            p = MapFindTlv(p_search, len, HCI_CONTROL_MCE_PARAM_NAME);
            memcpy(name, &p[2], p[1]);
            name[p[1]] = 0;
            p = MapFindTlv(p_search, len, HCI_CONTROL_MCE_PARAM_MAS_INS_ID);
            mas_ins_id = p[2];
            p = MapFindTlv(p_search, len, HCI_CONTROL_MCE_PARAM_SUPPORTED_TYPE);
            supported_type = p[2];

            qv.setValue<UINT16>(mas_ins_id + ((UINT16)supported_type << 8));
            ui->cbMceInstanceList->addItem(name, qv);
            p_search = p + 3;
        }
    }
    else
        Log("MAP Client failed to get MAS instance");
}

void MainWindow::MceResizeMessageWindows(bool larger)
{
    int top_change;
    if (larger)
        top_change = -26;
    else
        top_change = 26;

    QRect rect = ui->edtMceMessage->geometry();
    rect.adjust(0, top_change, 0, 0);
    ui->edtMceMessage->setGeometry(rect);

    rect = ui->edtMceCompose->geometry();
    rect.adjust(0, top_change, 0, 0);
    ui->edtMceCompose->setGeometry(rect);
}

void MainWindow::onHandleMceConnected(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        Log("MAP Client connected");
        p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_SESS_HANDLE);
        pDev->m_mce_handle = p[2] + ((UINT16)p[3] << 8);

        // Resize edit window to cover/show subject
        if (m_mce_msg_type & 1)     // is Email
        {
            if (ui->edtMceMessage->height() > 120)
                MceResizeMessageWindows(false);
        }
        else
        {
            if (ui->edtMceMessage->height() < 120)
                MceResizeMessageWindows(true);
        }

        m_mce_notif_registered = FALSE;

        // Set telecom folder
        UINT8 buf[20];

        p = buf;
        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

        UINT8 nav_flag = 2; // Down one level
        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_NAV_FLAG, &nav_flag, 1);

        m_mce_set_folder = "telecom";
        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_FOLDER, (UINT8 *)"telecom", m_mce_set_folder.size());

        SendWicedCommand(HCI_CONTROL_MCE_COMMAND_SET_FOLDER, buf, p - buf);
    }
    else
    {
        Log("MAP Client failed to connect");
        Log("Note for iOS users: Go to the Bluetooth menu, select the 'MAP Client' device and enable 'Show Notifications'. Then try the connection again.");
    }
}

void MainWindow::onHandleMceDisconnected(unsigned char *p_data, unsigned int len)
{
    UNUSED(p_data); // silence warning
    UNUSED(len);

    Log("MAP Client disconnected");

    ui->cbMceFolderList->clear();
    ui->listMceMessages->clear();
    ui->edMceFrom->clear();
    ui->edMceFromSubject->clear();
    ui->edtMceMessage->clear();
    ui->edMceTo->clear();
    ui->edMceToSubject->clear();
    ui->edtMceCompose->clear();

    CBtDevice * pDev = (CBtDevice *)GetSelectedDevice();
    if (pDev)
        pDev->m_mce_handle = NULL_HANDLE;
}

UINT16 MainWindow::MceSendMessageData(UINT16 mce_handle)
{
    UINT8 buf[200];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&mce_handle, 2);

    UINT16 send_len = m_mce_push_message.size();
    QByteArray array = m_mce_push_message.toLocal8Bit();
    if (send_len > 194)
    {
        send_len = 194;
        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_DATA, (UINT8 *)array.data(), send_len);
        m_mce_push_message.remove(0, send_len);
    }
    else
    {
        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_DATA_END, (UINT8 *)array.data(), send_len);
        m_mce_push_message.clear();
    }
    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_PUSH_MESSAGE, buf, p - buf);

    return send_len;
}

void MainWindow::onHandleMceFolderSet(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        Log("MAP Client folder set");

        UINT8 buf[20];

        p = buf;
        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

        m_mce_cur_folder = m_mce_set_folder;
        if (m_mce_cur_folder == "telecom")
        {
            // Set msg folder
            UINT8 nav_flag = 2; // Down one level
            p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_NAV_FLAG, &nav_flag, 1);

            m_mce_set_folder = "msg";
            p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_FOLDER, (UINT8 *)"msg", m_mce_set_folder.size());

            SendWicedCommand(HCI_CONTROL_MCE_COMMAND_SET_FOLDER, buf, p - buf);
        }
        else if (m_mce_cur_folder == "msg")
        {
            // Get folder listing
            m_mce_rcvd_text.clear();
            SendWicedCommand(HCI_CONTROL_MCE_COMMAND_LIST_FOLDERS, buf, p - buf);
        }
        else
        {
            if (m_mce_push_message.size() > 0)
            {
                MceSendMessageData(pDev->m_mce_handle);
            }
            else
            {
                if (m_mce_cur_folder == "deleted")
                    ui->btnMceDelete->setText("Undelete");
                else
                    ui->btnMceDelete->setText("Delete");

                if (m_mce_cur_folder == "sent" || m_mce_cur_folder == "outbox")
                    ui->labelMceFrom->setText("To:");
                else
                    ui->labelMceFrom->setText("From:");

                // Get message listing
                m_mce_rcvd_text.clear();
                MceSendGetMessageListing(pDev->m_mce_handle, 0, MCE_MAX_LIST_COUNT);
            }
        }
    }
    else
        Log("MAP Client failed to set folder");
}

void MainWindow::MceSendGetMessageListing(UINT16 mce_handle, UINT16 start_offset, UINT16 max_count)
{
    UINT8 buf[20];
    UINT8 *p = buf;

    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&mce_handle, 2);
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_LIST_START_OFFSET, (UINT8 *)&start_offset, 2);
    p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_MAX_LIST_COUNT, (UINT8 *)&max_count, 2);

    m_mce_list_offset = start_offset;

    SendWicedCommand(HCI_CONTROL_MCE_COMMAND_LIST_MESSAGES, buf, p - buf);
}

void MainWindow::onHandleMceFolderList(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        bool is_end = false;
        p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA);
        if (!p)
        {
            p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA_END);
            is_end = true;
        }
        if (p)
        {
            char str[256];
            memcpy(str, &p[2], p[1]);
            str[p[1]] = 0;
            m_mce_rcvd_text.append(str);

            if (is_end)
            {
                int start = 0, end;
                bool has_outbox = false;

                Log("MAP Client folder list %d bytes", m_mce_rcvd_text.size());

                ui->cbMceFolderList->clear();
                ui->listMceMessages->clear();
                ui->edMceFrom->clear();
                ui->edMceFromSubject->clear();
                ui->edtMceMessage->clear();
                while ((start = m_mce_rcvd_text.indexOf("<folder name", start)) != -1)
                {
                    start = m_mce_rcvd_text.indexOf("\"", start) + 1;
                    end = m_mce_rcvd_text.indexOf(QString("\""), start);
                    QString folder = m_mce_rcvd_text.mid(start, end - start);
                    ui->cbMceFolderList->addItem(folder);
                    if (folder == "outbox")
                        has_outbox = true;
                }
                m_mce_rcvd_text.clear();
                ui->btnMceReply->setEnabled(has_outbox);
                ui->btnMceSend->setEnabled(has_outbox);
            }
        }
    }
    else
        Log("MAP Client failed to list folder");
}

void MainWindow::onHandleMceMessageList(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        bool is_end = false;
        p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA);
        if (!p)
        {
            p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA_END);
            is_end = true;
        }
        if (p)
        {
            char str[256];
            memcpy(str, &p[2], p[1]);
            str[p[1]] = 0;
            m_mce_rcvd_text.append(str);

            if (is_end)
            {
                int start = 0, end;
                int count = 0;

                Log("MAP Client message list %d bytes", m_mce_rcvd_text.size());

                if (m_mce_list_offset == 0)
                {
                    ui->listMceMessages->clear();
                    ui->edMceFrom->clear();
                    ui->edMceFromSubject->clear();
                    ui->edtMceMessage->clear();
                }
                while ((start = m_mce_rcvd_text.indexOf("<msg handle", start)) != -1)
                {
                    start = m_mce_rcvd_text.indexOf("\"", start) + 1;
                    end = m_mce_rcvd_text.indexOf(QString("\""), start);
                    ui->listMceMessages->addItem(m_mce_rcvd_text.mid(start, end - start));
                    count++;
                }
                m_mce_rcvd_text.clear();

                if (count == MCE_MAX_LIST_COUNT)
                    MceSendGetMessageListing(pDev->m_mce_handle, m_mce_list_offset + MCE_MAX_LIST_COUNT, MCE_MAX_LIST_COUNT);
                else
                {
                    Log("MAP Client %d messages listed", m_mce_list_offset + count);

                    if (!m_mce_notif_registered)
                    {
                        // Enable notification
                        UINT8 buf[20];

                        p = buf;
                        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

                        UINT8 notif_status = 1;
                        p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_NOTIF_STATUS, &notif_status, 1);

                        SendWicedCommand(HCI_CONTROL_MCE_COMMAND_NOTIF_REG, buf, p - buf);
                    }
                }
            }
        }
    }
    else
        Log("MAP Client failed to list message");
}

QString BMsgFindTag(QString input, QString tag)
{
    int start, end;
    QString content;

    tag.prepend('\n');
    if ((start = input.indexOf(tag, 0)) > 0)
    {
        start = input.indexOf(":", start) + 1;
        end = input.indexOf("\r", start);
        content = input.mid(start, end - start);
    }
    return content;
}

QString QuotedPrintableDecode(QString input)
{
    char *str = new char[input.size()];
    QByteArray array = input.toLocal8Bit();
    char *qp = array.data();
    int i = 0, j = 0, value;

    while (i < input.size())
    {
        if (qp[i] == '=')
        {
            sscanf(&qp[i], "=%02x", &value);
            str[j++] = (char)value;
            i += 3;
        }
        else
            str[j++] = qp[i++];
    }
    str[j] = 0;

    QString output(str);
    delete []str;

    return output;
}

QString MapMimeDecode(QString input)
{
    QString output = input;
    int start;
    int from = 0;

    while ((start = output.indexOf("=?", from)) != -1)
    {
        int index1, index2;
        QString charset, encoding, data;

        index1 = start + 2;
        index2 = output.indexOf('?', index1);
        if (index2 == -1)
            break;
        charset = output.mid(index1, index2 - index1);

        index1 = index2 + 1;
        index2 = output.indexOf('?', index1);
        if (index2 == -1)
            break;
        encoding = output.mid(index1, index2 - index1);

        index1 = index2 + 1;
        index2 = output.indexOf("?=", index1);
        if (index2 == -1)
            break;
        data = output.mid(index1, index2 - index1);

        if (encoding == "Q" || encoding == "q")
        {
            QString decoded = QuotedPrintableDecode(data);
            output.remove(start, index2 - start + 2);
            output.insert(start, decoded);
            from = start + decoded.size();
        }
        else
        {
            from = index2 + 2;
        }
    }

    return output;
}

void MainWindow::onHandleMceMessage(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        bool is_end = false;
        p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA);
        if (!p)
        {
            p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA_END);
            is_end = true;
        }
        if (p)
        {
            char str[256];
            memcpy(str, &p[2], p[1]);
            str[p[1]] = 0;
            m_mce_rcvd_text.append(str);

            if (is_end)
            {
                int start, end;
                QString vcard, from;

                Log("MAP Client message %d bytes", m_mce_rcvd_text.size());

                if (m_mce_cur_folder == "sent" || m_mce_cur_folder == "outbox")
                    start = m_mce_rcvd_text.lastIndexOf("\nBEGIN:VCARD");
                else
                    start = m_mce_rcvd_text.indexOf("\nBEGIN:VCARD", 0);
                if (start > 0)
                {
                    start = m_mce_rcvd_text.indexOf("\n", start + 1) + 1;
                    end = m_mce_rcvd_text.indexOf("\nEND:VCARD", start);
                    vcard = m_mce_rcvd_text.mid(start, end - start);
                    if (m_mce_msg_type & 1)     // is Email
                        from = BMsgFindTag(vcard, "EMAIL");
                    else
                        from = BMsgFindTag(vcard, "TEL");
                    if (from.size() == 0)
                        from = BMsgFindTag(vcard, "FN");
                    if (from.size() == 0)
                        from = BMsgFindTag(vcard, "N");
                    ui->edMceFrom->setText(from);
                }

                start = m_mce_rcvd_text.indexOf("\nBEGIN:MSG", 0);
                if (start > 0)
                {
                    start = m_mce_rcvd_text.indexOf("\n", start + 1) + 1;
                    end = m_mce_rcvd_text.indexOf("\nEND:MSG", start);

                    QString msg_body = MapMimeDecode(m_mce_rcvd_text.mid(start, end - start));
                    if (m_mce_msg_type & 1)     // is Email
                    {
                        ui->edMceFromSubject->setText(BMsgFindTag(msg_body, "Subject").remove(0, 1));
                        ui->edtMceMessage->setPlainText(msg_body.remove(0, msg_body.indexOf("\r\n\r\n") + 4));
                    }
                    else
                        ui->edtMceMessage->setPlainText(msg_body);
                }

                m_mce_rcvd_text.clear();
            }
        }
    }
    else
        Log("MAP Client failed to get message");
}

void MainWindow::onHandleMceMessagePushed(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        Log("MAP Client message data pushed");

        // Send more data
        if (m_mce_push_message.size() > 0)
            MceSendMessageData(pDev->m_mce_handle);
        else
        {
            // Change back to original folder
            UINT8 buf[20];
            p = buf;

            p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_SESS_HANDLE, (UINT8 *)&pDev->m_mce_handle, 2);

            UINT8 nav_flag = 3; // Up one level
            p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_NAV_FLAG, &nav_flag, 1);

            QByteArray array = m_mce_cur_folder.toLocal8Bit();
            p += MapAddTlv(p, HCI_CONTROL_MCE_PARAM_FOLDER, (UINT8 *)array.data(), m_mce_cur_folder.size());

            SendWicedCommand(HCI_CONTROL_MCE_COMMAND_SET_FOLDER, buf, p - buf);
        }
    }
    else
        Log("MAP Client failed to push message");
}

void MainWindow::onHandleMceMessageStatusSet(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        Log("MAP Client message status set");

        if (m_mce_delete_item)
        {
            ui->listMceMessages->takeItem(ui->listMceMessages->row(m_mce_delete_item));
            ui->listMceMessages->removeItemWidget(m_mce_delete_item);
        }
    }
    else
        Log("MAP Client failed to set message status");

    m_mce_delete_item = NULL;
}

void MainWindow::onHandleMceNotifReg(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);

    Log("MAP Client notification registration status %d", p[2]);

    if (p[2] == 0)
        m_mce_notif_registered = TRUE;
}

QString MceGetEventAttribute(QString notif, QString attr)
{
    int index1 = 0;
    int index2 = 0;

    index1 = notif.indexOf("<event ");
    if (index1 > 0)
    {
        index1 = notif.indexOf(attr, index1);
        if (index1 > 0)
        {
            index1 = notif.indexOf("\"", index1);
            if (index1 > 0)
                index2 = notif.indexOf("\"", ++index1);
        }
    }

    if (index1 > 0 && index2 > 0 && index2 > index1)
        return notif.mid(index1, index2 - index1);

    return NULL;
}

QString MceNotifToString(QString notif)
{
    QString st;
    QString type = MceGetEventAttribute(notif, "type");
    QString handle = MceGetEventAttribute(notif, "handle");
    QString msg_type = MceGetEventAttribute(notif, "msg_type");

    if (type == "NewMessage")
    {
        st = QString("New ") + msg_type + QString(" message received, handle ") + handle;
    }
    else if (type == "MessageDeleted")
    {
        st = msg_type + QString(" message ") + handle + QString(" has been deleted");
    }
    else if (type == "MessageShift")
    {
        QString folder = MceGetEventAttribute(notif, "folder");
        QString old_folder = MceGetEventAttribute(notif, "old_folder");
        folder.remove(0, folder.lastIndexOf("/"));
        old_folder.remove(0, old_folder.lastIndexOf("/"));
        st = msg_type + QString(" message ") + handle + QString(" moved from ") +
                old_folder + QString(" to ") + folder;
    }
    else if (type == "DeliverySuccess")
    {
        st = msg_type + QString(" message ") + handle + QString(" has been delivered");
    }
    else if (type == "SendingSuccess")
    {
        st = msg_type + QString(" message ") + handle + QString(" has been sent");
    }
    else if (type == "DeliveryFailure")
    {
        st = msg_type + QString(" message ") + handle + QString(" delivery failed");
    }
    else if (type == "SendingFailure")
    {
        st = msg_type + QString(" message ") + handle + QString(" sending failed");
    }
    else if (type == "MemoryFull")
    {
        st = "MAP server memory is full";
    }
    else if (type == "MemoryAvailable")
    {
        st = "MAP server memory is available again";
    }
    else if (type == "ReadStatusChanged")
    {
        st = msg_type + QString(" message ") + handle + QString(" read status changed");
    }
    else
    {
        st = QString("Notification type ") + type + QString(" received");
    }

    return st;
}

void MainWindow::onHandleMceNotif(unsigned char *p_data, unsigned int len)
{
    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    UINT8 *p;
    UINT8 status;

    p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_STATUS);
    status = p[2];
    if (status == 0)
    {
        bool is_end = false;
        p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA);
        if (!p)
        {
            p = MapFindTlv(p_data, len, HCI_CONTROL_MCE_PARAM_DATA_END);
            is_end = true;
        }
        if (p)
        {
            char str[256];
            memcpy(str, &p[2], p[1]);
            str[p[1]] = 0;
            m_mce_rcvd_text.append(str);

            if (is_end)
            {
                Log("MAP notification received: %s", MceNotifToString(m_mce_rcvd_text).toLocal8Bit().data());

                QString type = MceGetEventAttribute(m_mce_rcvd_text, "type");
                if (type == "NewMessage" && ui->cbMceFolderList->currentText() == "inbox")
                {
                    QString handle = MceGetEventAttribute(m_mce_rcvd_text, "handle");
                    ui->listMceMessages->insertItem(0, handle);
                }
                else if (type == "MessageDeleted" || type == "MessageShift")
                {
                    QString handle = MceGetEventAttribute(m_mce_rcvd_text, "handle");
                    QList<QListWidgetItem *> widgetItems = ui->listMceMessages->findItems(handle, 0);
                    if (widgetItems.count() > 0)
                    {
                        ui->listMceMessages->takeItem(ui->listMceMessages->row(widgetItems.first()));
                        ui->listMceMessages->removeItemWidget(widgetItems.first());
                    }
                }

                m_mce_rcvd_text.clear();
            }
        }
    }
    else
        Log("MAP Client notification error");
}

UINT16 MainWindow::MapAddTlv(UINT8 *buf, UINT8 type, UINT8 *value, UINT8 value_len)
{
    UINT8 *p = buf;

    *p++ = type;
    *p++ = value_len;
    memcpy(p, value, value_len);

    return value_len + 2;
}

UINT8 *MainWindow::MapFindTlv(UINT8 *buf, UINT8 data_len, UINT8 type)
{
    UINT8 *p = buf;
    UINT8 len;

    while (data_len > 0)
    {
        if (*p == type)
            return p;

        p++;
        len = *p++;
        p += len;
        data_len -= len + 2;
    }

    return NULL;
}
