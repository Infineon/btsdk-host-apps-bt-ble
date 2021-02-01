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
 * Sample MCU application for implementing Phonebook Client profile using WICED HCI protocol.
 */

#include "app_include.h"
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define WICED_PBC_STATUS_SUCCESS 0

enum
{
    WICED_BT_PBC_GET_PARAM_STATUS = 0,          /* status only*/
    WICED_BT_PBC_GET_PARAM_LIST,                /* list message */
    WICED_BT_PBC_GET_PARAM_PROGRESS,            /* progress message*/
    WICED_BT_PBC_GET_PARAM_PHONEBOOK,           /* phonebook param*/
    WICED_BT_PBC_GET_PARAM_PHONEBOOK_DATA,           /* phonebook param*/
    WICED_BT_PBC_GET_PARAM_FILE_TRANSFER_STATUS /* file transfer status*/
};

typedef UINT8 hci_control_pbc_get_param_type_t;

// Initialize app
void MainWindow::InitPBC()
{
    m_pbc_connection_active = false;
    m_pbc_file_remove = false;

    connect(this, SIGNAL(ShowPhonebookData()), this, SLOT(on_ShowPhonebookData()));
}

// Connect to peer
void MainWindow::on_btnPBCConnect_clicked()
{
    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    if(pDev->m_pbc_handle != NULL_HANDLE)
    {
        Log("PBC already connected for selected device");
        return;
    }

    BYTE    cmd[60];
    int     commandBytes = 0;

    for (int i = 0; i < 6; i++)
        cmd[commandBytes++] = pDev->m_address[5 - i];


    Log("Sending PBCP Connect Command, BDA: %02x:%02x:%02x:%02x:%02x:%02x",
           cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);

    SendWicedCommand(HCI_CONTROL_PBC_COMMAND_CONNECT, cmd, 6);

}

// Diconnect from peer
void MainWindow::on_btnPBCDisconnect_clicked()
{
    BYTE   cmd[60];
    int    commandBytes = 0;
    CBtDevice * pDev = GetConnectedPBCDevice();
    if (pDev == NULL)
        return;

    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    Log("Sending PBC Disconnect Command, Handle: 0x%04x", nHandle);
    SendWicedCommand(HCI_CONTROL_PBC_COMMAND_DISCONNECT, cmd, commandBytes);

    pDev->m_pbc_handle = NULL_HANDLE;
    pDev->m_conn_type &= ~CONNECTION_TYPE_PBC;
}

// Abort operation
void MainWindow::on_btnPBCAbort_clicked()
{
    BYTE   cmd[60];
    int    commandBytes = 0;
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
        return;

    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    Log("Sending PBC Abort Command, Handle: 0x%04x", nHandle);
    SendWicedCommand(HCI_CONTROL_PBC_COMMAND_ABORT, cmd, commandBytes);
}

// download phonebook
void MainWindow::on_btnPBCPhonebook_clicked()
{
    BYTE    cmd[60];
    int     commandBytes = 0;
    CBtDevice * pDev = GetConnectedPBCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    if (m_pbc_connection_active)
    {
        m_pbc_file_remove = true;
        SendWicedCommand(HCI_CONTROL_PBC_COMMAND_GET_PHONEBOOK, cmd, commandBytes);
        Log("Sending PBAP Get Phonebook Command, Handle: 0x%04x", nHandle);
    }
}

// download call history
void MainWindow::on_btnPBCCallHistory_clicked()
{
    BYTE    cmd[60];
    int     commandBytes = 0;
    CBtDevice * pDev = GetConnectedPBCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    if (m_pbc_connection_active)
    {
        m_pbc_file_remove = true;
        SendWicedCommand(HCI_CONTROL_PBC_COMMAND_GET_CALL_HISTORY, cmd, commandBytes);
        Log("Sending PBAP Client Get Call History Command, Handle: 0x%04x", nHandle);
    }
}


// Download incoming calls
void MainWindow::on_btnPBCICCall_clicked()
{
    BYTE    cmd[60];
    int     commandBytes = 0;
    CBtDevice * pDev = GetConnectedPBCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    if (m_pbc_connection_active)
    {
        m_pbc_file_remove = true;
        SendWicedCommand(HCI_CONTROL_PBC_COMMAND_GET_INCOMMING_CALLS, cmd, commandBytes);
        Log("Sending PBAP Client Get Incoming Calls Command, Handle: 0x%04x", nHandle);
    }
}

// download outgoing calls
void MainWindow::on_btnPBCOCCalls_clicked()
{
    BYTE    cmd[60];
    int     commandBytes = 0;
    CBtDevice * pDev = GetConnectedPBCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    if (m_pbc_connection_active)
    {
        m_pbc_file_remove = true;
        SendWicedCommand(HCI_CONTROL_PBC_COMMAND_GET_OUTGOING_CALLS, cmd, commandBytes);
        Log("Sending PBAP Client Get Outgoing Calls Command, Handle: 0x%04x", nHandle);
    }
}

// download missed calls
void MainWindow::on_btnPBCMissedCalls_clicked()
{
    BYTE    cmd[60];
    int     commandBytes = 0;
    CBtDevice * pDev = GetConnectedPBCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_pbc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    if (m_pbc_connection_active)
    {
        m_pbc_file_remove = true;
        SendWicedCommand(HCI_CONTROL_PBC_COMMAND_GET_MISSED_CALLS, cmd, commandBytes);
        Log("Sending PBAP Client Get Missed Calls Command, Handle: 0x%04x", nHandle);
    }
}

// Save pbap data to file
void MainWindow::wiced_bt_pbc_pb_save(UINT8* p_buffer, int len)
{
    int dummy = 0;
    int fd = -1;
    static char path[260];
    char   trace[1024];

    sprintf(trace, "wiced_bt_pbc_pb_save response received...len = %d ", len);
    Log(trace);

    if (p_buffer == NULL)
    {
        return;
    }

    /* Check msg len */
    if(!(len))
    {
        sprintf(trace, "wiced_bt_pbc_pb_save response received. DATA Len = 0");
        Log(trace);
        return;
    }

    /* Write received buffer to file */
    memset(path, 0, sizeof(path));
    QString strFilePath = QDir::tempPath() + "/pb_data.vcf";
    strncpy(path, strFilePath.toStdString().c_str(), 260-1);

    sprintf(trace, "wiced_bt_pbc_pb_save file path = %s", path);
    Log(trace);

    /* Delete temporary file */
    if(m_pbc_file_remove)
    {
        unlink(path);
       m_pbc_file_remove = false;
    }

    /* Save incoming data in temporary file */
    fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0666);
    sprintf(trace, "wiced_bt_pbc_pb_save fd = %d", fd);
    Log(trace);

    if(fd != -1)
    {
        dummy = write(fd, p_buffer, len);
        sprintf(trace, "wiced_bt_pbc_pb_save written = %d err = %d", dummy, errno);
        Log(trace);

        UNUSED(dummy);
        ::close(fd);
    }
}

char* MainWindow::GetCurrentWorkingDirectory()
{
    static char buf[260];
    memset(buf, 0, sizeof(buf));
    char* pStr = getcwd(buf, 260);
    UNUSED(pStr);
    return buf;
}

/*
Read contents from the downloaded phonebook object and display it in the UI.
This function is triggered in response to the signal.
*/

void MainWindow::on_ShowPhonebookData()
{
    QString str = QDir::tempPath();
    str += "/pb_data.vcf";

    QFile file(str);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    ui->edtPbcResult->clear();

    // Show RAW data
    QTextStream in(&file);
    while(!in.atEnd()){
        QString line = in.readLine();
        ui->edtPbcResult->appendPlainText(line);
        ui->edtPbcResult->show();
    }
}

// Process PBAP events received
void MainWindow::wiced_bt_pbc_process_pb_event(LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    UINT8  type = 0;
    UINT8  status = 0;


    sprintf(trace, "Rcvd wiced_bt_pbc_process_pb_event data_len = %d ",  (int)len);
    Log(trace);

    type = p_data[0] ;
    len--;
    status = p_data[1] ;
    len--;

    switch(type)
    {
        case WICED_BT_PBC_GET_PARAM_PHONEBOOK_DATA:

            len = len - 2;

            sprintf(trace, "Rcvd Event Type: %d - Status: %d", type, status);
            Log(trace);

            wiced_bt_pbc_pb_save(&p_data[4], len);

        break;

        case WICED_BT_PBC_GET_PARAM_FILE_TRANSFER_STATUS:

            Log("Phonebook Transfer Status : %d", status);
            if(status == WICED_PBC_STATUS_SUCCESS)
                emit ShowPhonebookData();
        break;

        default:
            break;
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventPBC(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_PBC:
        HandlePBCEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for PBAP client
void MainWindow::HandlePBCEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE    bda[6];

    UINT16  handle = 1;
    UINT8 status = 0;

//    Log("MainWindow::HandlePBCEvents(opcode = %d, len = %d,)", opcode, len);

    switch (opcode)
    {
        case HCI_CONTROL_PBC_EVENT_CONNECTED:
        {
            status = p_data[0];
            sprintf(trace, "Rcvd HCI_CONTROL_PBC_EVENT_CONNECTED   BDA: %02x:%02x:%02x:%02x:%02x:%02x  Status: %d",
                p_data[1], p_data[2], p_data[3], p_data[4], p_data[5], p_data[6], status);
            Log(trace);

            if (status == WICED_PBC_STATUS_SUCCESS)
            {
                for (int i = 0; i < 6; i++)
                    bda[i] = p_data[1 + i];

                // find device in the list with received address and save the connection handle
                if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
                    device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

                device->m_pbc_handle = handle;
                device->m_conn_type |= CONNECTION_TYPE_PBC;

                SelectDevice(ui->cbDeviceList, bda);

                m_pbc_connection_active = TRUE;
            }
        }
        break;

        case HCI_CONTROL_PBC_EVENT_DISCONNECTED:
        {
            sprintf(trace, "Rcvd Event - HCI_CONTROL_PBC_EVENT_DISCONNECTED");
            Log(trace);
            m_pbc_connection_active = false;

            CBtDevice * pDev = GetSelectedDevice();
            if (pDev && (pDev->m_pbc_handle == handle))
            {
                pDev->m_pbc_handle = NULL_HANDLE;
                pDev->m_conn_type &= ~CONNECTION_TYPE_PBC;
            }
        }
        break;

        case HCI_CONTROL_PBC_EVENT_ABORTED:
            sprintf(trace, "Rcvd Event - HCI_CONTROL_PBC_EVENT_ABORTED");
            Log(trace);
            m_pbc_connection_active = FALSE;
        break;

        case HCI_CONTROL_PBC_EVENT_PHONEBOOK:
        case HCI_CONTROL_PBC_EVENT_CALL_HISTORY:
        case HCI_CONTROL_PBC_EVENT_INCOMMING_CALLS:
        case HCI_CONTROL_PBC_EVENT_OUTGOING_CALLS:
        case HCI_CONTROL_PBC_EVENT_MISSED_CALLS:
        {
            wiced_bt_pbc_process_pb_event(p_data, len);
        }
        break;
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedPBCDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_pbc_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as PBC");
        return NULL;
    }

    return pDev;
}


void MainWindow::on_btnHelpPBC_clicked()
{
    onClear();
    Log("Phonebook Client help topic:");
    Log("");
    Log("Apps : hci_pbap_client");
    Log("Peer device - phone supporting PBAP server (any recent iPhone or Android phone)");
    Log("");
    Log("- From the phone, initiate device discovery and find the pbap_client application");
    Log("  and perform pairing.");
    Log("- From the Client Control UI, select the paired phone and click Connect");
    Log("- Use the UI to download phone book contacts, and incoming, outgoing or missed");
    Log("  calls.");
    Log("Note: only the first 100 records will be displayed. The application can be");
    Log("updated to get more records if desired.");
    Log("");
    Log("Note for iPhone users - upon pairing, iPhone will show error in esablishing connection.");
    Log("This can be ignored. After establishing connection from Client Control UI, go to Settings->Blueooth");
    Log("UI on iPhone, select the paired device (pbap client) and enable UI to 'Sync Contacts' before");
    Log("attempting to use the Client Control buttons for accessing contacts or calls.");

    ScrollToTop();

}
