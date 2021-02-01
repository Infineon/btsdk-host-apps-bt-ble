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
 * Sample MCU application for A2DP Audio Sink using WICED HCI protocol.
 */


#include "app_include.h"


// Initialize app
void MainWindow::InitAudioSnk()
{
    ui->tabAVSink->setEnabled(TRUE);
    ui->btnAVSinkConnect->setEnabled(TRUE);
    ui->btnAvSinkDisconnect->setEnabled(FALSE);
    ui->btnAvSinkStart->setEnabled(FALSE);
    ui->btnAvSinkSuspend->setEnabled(FALSE);
}

// Connect to peer device
void MainWindow::on_btnAVSinkConnect_clicked()
{
    uint8_t cmd[6];
    uint8_t *p = cmd;
    CBtDevice * pDev;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    // connect audio snk
    pDev =(CBtDevice *)GetSelectedDevice();

    if (pDev == NULL)
    {
        Log("No device selected");
        return;
    }

    if(pDev->m_avk_handle != NULL_HANDLE)
    {
        Log("AUDIO Sink already connected for selected device Handle: 0x%04x", pDev->m_avk_handle);
        return;
    }

    BDADDR_TO_STREAM(p, pDev->m_address);

    SendWicedCommand(HCI_CONTROL_AUDIO_SINK_COMMAND_CONNECT, cmd, p - cmd);

}

void MainWindow::on_btnAvSinkStart_clicked()
{
    uint8_t     cmd[2];
    uint8_t     *p = cmd;
    CBtDevice   *pDev;
    uint16_t    handle;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    pDev =(CBtDevice *)GetConnectedAudioSnkDevice();
    if (NULL == pDev)
        return;

    handle = pDev->m_avk_handle;

    UINT16_TO_STREAM(p, handle);

    Log("Sending AUDIO Sink Start Command, Handle: 0x%04x", handle);
    SendWicedCommand(HCI_CONTROL_AUDIO_SINK_COMMAND_START, cmd, p - cmd);
}

void MainWindow::on_btnAvSinkSuspend_clicked()
{
    uint8_t     cmd[2];
    uint8_t     *p = cmd;
    CBtDevice   *pDev;
    uint16_t    handle;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    pDev =(CBtDevice *)GetConnectedAudioSnkDevice();
    if (NULL == pDev)
        return;

    handle = pDev->m_avk_handle;

    UINT16_TO_STREAM(p, handle);

    Log("Sending AUDIO Sink Suspend Command, Handle: 0x%04x", handle);
    SendWicedCommand(HCI_CONTROL_AUDIO_SINK_COMMAND_STOP, cmd, p - cmd);
}

// Disconnect from peer device
void MainWindow::on_btnAvSinkDisconnect_clicked()
{
    uint8_t     cmd[2];
    uint8_t     *p = cmd;
    CBtDevice   *pDev;
    uint16_t    handle;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    pDev =(CBtDevice *)GetConnectedAudioSnkDevice();
    if (NULL == pDev)
        return;

    handle = pDev->m_avk_handle;

    UINT16_TO_STREAM(p, handle);

    Log("Sending AUDIO Sink Disconnect Command, Handle: 0x%04x", handle);
    SendWicedCommand(HCI_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT, cmd, p - cmd);
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventAudioSnk(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {

    case HCI_CONTROL_GROUP_AUDIO_SINK:
        HandleA2DPEventsAudioSnk(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for AV sink
void MainWindow::HandleA2DPEventsAudioSnk(DWORD opcode, BYTE *p_data, DWORD len)
{
    uint8_t     bda[6];
    CBtDevice   *device;
    uint16_t    handle;
    uint8_t     *p = p_data;
    CBtDevice   *pDev;
    uint8_t     cmd_status;
    uint8_t     label;
    uint8_t     cmd[4];

    switch (opcode)
    {
    case HCI_CONTROL_AUDIO_SINK_EVENT_COMMAND_STATUS:
        STREAM_TO_UINT8(cmd_status, p);
        Log("AUDIO Sink event command status:%d", cmd_status);
        break;

    // connected with peer
    case HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTED:

        ui->btnAVSinkConnect->setEnabled(FALSE);
        ui->btnAvSinkDisconnect->setEnabled(TRUE);
        ui->btnAvSinkStart->setEnabled(TRUE);

        STREAM_TO_BDADDR(bda, p);
        STREAM_TO_UINT16(handle, p);

        // find device in the list with received address and save the connection handle
        if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

        device->m_avk_handle = handle;
        device->m_conn_type |= CONNECTION_TYPE_AVK;

        SelectDevice(ui->cbDeviceList, bda);

        Log("AUDIO Sink Connected, Handle: 0x%04x", handle);
        break;

    // disconnected from peer
    case HCI_CONTROL_AUDIO_SINK_EVENT_DISCONNECTED:

        ui->btnAVSinkConnect->setEnabled(TRUE);
        ui->btnAvSinkDisconnect->setEnabled(FALSE);
        ui->btnAvSinkStart->setEnabled(FALSE);
        ui->btnAvSinkSuspend->setEnabled(FALSE);

        STREAM_TO_UINT16(handle, p);
        pDev = FindInList(CONNECTION_TYPE_AVK, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_avk_handle == handle))
        {
            pDev->m_avk_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_AVK;
        }
        Log("AUDIO Sink disconnected, Handle: 0x%04x", handle);
        break;

    case HCI_CONTROL_AUDIO_SINK_EVENT_STARTED:
        ui->btnAvSinkStart->setEnabled(FALSE);
        ui->btnAvSinkSuspend->setEnabled(TRUE);
        Log("AUDIO Sink started");
        break;

    case HCI_CONTROL_AUDIO_SINK_EVENT_STOPPED:
        ui->btnAvSinkStart->setEnabled(TRUE);
        ui->btnAvSinkSuspend->setEnabled(FALSE);
        Log("AUDIO Sink stopped");
        break;

    case HCI_CONTROL_AUDIO_SINK_EVENT_AUDIO_DATA:
        Log("AUDIO Sink got data");
        break;

    case HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTION_FAILED:
        Log("AUDIO Sink event connection attempt failed (0x%lX)", opcode);
        break;

    case HCI_CONTROL_AUDIO_SINK_EVENT_CODEC_CONFIGURED:
        DumpData((char *)("AUDIO Sink Codec Configured"), p, len, 1);
        break;

    case HCI_CONTROL_AUDIO_SINK_EVENT_START_IND:
        ui->btnAvSinkStart->setEnabled(FALSE);
        ui->btnAvSinkSuspend->setEnabled(TRUE);

        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT8(label, p);
        Log("AUDIO Sink Start Indication request event handle (0x%04x) label %d", handle,label);

        // Send response
        p = &cmd[0];
        UINT16_TO_STREAM(p, handle);
        UINT8_TO_STREAM(p, label);
        UINT8_TO_STREAM(p, 0); // A2D_SUCCESS
        SendWicedCommand(HCI_CONTROL_AUDIO_SINK_COMMAND_START_RSP, cmd, 4);
        break;

    default:
        /* Unhandled */
        Log("Rcvd unknown AUDIO Sink event:0x%lX", opcode);
        break;
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedAudioSnkDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_avk_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as AV SNK");
        return NULL;
    }

    return pDev;
}


void MainWindow::on_btnHelpAVK_clicked()
{
    onClear();
    Log("Not supported");
}
