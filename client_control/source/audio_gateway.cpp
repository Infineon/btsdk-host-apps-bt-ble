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
 * Sample MCU application for Audio Gateway (AG) using WICED HCI protocol.
 */

#include "app_include.h"

extern "C"
{
#include "app_host.h"
}

// Initialize app
void MainWindow::InitAG()
{
    m_audio_connection_active = false;
}

// Connect AG to peer device
void MainWindow::on_btnAGConnect_clicked()
{
    if (m_CommPort == NULL)
        return;

    CBtDevice * pDev =(CBtDevice *)GetSelectedDevice();
    if (NULL == pDev)
        return;

    app_host_ag_connect(pDev->m_address);

}

// Disconnect AG from peer device
void MainWindow::on_btnAGDisconnect_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    app_host_ag_disconnect(pDev->m_address);
}

// Connect or disconnect audio connection with peer device
void MainWindow::on_btnAGAudioConnect_clicked()
{
    CBtDevice * pDev = GetConnectedAGDevice();
    if (pDev == NULL)
        return;

    if (!m_audio_connection_active)
    {
        app_host_ag_audio_open(pDev->m_address);
        return;
    }
    else
    {
        app_host_ag_audio_close(pDev->m_address);
        return;
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventAG(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_AG:
        HandleAgEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for AG
void MainWindow::HandleAgEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE    bda[6];

    UINT16  handle, features;


    app_host_ag_event(opcode, p_data, len);


    switch (opcode)
    {
    // AG connected with peer
    case HCI_CONTROL_AG_EVENT_OPEN:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd HCI_CONTROL_AG_EVENT_OPEN   BDA: %02x:%02x:%02x:%02x:%02x:%02x  Status: %u",
            handle, p_data[2], p_data[3], p_data[4], p_data[5], p_data[6], p_data[7], p_data[8]);
        Log(trace);

        if (p_data[8] == HCI_CONTROL_HF_STATUS_SUCCESS)
        {
            for (int i = 0; i < 6; i++)
                bda[5 - i] = p_data[2 + i];

            // find device in the list with received address and save the connection handle
            if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
                device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

            device->m_ag_handle = handle;
            device->m_conn_type |= CONNECTION_TYPE_AG;

            SelectDevice(ui->cbDeviceList, bda);

        }
    }
        break;

    // AG diconnected from peer
    case HCI_CONTROL_AG_EVENT_CLOSE:
    {
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x12 - HCI_CONTROL_AG_EVENT_CLOSE", handle);
        Log(trace);

        ui->btnAGAudioConnect->setText("Audio Connect");
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_AG, handle, ui->cbDeviceList);
        if (pDev)
        {
            pDev->m_hf_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_AG;
            m_audio_connection_active = false;
            ui->btnAGAudioConnect->setText("Audio Connect");
        }
    }

        break;
    case HCI_CONTROL_AG_EVENT_CONNECTED:
        handle   = p_data[0] | (p_data[1] << 8);
        features = p_data[2] | (p_data[3] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x13 - HCI_CONTROL_AG_EVENT_CONNECTED  Features: 0x%04x", handle, features);
        Log(trace);
        m_audio_connection_active = false;
        ui->btnAGAudioConnect->setText("Audio Connect");
        break;

    // AG audio connected with peer
    case HCI_CONTROL_AG_EVENT_AUDIO_OPEN:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x14 - HCI_CONTROL_AG_EVENT_AUDIO_OPEN", handle);
        Log(trace);
        ui->btnAGAudioConnect->setText("Audio Disconnect");
        m_audio_connection_active = true;
        break;

    // AG audio diconnected from peer
    case HCI_CONTROL_AG_EVENT_AUDIO_CLOSE:
        handle   = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "[Handle: %u] Rcvd Event 0x15 - HCI_CONTROL_AG_EVENT_AUDIO_CLOSE", handle);
        Log(trace);
        ui->btnAGAudioConnect->setText("Audio Connect");
        m_audio_connection_active = false;
        break;
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedAGDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_ag_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as AG");
        return NULL;
    }

    return pDev;
}


void MainWindow::on_btnHelpAG_clicked()
{
    onClear();
    Log("Audio Gateway help topic:");
    Log("Apps : audio_gateway");
    Log("");

    Log("Peer device - headset, speaker, car-kit supporting hands-free profile");
    Log("- Connect");
    Log("  Connect to a peer device supporting the HF profile");
    Log("- Disconnect");
    Log("  Disconnect with a peer device");
    Log("- Audio Connect");
    Log("  Open a SCO audio channel with a peer device");

    ScrollToTop();
}
