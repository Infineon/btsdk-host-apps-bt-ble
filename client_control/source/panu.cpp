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


#include "app_include.h"

extern MainWindow *g_pMainWindow;

extern "C"
{
#include "app_host_panu.h"
}


// Initialize app
void MainWindow::InitPANU()
{
    g_pMainWindow = this;
    ui->btnPANUConnect->setEnabled(true);
    ui->btnPANUDisconnect->setEnabled(false);
}

// Connect to peer with SPP connection
void MainWindow::on_btnPANUConnect_clicked()
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

    if(pDev->m_panu_handle != NULL_HANDLE)
    {
        Log("PANU already connected for selected device");
        return;
    }

    app_host_panu_connect(pDev->m_address);
}

void MainWindow::on_btnPANUDisconnect_clicked()
{
    CBtDevice * pDev = GetConnectedPANUDevice();
    if (pDev == NULL)
        return;

    app_host_panu_disconnect(pDev->m_address);
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventPANU(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    //app_host_panu_event(opcode, p_data, len);
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_PANU:
        HandlePANUEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for SPP
void MainWindow::HandlePANUEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    CBtDevice *device;
    BYTE bda[6];
    uint16_t  handle;

    app_host_panu_event(opcode, p_data, len);
    switch (opcode)
    {
    case HCI_CONTROL_PANU_EVENT_CONNECTED:
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];

        // find device in the list with received address and save the connection handle
        if ((device = FindInList(bda,ui->cbDeviceList)) == NULL)
            device = AddDeviceToList(bda, ui->cbDeviceList, NULL);

        handle = p_data[6] + (p_data[7] << 8);
        device->m_panu_handle = handle;
        device->m_conn_type |= CONNECTION_TYPE_PANU;

        SelectDevice(ui->cbDeviceList, bda);
        Log("PANU connected");
        ui->btnPANUConnect->setEnabled(false);
        ui->btnPANUDisconnect->setEnabled(true);
        break;
    case HCI_CONTROL_PANU_EVENT_SERVICE_NOT_FOUND:
        Log("PANU Service not found");
        break;
    case HCI_CONTROL_PANU_EVENT_CONNECTION_FAILED:
        Log("PANU Connection Failed");
        break;
    case HCI_CONTROL_PANU_EVENT_DISCONNECTED:
        handle = p_data[0] | (p_data[1] << 8);
        Log("PANU disconnected, Handle: 0x%04x", handle);
        CBtDevice * pDev = FindInList(CONNECTION_TYPE_PANU, handle, ui->cbDeviceList);
        if (pDev && (pDev->m_panu_handle == handle))
        {
            pDev->m_panu_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_PANU;
        }
        ui->btnPANUConnect->setEnabled(true);
        ui->btnPANUDisconnect->setEnabled(false);
        break;
    }
}

CBtDevice* MainWindow::GetConnectedPANUDevice()
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_panu_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as PANU");
        return NULL;
    }

    return pDev;
}
