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
 * Sample MCU application for implementing BLE mesh application using WICED HCI protocol.
 */

#include "app_include.h"

// Initialize app
void MainWindow::InitMesh()
{

}

// Gemeric ON OFF model, On clicked
void MainWindow::on_btnMeshOn_clicked()
{

}


// Gemeric ON OFF model, off clicked
void MainWindow::on_btnMeshOff_clicked()
{

}


// Gemeric level model, level changed
void MainWindow::on_ctrlMeshSlider_valueChanged(int value)
{

}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventMesh(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_MESH:
        HandleMeshEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for Mesh
void MainWindow::HandleMeshEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char   trace[1024];
    CBtDevice *device;
    BYTE    bda[6];

    UINT16  handle, features;


    switch (opcode)
    {
        case HCI_CONTROL_MESH_EVENT_ONOFF_SET:
        {

        }
        break;
        case HCI_CONTROL_MESH_EVENT_LEVEL_SET:
        {
        }
        break;
    }
}
