/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Sample MCU application for testing.
 */
#include "app_include.h"
extern "C"
{
#include "app_host.h"
}

extern MainWindow *g_pMainWindow;

void MainWindow::InitHciLoopbackTest()
{
    g_pMainWindow = this;
    m_hci_test_is_active = false;
}

void MainWindow::StartHciLoopbackTest()
{
    //Log("StartHciLoopbackTest");
    ui->btnTest->setText("Stop");
    m_hci_test_is_active = true;
    HciLoopbackSendCmd(1, NULL, 0);
}

void MainWindow::StopHciLoopbackTest()
{
    //Log("StopHciLoopbackTest");
    ui->btnTest->setText("Start");
    m_hci_test_is_active = false;
    HciLoopbackSendCmd(2, NULL, 0);
}

void MainWindow::ConfigHciLoopbackTest()
{
    unsigned char cfg[4] = { 0, 0, 0, 0 };
    cfg[0] = ui->comboBox_test_packet_size->currentIndex();
    cfg[1] = ui->comboBox_test_packet_pattern->currentIndex();
    cfg[2] = ui->comboBox_test_packets_in_loop->currentIndex();
    cfg[3] = ui->comboBox_test_error_response->currentIndex();
    Log("ConfigHciLoopbackTest payload size %d pattern: %d packets: %d error response: %d", cfg[0], cfg[1], cfg[2], cfg[3]);
    SendWicedCommand(HCI_CONTROL_HCITEST_CONFIGURE, cfg, 4);
}

void MainWindow::on_btnTest_clicked()
{
    //Log("on_btnTest_clicked");
    if(m_hci_test_is_active)
    {
        StopHciLoopbackTest();
    }
    else
    {
        ConfigHciLoopbackTest();
        StartHciLoopbackTest();
    }
}

void MainWindow::HciLoopbackSendCmd(UINT8 cmd, void * p_data, unsigned int len)
{
    UINT8 buf[1030];

    buf[0] = cmd;
    if(p_data && len)
    {
        memcpy(&buf[1], p_data, len);
    }
    SendWicedCommand(HCI_CONTROL_HCITEST_COMMAND, buf, len + 1);
}


void MainWindow::onHandleWicedEventHciLoopback(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if(m_hci_test_is_active)
    {
        if(opcode == HCI_CONTROL_HCITEST_EVENT_PACKET)
        {
            // return packet
            HciLoopbackSendCmd(3, p_data, len);
        }
        else if(opcode != HCI_CONTROL_EVENT_WICED_TRACE)
        {
            Log("onHandleWicedEventHciLoopback opcode %x len %d", opcode, len);
        }
    }
}

