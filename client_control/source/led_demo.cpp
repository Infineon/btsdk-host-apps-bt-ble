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
 * Sample MCU application for LED DEMO using WICED HCI protocol.
 */


#include "app_include.h"

// Initialize app
void MainWindow::InitLED_Demo()
{
    m_led_brightness_level = 0;
    ui->horizontalSliderLEDbrightness->setRange(0, 100);
    ui->horizontalSliderLEDbrightness->setSliderPosition(0);
}

// Slider moved for LED brightness setting
void MainWindow::on_horizontalSliderLEDbrightness_sliderMoved(int position)
{
    if (m_led_brightness_level != position)
    {
        BYTE    cmd[10];
        int     commandBytes = 0;
        m_led_brightness_level = position;

        cmd[commandBytes++] = m_led_brightness_level;
        SendWicedCommand(HCI_CONTROL_LED_COMMAND_SET_BRIGHTNESS, cmd, commandBytes);
        Log("LED Brightness Level : %d ", m_led_brightness_level);
        if (m_led_brightness_level == 0)
        {
            ui->pushButtonLEDONOFF->setText("Turn ON");
        }
        else if (m_led_brightness_level == 100)
        {
            ui->pushButtonLEDONOFF->setText("Turn OFF");
        }
    }
}

void MainWindow::on_pushButtonLEDONOFF_clicked()
{
    UINT16 commnd;
    if (m_led_brightness_level == 0)
    {
        m_led_brightness_level = 100;
        commnd = HCI_CONTROL_LED_COMMAND_TURN_ON;
        ui->pushButtonLEDONOFF->setText("Turn OFF");
        ui->horizontalSliderLEDbrightness->setSliderPosition(100);
        Log("Send LED Turn ON command ");
    }
    else
    {
        m_led_brightness_level = 0;
        commnd = HCI_CONTROL_LED_COMMAND_TURN_OFF;
        ui->pushButtonLEDONOFF->setText("TURN ON");
        ui->horizontalSliderLEDbrightness->setSliderPosition(0);
        Log("Send LED Turn OFF command ");
    }
    SendWicedCommand(commnd, NULL, 0);
}
