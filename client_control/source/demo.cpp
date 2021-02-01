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
 * Demo applications.
 */

#include "app_include.h"


// Initialize app
void MainWindow::InitDemo()
{

}


// Handle WICED HCI events
void MainWindow::onHandleWicedEventDemo(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_DEMO:
        HandleDemoEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for demo
void MainWindow::HandleDemoEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    BYTE   ssid[30];
    BYTE   password[30];
    memset(ssid, 0, 30);
    memset(password, 0, 30);
    uint8_t sssid_len;
    uint8_t passwd_len;


    switch (opcode)
    {
        case HCI_CONTROL_DEMO_EVENT_SSID_PASSWD:
        {
            /* Buffer format: ssid len, password len, SSID, password */

            if(len < 2)
            {
                Log("HandleDemoEvents, invalid data, packet size %ld", len);
                break;
            }
            sssid_len = p_data[0];
            passwd_len = p_data[1];

            if(len < (sssid_len + passwd_len))
            {
                Log("HandleDemoEvents, invalid data, SSID len %d, passd len %d, total packet size %ld",
                    sssid_len, passwd_len, len);
                break;
            }

            Log("HandleDemoEvents, SSID len %d, passd len %d, total packet size %ld",
                sssid_len, passwd_len, len);

            if((sssid_len > 30) || (passwd_len > 30))
            {
                Log("HandleDemoEvents, invalid data, SSID len %d, passd len %d",
                    sssid_len, passwd_len);
                break;
            }

            memcpy(ssid, &p_data[2], sssid_len);
            memcpy(password, &p_data[sssid_len+2], passwd_len);

            Log("HandleDemoEvents, SSID %s, password %s", ssid, password);

            ui->labelSSID->setText((char *)ssid);
            ui->labelPassword->setText((char *)password);

            ConnectWiFi(ssid, password);

        }
        break;
    }
}

static void addDelay()
{
    QThread::sleep(5); // Sleep for 5 seconds
}

void MainWindow::ConnectWiFi(BYTE *ssid, BYTE *password)
{
    char command[100];

    Log("ConnectWiFi");

    sprintf(command, "wl mpc 0");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl PM 0");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl down");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl wsec 4");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl wpa_auth 0x80");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl sup_wpa 1");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl set_pmk %s", password);
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl up");
    system(command);
    Log(command);

    addDelay();

    sprintf(command, "wl join %s imode bss amode wpa2psk", ssid);
    system(command);
    Log(command);

    addDelay();
}


void MainWindow::on_btnHelpDemo_clicked()
{
    onClear();
    Log("Demo help topic:");
    Log("");
    Log("Apps : ble_wifi_introducer");
    Log("");
    Log("The BLE WiFi Introducer app shows an example interfacing the BLE and WiFi components on a combo chip.");
    Log("It demonstrates GATT database initialization, DCT configuration, processing read/write requests ");
    Log("from a BLE client, and sending data to the client.");
    Log("https://community.cypress.com/community/wiced-wifi/wiced-wifi-forums/blog/2016/07/18/ble-wifi-introducer");

    ScrollToTop();
}
