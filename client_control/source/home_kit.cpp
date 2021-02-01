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
 * Sample MCU application for implementing Apple HomeKit protocol using WICED HCI protocol.
 */

#include "app_include.h"
#include <QTimer>

const char* door_state[] = {
    "Open",
    "Closed",
    "Opening",
    "Closing",
    "Stopped"
};

const char* lock_state[] = {
    "Unsecured",
    "Secured",
    "Jammed",
    "Unknown"
};

const char* lock_target_state[] = {
    "Unsecured",
    "Secured"
};

unsigned char software_token[1024];
char software_token_encode[1400];
unsigned char *p_token_buf;
int software_token_size;

#define HAP_CHARACTERISTIC_FORMAT_BOOL                      0x01
#define HAP_CHARACTERISTIC_FORMAT_UINT8                     0x04
#define HAP_CHARACTERISTIC_FORMAT_UINT16                    0x06
#define HAP_CHARACTERISTIC_FORMAT_UINT32                    0x08
#define HAP_CHARACTERISTIC_FORMAT_UINT64                    0x0a
#define HAP_CHARACTERISTIC_FORMAT_INT32                     0x10
#define HAP_CHARACTERISTIC_FORMAT_FLOAT                     0x14
#define HAP_CHARACTERISTIC_FORMAT_STRING                    0x19
#define HAP_CHARACTERISTIC_FORMAT_TLV8                      0x1b

const char* TypeString(BYTE type)
{
    QString str;
    switch (type)
    {
    case HAP_CHARACTERISTIC_FORMAT_BOOL:
        return "boolean";
    case HAP_CHARACTERISTIC_FORMAT_UINT8:
        return "uint8";
    case HAP_CHARACTERISTIC_FORMAT_UINT16:
        return "uint16";
    case HAP_CHARACTERISTIC_FORMAT_UINT32:
        return "uint32";
    case HAP_CHARACTERISTIC_FORMAT_UINT64:
        return "uint64";
    case HAP_CHARACTERISTIC_FORMAT_INT32:
        return "int32";
    case HAP_CHARACTERISTIC_FORMAT_FLOAT:
        return "float";
    case HAP_CHARACTERISTIC_FORMAT_STRING:
        return "string";
    case HAP_CHARACTERISTIC_FORMAT_TLV8:
        return "tlv8";
    default:
        return "unknown";
    }
}

// Initialize app
void MainWindow::InitHK()
{
    m_bLightOn = false;
    m_nLightBrightness = 0;
    m_nDoorState = 1;
    m_nLockState = 1;
    m_nLockTargetState = 1;
    m_nIdentifyTimerCounter = 0;

    SetLightOnOff(m_bLightOn);

    char strBrightness[20];
    sprintf(strBrightness, "%d", m_nLightBrightness);

    ui->lineEditHKBrightness->setText(strBrightness);

    for (int i = 0; i < 5; i++)
    {
        ui->cbDoorState->addItem(door_state[i]);
    }
    ui->cbDoorState->setCurrentIndex(m_nDoorState);

    for (int i = 0; i < 4; i++)
    {
        ui->cbLockState->addItem(lock_state[i]);
    }
    ui->cbLockState->setCurrentIndex(m_nLockState);

    for (int i = 0; i < 2; i++)
    {
        ui->cbLockTargetState->addItem(lock_target_state[i]);
    }
    ui->cbLockTargetState->setCurrentIndex(m_nLockTargetState);

    p_timer = new QTimer(this);

    connect(p_timer, SIGNAL(timeout()), this, SLOT(on_timer()));
}

void MainWindow::StartHK()
{
    on_btnHKList_clicked();
}

// Send read or write WICED HCI command
void MainWindow::SendHciCommand(UINT16 command, USHORT handle, LPBYTE p, DWORD dwLen)
{
    if (m_CommPort == NULL)
        return;

    BYTE buffer[32];
    char trace[1024];

    buffer[0] = handle & 0xff;
    buffer[1] = (handle >> 8) & 0xff;
    if (p)
        memcpy(&buffer[2], p, dwLen);

    SendWicedCommand(command, buffer, dwLen + 2);

    switch (command)
    {
    case HCI_CONTROL_HK_COMMAND_READ:
        sprintf(trace, "Read characteristic [%02x]", handle);
        Log(trace);
        break;
    case HCI_CONTROL_HK_COMMAND_WRITE:
        sprintf(trace, "Write characteristic [%02x] : ", handle);
        if(p)
        {
            for (int i = 0; i < (int)dwLen; i++)
                sprintf(&trace[strlen(trace)], "%02x", p[i]);
        }
        Log(trace);
        break;
    }
}

// read value
void MainWindow::on_btnHKRead_clicked()
{
    QString str = ui->lineEditHKHandle->text();
    USHORT handle = GetHandle(str);

    SendHciCommand(HCI_CONTROL_HK_COMMAND_READ, handle, NULL, 0);
}

// write value
void MainWindow::on_btnHKWrite_clicked()
{
    USHORT handle;
    BYTE  buffer[32];
    DWORD num_bytes;

    QString strHandle = ui->lineEditHKHandle->text();
    handle = GetHandle(strHandle);

    QString strHex = ui->lineEditHKHexVal->text();
    num_bytes = GetHexValue(buffer, sizeof(buffer), strHex);

    SendHciCommand(HCI_CONTROL_HK_COMMAND_WRITE, handle, buffer, num_bytes);
}

// List characteristics
void MainWindow::on_btnHKList_clicked()
{
    m_hIdentify = 0;
    m_hLightOn = 0;
    m_hLightBrightness = 0;
    m_hDoorState = 0;
    m_hLockState = 0;
    m_hLockTargetState = 0;

    SendWicedCommand(HCI_CONTROL_HK_COMMAND_LIST, NULL, 0);
    Log("List characteristics");
}

// Bulb on/off
void MainWindow::on_btnHKSwitch_clicked()
{
    m_bLightOn = !m_bLightOn;
    SetLightOnOff(m_bLightOn);
    SendHciCommand(HCI_CONTROL_HK_COMMAND_WRITE, m_hLightOn, (LPBYTE)&m_bLightOn, 1);
}

void MainWindow::ShowMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Please enter a value between 0 and 100");
    msgBox.exec();
}

// Set brightness
void MainWindow::on_btnHKSet_clicked()
{
    QString str = ui->lineEditHKBrightness->text();
    if(str.length() == 0 || str.length() > 3)
    {
        ShowMessage();
        return;
    }

    for(int i = 0; i < str.length(); i++)
    {
        QChar c = str.at(i);
        if(!c.isDigit())
        {
            ShowMessage();
            return;
        }
    }

    uint brightness = ui->lineEditHKBrightness->text().toUInt();

    if (brightness > 100)
    {
        ShowMessage();
        return;
    }

    m_nLightBrightness = brightness;
    SendHciCommand(HCI_CONTROL_HK_COMMAND_WRITE, m_hLightBrightness,(LPBYTE)&m_nLightBrightness, 4);

}

// Door state set
void MainWindow::on_cbDoorState_currentIndexChanged(int index)
{
    if (m_CommPort == NULL)
        return;

    m_nDoorState = index;
    SendHciCommand(HCI_CONTROL_HK_COMMAND_WRITE, m_hDoorState, (LPBYTE)&m_nDoorState, 1);

}

// Lock state set
void MainWindow::on_cbLockState_currentIndexChanged(int index)
{
    m_nLockState = index;
    SendHciCommand(HCI_CONTROL_HK_COMMAND_WRITE, m_hLockState, (LPBYTE)&m_nLockState, 1);
}

void MainWindow::on_cbLockTargetState_currentIndexChanged(int index)
{
    m_nLockTargetState = index;
    SendHciCommand(HCI_CONTROL_HK_COMMAND_WRITE, m_hLockTargetState, (LPBYTE)&m_nLockTargetState, 1);
}


// Device factory reset
void MainWindow::on_btnHKFactoryReset_clicked()
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("Warning");
    msgBox.setText("Are you sure you want to factory reset?");
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    if(msgBox.exec() == QMessageBox::Yes)
    {
        SendHciCommand(HCI_CONTROL_HK_COMMAND_FACTORY_RESET, 0, NULL, 0);
    }
}

// Get software authentication token from device
void MainWindow::on_btnHKGetToken_clicked()
{
    SendHciCommand(HCI_CONTROL_HK_COMMAND_GET_TOKEN, 0, NULL, 0);
}

// event to set on off builb state
void MainWindow::SetLightOnOff(BOOL on)
{

    if (on)
        ui->labelHKLight->setText("ON");
    else
        ui->labelHKLight->setText("OFF");
}

// Update UI on event
void MainWindow::UpdateUI(USHORT handle, LPBYTE p, DWORD dwLen)
{
    if(dwLen < 1)
    {
        Log("UpdateUI bad length: %ld", dwLen);
        return;
    }

    char str[10];
    if (handle == m_hLightOn)
    {
        m_bLightOn = p[0];
        Log("UpdateUI Lightbulb On : %d", m_bLightOn);
        SetLightOnOff(m_bLightOn);
    }
    else if (handle == m_hLightBrightness)
    {
        m_nLightBrightness = p[0];
        sprintf(str, "%d", m_nLightBrightness);
        Log("UpdateUI Lightbulb Brightness : %d", m_nLightBrightness);
        ui->lineEditHKBrightness->setText(str);
    }
    else if (handle == m_hLockState)
    {
        m_nLockState = p[0];
        Log("UpdateUI Lock State : %d", m_nLockState);
        ui->cbLockState->setCurrentIndex(m_nLockState);
    }
    else if (handle == m_hLockTargetState)
    {
        m_nLockTargetState = p[0];
        Log("UpdateUI Lock Target State : %d", m_nLockTargetState);
        ui->cbLockTargetState->setCurrentIndex(m_nLockTargetState);
    }
}

void MainWindow::on_timer()
{
    if (m_nIdentifyTimerCounter > 0)
    {
        if (--m_nIdentifyTimerCounter > 0)
        {
            m_bLightOn = !m_bLightOn;
            SetLightOnOff(m_bLightOn);
        }
        else
        {
            p_timer->stop();
        }
    }
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventHK(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
        {
            case HCI_CONTROL_GROUP_HK:
            HandleHkEvent(opcode, p_data, len);
            break;
        }
}

int base64encode(const void* data_buf, size_t dataLength, char* result, size_t resultSize)
{
    const char base64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const uint8_t *data = (const uint8_t *)data_buf;
    size_t resultIndex = 0;
    size_t x;
    uint32_t n = 0;
    int padCount = dataLength % 3;
    uint8_t n0, n1, n2, n3;

    /* increment over the length of the string, three characters at a time */
    for (x = 0; x < dataLength; x += 3)
    {
        /* these three 8-bit (ASCII) characters become one 24-bit number */
        n = ((uint32_t)data[x]) << 16; //parenthesis needed, compiler depending on flags can do the shifting before conversion to uint32_t, resulting to 0

        if ((x + 1) < dataLength)
            n += ((uint32_t)data[x + 1]) << 8;//parenthesis needed, compiler depending on flags can do the shifting before conversion to uint32_t, resulting to 0

        if ((x + 2) < dataLength)
            n += data[x + 2];

        /* this 24-bit number gets separated into four 6-bit numbers */
        n0 = (uint8_t)(n >> 18) & 63;
        n1 = (uint8_t)(n >> 12) & 63;
        n2 = (uint8_t)(n >> 6) & 63;
        n3 = (uint8_t)n & 63;

        /*
        * if we have one byte available, then its encoding is spread
        * out over two characters
        */
        if (resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
        result[resultIndex++] = base64chars[n0];
        if (resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
        result[resultIndex++] = base64chars[n1];

        /*
        * if we have only two bytes available, then their encoding is
        * spread out over three chars
        */
        if ((x + 1) < dataLength)
        {
            if (resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
            result[resultIndex++] = base64chars[n2];
        }

        /*
        * if we have all three bytes available, then their encoding is spread
        * out over four characters
        */
        if ((x + 2) < dataLength)
        {
            if (resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
            result[resultIndex++] = base64chars[n3];
        }
    }

    /*
    * create and add padding that is required if we did not have a multiple of 3
    * number of characters available
    */
    if (padCount > 0)
    {
        for (; padCount < 3; padCount++)
        {
            if (resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
            result[resultIndex++] = '=';
        }
    }
    if (resultIndex >= resultSize) return 1;   /* indicate failure: buffer too small */
    result[resultIndex] = 0;
    return 0;   /* indicate success */
}

// Handle WICED HCI events for Homekit
void MainWindow::HandleHkEvent(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char trace[1024];
    uint handle;
    char strhandle[20];
    FILE *fp = NULL;

    Log("CLightbulbControlDlg::HandleHkEvent  Rcvd Op Code: 0x%04lx, len: %ld", opcode, len);

    switch (opcode)
    {
    case HCI_CONTROL_HK_EVENT_READ_RESPONSE:
        sprintf(trace, "Read Response : ");
        for (int i = 0; i < (int)len - 2; i++)
            sprintf(&trace[strlen(trace)], "%02x", p_data[2 + i]);
        ui->lineEditHKHexVal->setText(&trace[strlen("Read Response : ")]);
        Log(trace);
        break;
    case HCI_CONTROL_HK_EVENT_UPDATE:
        handle = p_data[0] + (p_data[1] << 8);
        sprintf(strhandle, "%04x", handle);
        ui->lineEditHKHandle->setText(strhandle);

        if (handle == m_hIdentify && p_data[2])
        {
            Log("Received Identify");
            m_nIdentifyTimerCounter = 5;

            p_timer->start(1000); // start a 1 sec timer to flicker the light
            break;
        }
        else if (handle == m_hLockTargetState)
        {
            sprintf(trace, "Characteristic update [%s] : %02x", strhandle, p_data[2]);
            ui->lineEditHKHexVal->setText(&trace[strlen("Characteristic update [0000] : ")]);
            Log(trace);
            if (len > 3)
            {
                Log("Additional authorization data : ");
                for (int i = 0; i < (int)len - 3; i++)
                    sprintf(&trace[strlen(trace)], "%02x", p_data[3 + i]);
               Log(trace);
            }
        }
        else
        {
            sprintf(trace, "Characteristic update [%s] : ", strhandle);
            for (int i = 0; i < (int)len - 2; i++)
                sprintf(&trace[strlen(trace)],"%02x", p_data[2 + i]);
            ui->lineEditHKHexVal->setText(&trace[strlen("Characteristic update [0000] : ")]);
            Log(trace);
        }

        UpdateUI(handle, &p_data[2], len - 2);
        break;
    case HCI_CONTROL_HK_EVENT_LIST_ITEM:
        handle = p_data[0] + (p_data[1] << 8);
        if (strcmp((char *)&p_data[3], "Identify") == 0)
            m_hIdentify = handle;
        else if (strcmp((char *)&p_data[3], "Lightbulb On") == 0)
            m_hLightOn = handle;
        else if (strcmp((char *)&p_data[3], "Lightbulb Brightness") == 0)
            m_hLightBrightness = handle;
        else if (strcmp((char *)&p_data[3], "Current Door State") == 0)
            m_hDoorState = handle;
        else if (strcmp((char *)&p_data[3], "Lock Current State") == 0)
            m_hLockState = handle;
        else if (strcmp((char *)&p_data[3], "Lock Target State") == 0)
            m_hLockTargetState = handle;
        sprintf(trace, "0x%04x    %s     %s", handle, TypeString(p_data[2]), &p_data[3]);
        Log(trace);
        break;
    case HCI_CONTROL_HK_EVENT_TOKEN_DATA:
        if (p_data[0] & HCI_TOKEN_DATA_FLAG_UUID)
        {
            fp = fopen("token.txt", "w");
            if (fp)
            {
                fprintf(fp, "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
                    p_data[16], p_data[15], p_data[14], p_data[13], p_data[12], p_data[11], p_data[10], p_data[9],
                    p_data[8], p_data[7], p_data[6], p_data[5], p_data[4], p_data[3], p_data[2], p_data[1]);
                fclose(fp);
            }
            break;
        }

        if (p_data[0] & HCI_TOKEN_DATA_FLAG_START)
        {
            p_token_buf = software_token;
            software_token_size = 0;
        }

        memcpy(p_token_buf, &p_data[1], len - 1);
        p_token_buf += len - 1;
        software_token_size += len - 1;

        if (p_data[0] & HCI_TOKEN_DATA_FLAG_END)
        {
            fp = fopen("token.txt", "a");
            if (fp)
            {
                base64encode(software_token, software_token_size, software_token_encode, 1400);
                fprintf(fp, "%s\n", software_token_encode);
                fclose(fp);
            }
        }
        break;
    default:
        Log("HandleHkEvent  Rcvd Unsupported Op Code: 0x%04lx", opcode);
        break;
    }
}

void MainWindow::on_btnHelpHK_clicked()
{
    onClear();
    Log("HomeKit help topic:");
    Log("");
    Log("Apps : homekit_lightbulb and homekit_lock (MFI licensees only)");
    Log("Peer device : iOS 'My Home' app");
    Log("");
    Log("The apps simluates light bulb and lock devices that can be controlled by the iOS");

    ScrollToTop();
}
