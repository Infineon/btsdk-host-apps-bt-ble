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
 * Sample MCU application for implementing BLE GATT DB application using WICED HCI protocol.
 */

#include "app_include.h"
#include "wiced_bt_defs.h"
#include <QUuid>

extern "C"
{
#include "wiced_hci.h"
#include "app_host.h"
}


// Initialize app
void MainWindow::InitGATT_DB()
{
    //ui->btnHelpGATT_DB->setIcon(style()->standardIcon(QStyle::SP_MessageBoxQuestion));
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventGATT_DB(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_GATT:
        HandleGATT_DBEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for GATT event
void MainWindow::HandleGATT_DBEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    switch (opcode)
    {
        case HCI_CONTROL_GATT_EVENT_READ_REQUEST:
            HandleGattReadRequestEvent(p_data, len);
            break;
        case HCI_CONTROL_GATT_EVENT_WRITE_REQUEST:
            HandleGattReadRequestEvent(p_data, len);
            break;
    }
}

// Handle WICED HCI GATT Read Request event
void MainWindow::HandleGattReadRequestEvent(LPBYTE p_data, DWORD len)
{
    uint16_t conn_id;
    uint16_t handle;
    uint16_t offset;


    if (len < 8)
    {
        Log("%s: event length too short, len = %ld", __func__, len);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT16(handle,  p_data);
    STREAM_TO_UINT16(offset,  p_data);
    STREAM_TO_UINT16(len,     p_data);

    Log("%s: conn_id = %d, handle = %d, offset = %d, len = %ld", __func__, conn_id, handle, offset, len);

    // For real application, process Read
}


// Handle WICED HCI GATT Write Request event
void MainWindow::HandleGattWriteRequestEvent(LPBYTE p_data, DWORD len)
{
    uint16_t conn_id;
    uint16_t handle;
    uint16_t offset;
    uint16_t write_len;
    UNUSED(len);

    STREAM_TO_UINT16(conn_id,   p_data);
    STREAM_TO_UINT16(handle,    p_data);
    STREAM_TO_UINT16(offset,    p_data);
    STREAM_TO_UINT16(write_len, p_data);

    Log("%s: conn_id = %d, handle = %d, offset = %d, len = %d", __func__, conn_id, handle, offset, write_len);
    Log("data[0] = %d", p_data[0]);

    // For real application, process the Write
}

// Show error message if user did not enter value as 2 byte hex
void ShowGattMessageUint16(QString &strContext)
{
    QMessageBox msgBox;
    strContext += "Please enter value in 2 byte hex (ex. 12AB)";
    msgBox.setText(strContext);
    msgBox.exec();
}

// Check if value is 2 byte hex, if not show error message and return false.
// If valid, pass back the value and return true
bool ValidateUint16Hex(const QString &str, uint16_t & value, QString &strContext)
{
    // Check for XXXX string
    int len = str.length();
    if(len != 4)
    {
        ShowGattMessageUint16(strContext);
        return false;
    }

    // Check if it is valid
    bool ok = true;
    value = str.toUInt(&ok, 16);
    if(!ok)
    {
        ShowGattMessageUint16(strContext);
        return false;
    }

    return true;
}

// Show error message if user did not enter value as 1 byte hex
void ShowGattMessageUint8(QString &strContext)
{
    QMessageBox msgBox;
    strContext += "Please enter value in 1 byte hex (ex. 1A)";
    msgBox.setText(strContext);
    msgBox.exec();
}

// Check if value is 1 byte hex, if not show error message and return false.
// If valid, pass back the value and return true
bool ValidateUint8Hex(const QString &str, uint8_t & value, QString &strContext)
{
    // Check for 0xXX string
    int len = str.length();
    if(len != 2)
    {
        ShowGattMessageUint8(strContext);
        return false;
    }

    // Check if it is valid
    bool ok = true;
    value = str.toUInt(&ok, 16);
    if(!ok)
    {
        ShowGattMessageUint8(strContext);
        return false;
    }

    return true;
}

// Show error message if user did not enter value 16 bit or 128 bit UUID
void ShowGattMessageUUID(QString &strContext)
{
    QMessageBox msgBox;
    strContext += "Please enter UUID 16 bit 0xXX or 128 bit - five hex fields separated by '-' and enclosed in curly braces {xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}";
    msgBox.setText(strContext);
    msgBox.exec();
}

// Check if value is 16 bit or 128 bit UUID, if not show error message and return false.
// If valid, pass back the value and return true
bool ValidateUUID(const QString &str, uint16_t & value, QByteArray &arr, bool &isUUID16, QString &strContext)
{
    // Check if it is 2 byte hex (XXXX)
    if(str.length() <= 4)
    {
        isUUID16 = true;
        return ValidateUint16Hex(str, value, strContext);
    }

    // Check for 128 bit UUID
    // Create a temp QUuid to test the string and check if it is valid
    // It is formatted as five hex fields separated by '-' and
    // enclosed in curly braces, i.e., "{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}"
    isUUID16 = false;
    QUuid *uuid = new QUuid(str);
    if(uuid->isNull())
    {
        ShowGattMessageUUID(strContext);
        return false;
    }

    // pass back the QByteArray
    arr = uuid->toByteArray();
    delete uuid;
    return true;
}

/*
* Convert QByteArray of 128 bit UUID to 16 byte UUID array
*
* See documentation for QByteArray QUuid::toByteArray()
*
* Returns the 16 byte UUID array
* The input byte array is formatted as five hex fields separated by '-' and
* enclosed in curly braces, i.e., "{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}"
* where 'x' is a hex digit. From left to right, the five hex fields are
* obtained from the four public data members in QUuid as follows:
*/
void QByteArrayUUID128ToUint16Array(uint8_t *p_arr16, QByteArray &arr)
{
    uint8_t *data = (uint8_t *) arr.data();
    char str[40] = {0};

    // Create a temp string which concatenates all hex bytes
    // of the QByteArray array. Remove curly braces and '-'
    // The array is represented as {8-4-4-4-12}
    memset(str, 0, 40);

    memcpy(str,      &data[1],  8);
    memcpy(&str[8],  &data[10], 4);
    memcpy(&str[12], &data[15], 4);
    memcpy(&str[16], &data[20], 4);
    memcpy(&str[20], &data[25], 12);

    // The temp string length is 32.
    // Concatenate pair of adjoining char to 1 byte and convert it
    // to 16 byte (128 bit) UUID stream
    int index = 0;
    for(int i = 0; i < 16; i++)
    {
        index = i * 2;

        char str_temp[3];

        str_temp[0] = str[index];
        str_temp[1] = str[index+1];
        str_temp[2] = 0;

        QString qstr(str_temp);
        uint8_t ui = qstr.toUInt(NULL, 16);
        p_arr16[i] = ui;

    }
}

// Add Primary Service
void MainWindow::on_btnAddPrimarySvc_clicked()
{
    QString strContext;
    wiced_hci_bt_primary_service_data_t data;
    memset(&data, 0, sizeof(wiced_hci_bt_primary_service_data_t));

    uint16_t handle = 0;
    strContext = "Primary Service: Handle - ";
    if(!ValidateUint16Hex(ui->editPrimarySvcHandle->text(), handle, strContext))
        return;

    uint16_t uuid16 = 0;
    QByteArray arrUUID128;
    bool isUUID16;
    strContext = "Primary Service: UUID - ";
    if(!ValidateUUID(ui->editPrimarySvcUUID->text(),uuid16, arrUUID128, isUUID16, strContext))
        return;

    data.handle = handle;
    data.uuid.uuid_type_16 = isUUID16;

    if(isUUID16)
    {
        data.uuid.u.uuid16 = uuid16;
    }
    else
    {
        QByteArrayUUID128ToUint16Array(data.uuid.u.uuid128, arrUUID128);
    }

    wiced_hci_gatt_db_primary_service(&data);
    app_host_log("Send wiced_hci_gatt_db_primary_service");

}

// Add Secondary Service
void MainWindow::on_btnAddSecondarySvc_clicked()
{
    QString strContext;
    wiced_hci_bt_secondary_service_data_t data;
    memset(&data, 0, sizeof(wiced_hci_bt_secondary_service_data_t));

    uint16_t handle = 0;
    strContext = "Secondary Service: Handle - ";
    if(!ValidateUint16Hex(ui->editSecondarySvcHandle->text(), handle, strContext))
        return;

    uint16_t uuid16 = 0;
    QByteArray arrUUID128;
    bool isUUID16;
    strContext = "Secondary Service: UUID - ";
    if(!ValidateUUID(ui->editSecondarySvcUUID->text(),uuid16, arrUUID128, isUUID16, strContext))
        return;

    data.handle = handle;
    data.uuid.uuid_type_16 = isUUID16;

    if(isUUID16)
    {
        data.uuid.u.uuid16 = uuid16;
    }
    else
    {
        QByteArrayUUID128ToUint16Array(data.uuid.u.uuid128, arrUUID128);
    }

    wiced_hci_gatt_db_secondary_service(&data);
    app_host_log("Send wiced_hci_gatt_db_secondary_service");
}

// Add Included Service
void MainWindow::on_btnAddIncSvc_clicked()
{
    QString strContext;
    wiced_included_service_data_t data;
    memset(&data, 0, sizeof(wiced_included_service_data_t));

    uint16_t handle = 0;
    strContext = "Included Service: Handle - ";
    if(!ValidateUint16Hex(ui->editIncludedSvcHandle->text(), handle, strContext))
        return;

    data.included_svc.handle = handle;

    uint16_t svc_handle = 0;
    strContext = "Included Service: Service Handle - ";
    if(!ValidateUint16Hex(ui->editIncSvcServiceHandle->text(), svc_handle, strContext))
        return;

    data.svc_handle = svc_handle;

    uint16_t end_grp = 0;
    strContext = "Included Service: End Group - ";
    if(!ValidateUint16Hex(ui->editIncSvcEndGroup->text(), end_grp, strContext))
        return;

    data.end_grp = end_grp;

    uint16_t uuid16 = 0;
    QByteArray arrUUID128;
    bool isUUID16;
    strContext = "Included Service: UUID - ";
    if(!ValidateUUID(ui->editIncludedSvcUUID->text(),uuid16, arrUUID128, isUUID16, strContext))
        return;

    data.included_svc.uuid.uuid_type_16 = isUUID16;

    if(isUUID16)
    {
        data.included_svc.uuid.u.uuid16 = uuid16;
    }
    else
    {
        QByteArrayUUID128ToUint16Array(data.included_svc.uuid.u.uuid128, arrUUID128);
    }

    wiced_hci_gatt_db_included_service(&data);
    app_host_log("Send wiced_hci_gatt_db_included_service");

}

// Add Characteristic
void MainWindow::on_btnAddChar_clicked()
{
    QString strContext;
    wiced_characteristic_data_t data;
    memset(&data, 0, sizeof(wiced_characteristic_data_t));

    uint16_t handle = 0;
    strContext = "Characteristic : Handle - ";
    if(!ValidateUint16Hex(ui->editCharHandle->text(), handle, strContext))
        return;

    data.handle = handle;  // handle

    uint16_t handle_val = 0;
    strContext = "Characteristic : Handle Value - ";
    if(!ValidateUint16Hex(ui->editCharValue->text(), handle_val, strContext))
        return;

    data.handle_val = handle_val;

    uint8_t prop = 0;
    strContext = "Characteristic : Property - ";
    if(!ValidateUint8Hex(ui->editCharProperty->text(), prop, strContext))
        return;

    data.prop = prop;

    uint8_t perm = 0;
    strContext = "Characteristic : Permission - ";
    if(!ValidateUint8Hex(ui->editCharPermission->text(), perm, strContext))
        return;

    data.perm = perm;

    uint16_t uuid16 = 0;
    QByteArray arrUUID128;
    bool isUUID16;
    strContext = "Characteristic : UUID - ";
    if(!ValidateUUID(ui->editCharUUID->text(),uuid16, arrUUID128, isUUID16, strContext))
        return;

    data.uuid.uuid_type_16 = isUUID16;

    if(isUUID16)
    {
        data.uuid.u.uuid16 = uuid16;
    }
    else
    {
        QByteArrayUUID128ToUint16Array(data.uuid.u.uuid128, arrUUID128);
    }

    wiced_hci_gatt_db_characteristic(&data);
    app_host_log("Send wiced_hci_gatt_db_characteristic");
}

// Add Descriptor
void MainWindow::on_btnAddDesc_clicked()
{
    QString strContext;
    wiced_descriptor_data_t data;
    memset(&data, 0, sizeof(wiced_descriptor_data_t));

    uint16_t handle = 0;
    strContext = "Descriptor : Handle - ";
    if(!ValidateUint16Hex(ui->editDescHandle->text(), handle, strContext))
        return;

    data.handle = handle;

    uint8_t perm = 0;
    strContext = "Descriptor : Permission - ";
    if(!ValidateUint8Hex(ui->editDescPermission->text(), perm, strContext))
        return;

    data.perm = perm;

    uint16_t uuid16 = 0;
    QByteArray arrUUID128;
    bool isUUID16;
    strContext = "Descriptor : UUID - ";
    if(!ValidateUUID(ui->editDescUUID->text(),uuid16, arrUUID128, isUUID16, strContext))
        return;

    data.uuid.uuid_type_16 = isUUID16;

    if(isUUID16)
    {
        data.uuid.u.uuid16 = uuid16;
    }
    else
    {
        QByteArrayUUID128ToUint16Array(data.uuid.u.uuid128, arrUUID128); // UUID 128-bit
    }

    wiced_hci_gatt_db_descriptor(&data);
    app_host_log("Send wiced_hci_gatt_db_descriptor");
}

// Enter advertisment data and start advertisements
void MainWindow::on_btnStartAdvertGATTDB_clicked()
{
    BYTE command[32] = { 0 };
    int  commandBytes = 0;

    QString str = ui->editAdvertData->text();

    if(str.length() > 31)
    {
        QMessageBox msgBox;
        msgBox.setText("Enter advertisment data, 31 bytes max");
        msgBox.exec();
        return;
    }

    commandBytes = str.length();

    for(int i = 0; i < commandBytes; i++)
    {
        QString strChar = str.at(i);
        bool ok = true;
        uint8_t value = strChar.toUInt(&ok, 16);
        if(!ok)
        {
            QMessageBox msgBox;
            QString strContext = "Enter advertisement data as a hex array, for example '1234ABCD'";
            msgBox.setText(strContext);
            msgBox.exec();
            return;
        }
        command[i] = value;
    }

    wiced_hci_gatt_db_set_advert_data(command, commandBytes);
    app_host_log("Send wiced_hci_gatt_db_start_advert");

    // Start advertisments
    OnBnClickedStartStopAdvertisements();
}

// Init GATT database
void MainWindow::on_btnInitGATTDB_clicked()
{
    wiced_hci_gatt_db_init();
    Log("Send HCI_CONTROL_GATT_COMMAND_DB_INIT");

}


void MainWindow::on_btnHelpGATT_DB_clicked()
{
    onClear();
    Log("GATT DB help topic:");
    Log("");
    Log(" * Apps : gatt_db");
    Log("");
    Log(" * Download snip.ble.gatt_db application on your WICED device.");
    Log(" * Click on 'Init Database' button to initialize GATT database.");
    Log(" * Add the desired services, characteristics and descriptors to");
    Log("   populate GATT database. Use the + button to add.");
    Log(" * The UUID field can be 2 byte hex, represented as 0xXX or byte array is formatted as five hex");
    Log("   fields separated by '-' and enclosed in curly braces, i.e., {xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}");
    Log("   where 'x' is a hex digit. The Property and Permission fields are single 1 hex represented as 0xX.");
    Log("   All other fields are 2 byte hex represented as 0xXX.");
    Log(" * Add advertizement data (max 31 bytes) and click on 'Start Advert' button to start advertisment.");
    Log(" * Using a peer BLE device or app such as LightBlue, disover the GATT device and services");
    ScrollToTop();
}
