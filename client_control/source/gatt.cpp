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
 * Sample MCU application for GATT profile using WICED HCI protocol.
 */
#include "app_include.h"

extern "C"
{
#include "wiced_hci.h"
#include "app_host_gatt.h"
}

Q_DECLARE_METATYPE( CBtDevice* )

enum
{
    UUID_SERVICE_GAP                                    = 0x1800,
    UUID_SERVICE_GATT                                   = 0x1801,
    UUID_SERVICE_IMMEDIATE_ALERT                        = 0x1802,
    UUID_SERVICE_LINK_LOSS                              = 0x1803,
    UUID_SERVICE_TX_POWER                               = 0x1804,
    UUID_SERVICE_CURRENT_TIME                           = 0x1805,
    UUID_SERVICE_REFERENCE_TIME_UPDATE                  = 0x1806,
    UUID_SERVICE_DST_CHANGE                             = 0x1807,
    UUID_SERVICE_GLUCOSE                                = 0x1808,
    UUID_SERVICE_HEALTH_THERMOMETER                     = 0x1809,
    UUID_SERVICE_DEVICE_INFORMATION                     = 0x180A,
    UUID_SERVICE_NETWORK_AVAILABILITY                   = 0x180B,
    UUID_SERVICE_WATCHDOG                               = 0x180C,
    UUID_SERVICE_HEART_RATE                             = 0x180D,
    UUID_SERVICE_PHONE_ALERT_STATUS                     = 0x180E,
    UUID_SERVICE_BATTERY                                = 0x180F,
    UUID_SERVICE_BLOOD_PRESSURE                         = 0x1810,
    UUID_SERVICE_ALERT_NOTIFICATION                     = 0x1811,
    UUID_SERVICE_HID                                    = 0x1812,
    UUID_SERVICE_SCAN_PARAMETERS                        = 0x1813,
    UUID_SERVICE_RSC                                    = 0x1814,
    UUID_SERVICE_AUTOMATION_IO                          = 0x1815,
    UUID_SERVICE_CSC                                    = 0x1816,
    UUID_SERVICE_CYCLING_POWER                          = 0x1818,
    UUID_SERVICE_LOCATION_NAVIGATION                    = 0x1819,
    UUID_SERVICE_ENVIRONMENTAL_SENSING                  = 0x181A,
    UUID_SERVICE_BODY_COMPOSITION                       = 0x181B,
    UUID_SERVICE_USER_DATA                              = 0x181C,
    UUID_SERVICE_WEIGHT_SCALE                           = 0x181D,
    UUID_SERVICE_BOND_MANAGEMENT                        = 0x181E,
    UUID_SERVICE_CONTINUOUS_GLUCOSE_MONITORING          = 0x181F,
    UUID_SERVICE_INTERNET_PROTOCOL_SUPPORT              = 0x1820,
    UUID_SERVICE_INDOOR_POSITIONING                     = 0x1821,
    UUID_SERVICE_PULSE_OXIMETER                         = 0x1822,
    UUID_SERVICE_HTTP_PROXY                             = 0x1823,
    UUID_SERVICE_TRANSPORT_DISCOVERY                    = 0x1824,
    UUID_SERVICE_OBJECT_TRANSFER                        = 0x1825,
};

typedef struct
{
    uint16_t uuid16;
    QString service;
} gatt_uuid16_service_desc_t;


static gatt_uuid16_service_desc_t gatt_uuid16_service_desc[] =
{
        {UUID_SERVICE_GAP,                                    "GAP"},
        {UUID_SERVICE_GATT,                                   "GATT"},
        {UUID_SERVICE_IMMEDIATE_ALERT,                        "IMMEDIATE_ALERT"},
        {UUID_SERVICE_LINK_LOSS,                              "LINK_LOSS"},
        {UUID_SERVICE_TX_POWER,                               "TX_POWER"},
        {UUID_SERVICE_CURRENT_TIME,                           "CURRENT_TIME"},
        {UUID_SERVICE_REFERENCE_TIME_UPDATE,                  "REFERENCE_TIME_UPDATE"},
        {UUID_SERVICE_DST_CHANGE,                             "DST_CHANGE"},
        {UUID_SERVICE_GLUCOSE,                                "GLUCOSE"},
        {UUID_SERVICE_HEALTH_THERMOMETER,                     "HEALTH_THERMOMETER"},
        {UUID_SERVICE_DEVICE_INFORMATION,                     "DEVICE_INFORMATION"},
        {UUID_SERVICE_NETWORK_AVAILABILITY,                   "NETWORK_AVAILABILITY"},
        {UUID_SERVICE_WATCHDOG,                               "WATCHDOG"},
        {UUID_SERVICE_HEART_RATE,                             "HEART_RATE"},
        {UUID_SERVICE_PHONE_ALERT_STATUS,                     "PHONE_ALERT_STATUS"},
        {UUID_SERVICE_BATTERY,                                "BATTERY"},
        {UUID_SERVICE_BLOOD_PRESSURE,                         "BLOOD_PRESSURE"},
        {UUID_SERVICE_ALERT_NOTIFICATION,                     "ALERT_NOTIFICATION"},
        {UUID_SERVICE_HID,                                    "HID"},
        {UUID_SERVICE_SCAN_PARAMETERS,                        "SCAN_PARAMETERS"},
        {UUID_SERVICE_RSC,                                    "RSC"},
        {UUID_SERVICE_AUTOMATION_IO,                          "AUTOMATION_IO"},
        {UUID_SERVICE_CSC,                                    "CSC"},
        {UUID_SERVICE_CYCLING_POWER,                          "CYCLING_POWER"},
        {UUID_SERVICE_LOCATION_NAVIGATION,                    "LOCATION_NAVIGATION"},
        {UUID_SERVICE_ENVIRONMENTAL_SENSING,                  "ENVIRONMENTAL_SENSING"},
        {UUID_SERVICE_BODY_COMPOSITION,                       "BODY_COMPOSITION"},
        {UUID_SERVICE_USER_DATA,                              "USER_DATA"},
        {UUID_SERVICE_WEIGHT_SCALE,                           "WEIGHT_SCALE"},
        {UUID_SERVICE_BOND_MANAGEMENT,                        "BOND_MANAGEMENT"},
        {UUID_SERVICE_CONTINUOUS_GLUCOSE_MONITORING,          "CONTINUOUS_GLUCOSE_MONITORING"},
        {UUID_SERVICE_INTERNET_PROTOCOL_SUPPORT,              "INTERNET_PROTOCOL_SUPPORT"},
        {UUID_SERVICE_INDOOR_POSITIONING,                     "INDOOR_POSITIONING"},
        {UUID_SERVICE_PULSE_OXIMETER,                         "PULSE_OXIMETER"},
        {UUID_SERVICE_HTTP_PROXY,                             "HTTP_PROXY"},
        {UUID_SERVICE_TRANSPORT_DISCOVERY,                    "TRANSPORT_DISCOVERY"},
        {UUID_SERVICE_OBJECT_TRANSFER,                        "OBJECT_TRANSFER"},
};


typedef struct
{
    uint8_t uuid128[16];
    QString service;
} gatt_uuid128_service_desc_t;

static gatt_uuid128_service_desc_t gatt_uuid128_service_desc[] =
{
    {{0x7e, 0x60, 0xfa, 0xf2, 0xbe, 0x7e, 0x3b, 0xa6, 0xe0, 0x47, 0x9d, 0x05, 0xb2, 0x93, 0x52, 0x69}, "Cypress BT Serial GATT Service"},

    {{0x89, 0xD3, 0x50, 0x2B, 0x0F, 0x36, 0x43, 0x3A, 0x8E, 0xF4, 0xC5, 0x02, 0xAD, 0x55, 0xF8, 0xDC}, "Apple Media Service"},
    {{0x79, 0x05, 0xF4, 0x31, 0xB5, 0xCE, 0x4E, 0x99, 0xA4, 0x0F, 0x4B, 0x1E, 0x12, 0x2D, 0x00, 0xD0}, "Apple Notification Center Service"},
    {{0x9f, 0xa4, 0x80, 0xe0, 0x49, 0x67, 0x45, 0x42, 0x93, 0x90, 0xd3, 0x43, 0xdc, 0x5d, 0x04, 0xae}, "Apple Proprietary 1"},
    {{0xd0, 0x61, 0x1e, 0x78, 0xbb, 0xb4, 0x45, 0x91, 0xa5, 0xf8, 0x48, 0x79, 0x10, 0xae, 0x43, 0x66}, "Apple Proprietary 2"}

    /* Add other 128 UUID Service here */
};


 static const CHAR *szScanState[] =
 {
     "Scan:None",
     "Scan:High",
     "Scan:Low",
     "Conn:High",
     "Conn:Low",
 };

 static const CHAR *szAdvState[] =
 {
     "Advertisement:Not discoverable",
     "Advertisement:low directed",
     "Advertisement:high directed",
     "Advertisement:low undirected",
     "Advertisement:high undirected",
 };

 static const char *szStatusCode[] =
 {
     "Success",
     "In Progress",
     "Connected",
     "Not Connected",
     "Bad Handle",
     "Wrong State",
     "Invalid Args",
 };

 // Initialize app
void MainWindow::InitGATT()
{
    // setup signals/slots

    connect(ui->btnBLEConnect, SIGNAL(clicked()), this, SLOT(OnBnClickedLeConnect()));
    connect(ui->btnBLECancelConnect, SIGNAL(clicked()), this, SLOT(OnBnClickedLeCancelConnect()));
    connect(ui->btnBLEDisconnect, SIGNAL(clicked()), this, SLOT(OnBnClickedLeDisconnect()));
    connect(ui->btnBLEDiscoverServices, SIGNAL(clicked()), this, SLOT(OnBnClickedDiscoverServices()));
    connect(ui->btnBLEDiscoverChars, SIGNAL(clicked()), this, SLOT(OnBnClickedDiscoverCharacteristics()));
    connect(ui->btnBLEDiscoverDescriptors, SIGNAL(clicked()), this, SLOT(OnBnClickedDiscoverDescriptors()));
    connect(ui->btnBLEStartStopAdvert, SIGNAL(clicked()), this, SLOT(OnBnClickedStartStopAdvertisements()));
    connect(ui->btnBLEValueNotify, SIGNAL(clicked()), this, SLOT(OnBnClickedSendNotification()));
    connect(ui->btnBLEValueIndicate, SIGNAL(clicked()), this, SLOT(OnBnClickedSendIndication()));
    connect(ui->btnBLERead, SIGNAL(clicked()), this, SLOT(OnBnClickedCharacteristicRead()));
    connect(ui->btnBLEWrite, SIGNAL(clicked()), this, SLOT(OnBnClickedCharacteristicWrite()));
    connect(ui->btnBLEWriteNoRsp, SIGNAL(clicked()), this, SLOT(OnBnClickedCharacteristicWriteWithoutResponse()));

    connect(this, SIGNAL(HandleLeAdvState(BYTE)), this, SLOT(processHandleLeAdvState(BYTE)));

    m_advertisments_active = FALSE;

}

void MainWindow::setGATTUI()
{
    ui->btnBLEStartDisc->setEnabled(!m_scan_active );
    ui->btnBLEStopDisc->setEnabled(m_scan_active );
}

// User called connect
void MainWindow::OnBnClickedLeConnect()
{
    int item =  ui->cbBLEDeviceList->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->cbBLEDeviceList->itemData(item).value<CBtDevice *>();

    if (p_device == NULL)
        return;

    Log("LeConnect BtDevice : %02x:%02x:%02x:%02x:%02x:%02x",
        p_device->m_address[0], p_device->m_address[1], p_device->m_address[2], p_device->m_address[3],
            p_device->m_address[4], p_device->m_address[5]);

    app_host_gatt_connect(p_device->address_type, p_device->m_address);
}

// User called LE connect cancel
void MainWindow::OnBnClickedLeCancelConnect()
{
    int item = ui->cbBLEDeviceList->currentIndex();
    if (item < 0)
        return;

    CBtDevice *p_device = (CBtDevice *)ui->cbBLEDeviceList->itemData(item).value<CBtDevice *>();
    if (p_device == NULL)
        return;

    app_host_gatt_cancel_connect(p_device->address_type, p_device->m_address);
}

// Get device connection handle
USHORT MainWindow::GetConHandle(QComboBox *pCombo)
{
    int sel;
    if ((sel = pCombo->currentIndex()) >= 0)
        return ((CBtDevice *)pCombo->itemData(sel).value<CBtDevice *>())->con_handle;
    return 0;
}

// User called LE disconnect
void MainWindow::OnBnClickedLeDisconnect()
{
    USHORT con_handle = GetConHandle(ui->cbBLEDeviceList);
    app_host_gatt_le_disconnect(con_handle);
}

// Discover services
void MainWindow::OnBnClickedDiscoverServices()
{
    USHORT con_handle = GetConHandle(ui->cbBLEDeviceList);
    app_host_gatt_disc_services(con_handle);
}

// Discover characteristics
void MainWindow::OnBnClickedDiscoverCharacteristics()
{
    QString str1 = ui->edtBLEHandleStart->text();
    DWORD s_handle = GetHandle(str1);
    QString str2 = ui->edtBLEHandleEnd->text();
    DWORD e_handle = GetHandle(str2);

    USHORT con_handle   = GetConHandle(ui->cbBLEDeviceList);
    app_host_gatt_disc_chars(s_handle, e_handle, con_handle);
}

// Discover Descriptors
void MainWindow::OnBnClickedDiscoverDescriptors()
{
    QString str1 = ui->edtBLEHandleStart->text();
    DWORD s_handle = GetHandle(str1);
    QString str2 = ui->edtBLEHandleEnd->text();
    DWORD e_handle = GetHandle(str2);

    USHORT con_handle   = GetConHandle(ui->cbBLEDeviceList);
    app_host_gatt_disc_desc(s_handle, e_handle, con_handle);
}

// Start or stop advertisements
void MainWindow::OnBnClickedStartStopAdvertisements()
{
    if (!m_advertisments_active)
    {
        m_advertisments_active = TRUE;
        ui->btnBLEStartStopAdvert->setText("Stop Adverts");
    }
    else
    {
        m_advertisments_active = FALSE;
        ui->btnBLEStartStopAdvert->setText("Start Adverts");
    }

    app_host_gatt_start_stop_advert(m_advertisments_active);
}

// Send notifications
void MainWindow::OnBnClickedSendNotification()
{
    QString str = ui->edtBLEHandle->text();
    USHORT con_handle   = GetConHandle(ui->cbBLEDeviceList);
    DWORD hdlc = GetHandle(str);
    static DWORD num_bytes;
    static unsigned char prev_send = 0;
    BYTE buffer[32]={0};
    UNUSED(prev_send); // silence warning

#ifdef REPEAT_NOTIFICATIONS_FOREVER
    if (sending_notifications)
    {
        command[commandBytes] = prev_send++;
    }
    else
#endif
    {
        QString str2 = ui->edtBLEHandleValue->text();
        //num_bytes = GetHexValue(&command[commandBytes], sizeof(command) - commandBytes, str);
        num_bytes = GetHexValue(buffer, sizeof(buffer), str2);
        prev_send = buffer[0] + 1;
    }

    app_host_gatt_send_notif(con_handle, hdlc, buffer, num_bytes);
}


// Send indications
void MainWindow::OnBnClickedSendIndication()
{
    USHORT con_handle   = GetConHandle(ui->cbBLEDeviceList);
    BYTE buffer[32]={0};
    QString str1 = ui->edtBLEHandle->text();
    DWORD hdlc = GetHandle(str1);
    DWORD num_bytes;
    QString str2 = ui->edtBLEHandleValue->text();
    num_bytes = GetHexValue(buffer, sizeof(buffer), str2);

    app_host_gatt_send_indicate(con_handle, hdlc, buffer, num_bytes);
}

// Read characteristics
void MainWindow::OnBnClickedCharacteristicRead()
{
    USHORT con_handle   = GetConHandle(ui->cbBLEDeviceList);
    QString str = ui->edtBLEHandle->text();
    DWORD  hdlc         = GetHandle(str);
    app_host_gatt_read_char(con_handle,hdlc);
}

// Write characteristics
void MainWindow::OnBnClickedCharacteristicWrite()
{
    USHORT con_handle = GetConHandle(ui->cbBLEDeviceList);
    QString str1 = ui->edtBLEHandle->text();
    DWORD hdlc = GetHandle(str1);
    QString str2 = ui->edtBLEHandleValue->text();
    uint8_t value[32];
    DWORD num_bytes = GetHexValue(value, sizeof(value), str2);
    app_host_gatt_write_char(con_handle, hdlc, value, num_bytes);
}

// Write characteristics Without Response
void MainWindow::OnBnClickedCharacteristicWriteWithoutResponse()
{
    USHORT con_handle = GetConHandle(ui->cbBLEDeviceList);
    QString str1 = ui->edtBLEHandle->text();
    DWORD hdlc = GetHandle(str1);
    QString str2 = ui->edtBLEHandleValue->text();
    uint8_t value[32];
    DWORD num_bytes = GetHexValue(value, sizeof(value), str2);
    app_host_gatt_write_char_no_rspn(con_handle, hdlc, value, num_bytes);
}

// Set role as slave or master
void MainWindow::SetRole(CBtDevice *pDevice, uint8_t role)
{
    pDevice->role = role;
}

uint8_t MainWindow::GetRole(CBtDevice *pDevice)
{
    return pDevice->role;
}

// Update UI
void MainWindow::UpdateGattButtons(CBtDevice *pDevice)
{
    BOOL enable = 1;
    UNUSED(pDevice);

    // Enable GATT Buttons for both Master and Slave connections
    ui->btnBLEDiscoverServices->setEnabled(enable);
    ui->btnBLEDiscoverChars->setEnabled(enable);
    ui->btnBLEDiscoverDescriptors->setEnabled(enable);
    ui->btnBLERead->setEnabled(enable);
    ui->btnBLEWrite->setEnabled(enable);
    ui->btnBLEWriteNoRsp->setEnabled(enable);
    ui->btnBLEValueNotify->setEnabled(enable);
    ui->btnBLEValueIndicate->setEnabled(enable);
}

// Event received for LE adv state
void MainWindow::processHandleLeAdvState(BYTE val)
{
    ui->lblBLEAdvertisementState->setText(szAdvState[val]);

    if (val)
    {
        m_advertisments_active = TRUE;
        ui->btnBLEStartStopAdvert->setText("Stop Adverts");
    }
    else
    {
        m_advertisments_active = FALSE;
        ui->btnBLEStartStopAdvert->setText("Start Adverts");
    }

    Log("Advertisement:%d", val);
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventGATT(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_ANCS:
    case HCI_CONTROL_GROUP_ALERT:
        HandleLEEvents(opcode, p_data, len);
        break;
    case HCI_CONTROL_GROUP_GATT:
        HandleGattEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for LE
void MainWindow::HandleLEEvents(DWORD identifier, LPBYTE p_data, DWORD len)
{
    CHAR   trace[1024];
    BYTE    bda[6];
    CBtDevice *device;

    switch (identifier)
    {
    case HCI_CONTROL_LE_EVENT_SCAN_STATUS:

        ui->lblBLEScanState->setText(szScanState[p_data[0]]);
        sprintf (trace, "Scan:%d", p_data[0]);
        if(p_data[0] == HCI_CONTROL_SCAN_EVENT_NO_SCAN)
        {
            m_scan_active = false;
            setGATTUI();
        }
        Log(trace);
        break;

    case HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT:
    {
        char bd_name[50] = {0};



        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[2 + i];
        // check if this device is not present yet.
        if (FindInList(bda, ui->cbBLEDeviceList) != NULL)
            break;

        sprintf (trace, "Advertisement report:%d type:%d address %02x:%02x:%02x:%02x:%02x:%02x rssi:%d",
            p_data[0], p_data[1], p_data[7], p_data[6], p_data[5], p_data[4], p_data[3], p_data[2],
            p_data[8] - 256);
        Log(trace);

        // dump advertisement data to the trace
        if (len > 10)
        {
#if 0
            trace[0] = 0;
            for (int i = 0; i < (int)len - 8; i++)
                sprintf(&trace[strlen(trace)], "%02x", p_data[9 + i]);
            Log(trace);
#endif

            DecodeEIR(&p_data[9], len - 9, bd_name, sizeof(bd_name));

        }

        device = AddDeviceToList(bda, ui->cbBLEDeviceList, bd_name); // /* p_data[0], */ bda, p_data[7] + (p_data[8] << 8), CONNECTION_TYPE_LE);
        device->m_conn_type |= CONNECTION_TYPE_LE;
        device->address_type = p_data[1];
        device->m_bIsLEDevice = true;
    }
    break;
    case HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE:

        emit HandleLeAdvState(p_data[0]);

        break;
    case HCI_CONTROL_LE_EVENT_CONNECTED:
        sprintf (trace, "Connection up:type:%d address %02x:%02x:%02x:%02x:%02x:%02x connection handle:%04x role:%d",
            p_data[0], p_data[6], p_data[5], p_data[4], p_data[3], p_data[2], p_data[1], p_data[7] + (p_data[8] << 8), p_data[9]);
        Log(trace);

        // Check if the device is connected in Peripheral (Slave) role.
        if (p_data[9])
        {
            // If the device is connected in a peripheral role adv. would be stopped automatically.
            // Hence the button's state needs to be changed as well.
            m_advertisments_active = FALSE;
            ui->btnBLEStartStopAdvert->setText("Start Adverts");
        }

        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[1 + i];

        setHIDD_linkChange(bda, TRUE);

        // find device in the list with received address and save the connection handle
        device = AddDeviceToList(bda, ui->cbBLEDeviceList, NULL); // /* p_data[0], */ bda, p_data[7] + (p_data[8] << 8), CONNECTION_TYPE_LE);

        device->con_handle = p_data[7] + (p_data[8] << 8);
        device->m_conn_type |= CONNECTION_TYPE_LE;
        device->m_bIsLEDevice = true;

        SetRole(device, p_data[9]);     // Save LE role
        SelectDevice(ui->cbBLEDeviceList, bda);
        UpdateGattButtons(device);
        break;

    case HCI_CONTROL_LE_EVENT_DISCONNECTED:
        sprintf (trace, "Connection down:connection handle:%04x reason:0x%x",
            p_data[0] + (p_data[1] << 8), p_data[2]);
        Log(trace);
        ui->lblCTMessage->setText("");
        ui->lblCTTitle->setText("");
        ui->btnCTANCSPositive->setText("");
        ui->btnCTANCSNegative->setText("");
        setHIDD_linkChange(nullptr, FALSE);
        break;

    case HCI_CONTROL_ANCS_EVENT_NOTIFICATION:
        m_notification_uid = p_data[0] + (p_data[1] << 8) + (p_data[2] << 16), (p_data[3] << 24);
        sprintf (trace, "(ANCS) %04lu Command:%u Category:%u Flags:%u", m_notification_uid, p_data[4], p_data[5], p_data[6]);
        Log(trace);

        // notification Added or Modified
        if ((p_data[4] == 0) || (p_data[4] == 1))
        {
            int len_int = 7;
            ui->lblCTMessage->setText((char*)&p_data[len_int]);

            len_int += (int)strlen((char *)&p_data[len_int]) + 1;
            ui->lblCTTitle->setText((char*)&p_data[len_int]);

            len_int += (int)strlen((char *)&p_data[len_int]) + 1;
            ui->btnCTANCSPositive->setText((char*)&p_data[len_int]);

            len_int += (int)strlen((char *)&p_data[len_int]) + 1;
            ui->btnCTANCSNegative->setText((char*)&p_data[len_int]);
        }
        else // removed
        {
            ui->lblCTMessage->setText("Message");
            ui->lblCTTitle->setText("Tile");
            ui->btnCTANCSPositive->setText("ANCS Positive");
            ui->btnCTANCSNegative->setText("ANCS Negative");
        }

        break;

    case HCI_CONTROL_EVENT_COMMAND_STATUS:
        sprintf (trace, "Status: %s", (p_data[0] <= HCI_CONTROL_STATUS_INVALID_ARGS) ? szStatusCode[p_data[0]] : "????");
        Log(trace);
        break;
    }
}

// Handle WICED HCI events for GATT
void MainWindow::HandleGattEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    CHAR   trace[1024];

    switch (opcode)
    {
    case HCI_CONTROL_GATT_EVENT_COMMAND_STATUS:
        sprintf (trace, "GATT command status:%x", p_data[0]);
        Log(trace);
        break;
    case HCI_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE:
        sprintf (trace, "Discovery Complete connection handle:%04x", p_data[0] + (p_data[1] << 8));
        Log(trace);
        break;
    case HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED:
        if (len == 8)
        {
            sprintf (trace, "Service:UUID %04x(%s) start handle:%04x end handle %04x",
                p_data[2] + (p_data[3] << 8),
                GetServiceUUIDDesc(p_data[2] + (p_data[3] << 8)).toStdString().c_str(),
                p_data[4] + (p_data[5] << 8),
                p_data[6] + (p_data[7] << 8));
        }
        else
        {
            sprintf (trace, "Service:UUID %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x (%s) start handle:%04x end handle %04x",
                p_data[2],  p_data[3],  p_data[4],  p_data[5],  p_data[6],  p_data[7],  p_data[8],  p_data[9],
                p_data[10], p_data[11], p_data[12], p_data[13], p_data[14], p_data[15], p_data[16], p_data[17],
                GetServiceUUIDDesc(&p_data[2]).toStdString().c_str(),
                p_data[18] + (p_data[19] << 8), p_data[20] + (p_data[21] << 8));
        }
        Log(trace);
        break;
    case HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED:
        if (len == 9)
        {
            sprintf (trace, "Characteristic:char handle:%04x UUID %04x Properties:0x%02x value handle:%04x",
                p_data[2] + (p_data[3] << 8), p_data[4] + (p_data[5] << 8), p_data[6], p_data[7] + (p_data[8] << 8));
        }
        else
        {
            sprintf (trace, "Characteristic:char handle:%04x UUID %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x Properties:0x%02x value handle:%04x",
                p_data[2] + (p_data[3] << 8),
                p_data[4],  p_data[5],  p_data[6],  p_data[7],  p_data[8],  p_data[9],  p_data[10], p_data[11],
                p_data[12], p_data[13], p_data[14], p_data[15], p_data[16], p_data[17], p_data[18], p_data[19],
                p_data[20], p_data[21] + (p_data[22] << 8));
        }
        Log(trace);
        break;
    case HCI_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED:
        if (len == 6)
        {
            sprintf (trace, "Descriptor:UUID %04x handle:%04x",
                p_data[2] + (p_data[3] << 8), p_data[4] + (p_data[5] << 8));
        }
        else
        {
            sprintf (trace, "Descriptor:UUID %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x handle:%04x",
                p_data[2], p_data[3], p_data[4], p_data[5], p_data[6], p_data[7], p_data[8], p_data[9],
                p_data[10], p_data[11], p_data[12], p_data[13], p_data[14], p_data[15], p_data[16], p_data[17],
                p_data[18] + (p_data[19] << 8));
        }
        Log(trace);
        break;
    case HCI_CONTROL_GATT_EVENT_READ_RESPONSE:
        sprintf (trace, "Read Response [%02x] ", p_data[0] + (p_data[1] << 8));
        for (int i = 0; i < (int)len - 2; i++)
            sprintf(&trace[strlen(trace)], "%02x", p_data[2 + i]);
        Log(trace);
        ui->edtBLEHandleValue->setText(trace);

        break;
    case HCI_CONTROL_GATT_EVENT_READ_REQUEST:
        sprintf (trace, "Read Req:Conn:%04x handle:%04x", p_data[0] + (p_data[1] << 8), p_data[2] + (p_data[3] << 8));
        Log(trace);
        app_host_gatt_read_response(p_data);
        break;
    case HCI_CONTROL_GATT_EVENT_WRITE_REQUEST:
        sprintf (trace, "Write Req:Conn:%04x handle:%04x", p_data[0] + (p_data[1] << 8), p_data[2] + (p_data[3] << 8));
        app_host_gatt_write_response(p_data);
        break;

    case HCI_CONTROL_GATT_EVENT_WRITE_RESPONSE:
        if (len == 2)
            sprintf (trace, "Write Rsp [%x] ", p_data[0] + (p_data[1] << 8));
        else
            sprintf (trace, "Write Rsp [%x] result:0x%x", p_data[0] + (p_data[1] << 8), p_data[2]);
        Log(trace);
#ifdef REPEAT_NOTIFICATIONS_FOREVER
        if (sending_notifications)
        {
            OnBnClickedSendNotification();
        }
#endif
        break;
    case HCI_CONTROL_GATT_EVENT_NOTIFICATION:
        sprintf (trace, "Notification [%02x] handle:%04x ", p_data[0] + (p_data[1] << 8), p_data[2] + (p_data[3] << 8));
        for (int i = 0; i < (int)len - 4; i++)
            sprintf(&trace[strlen(trace)], "%02x", p_data[4 + i]);
        Log(trace);
        break;
    case HCI_CONTROL_GATT_EVENT_INDICATION:
        sprintf (trace, "Indication [%02x] handle:%04x ", p_data[0] + (p_data[1] << 8), p_data[2] + (p_data[3] << 8));
        for (int i = 0; i < (int)len - 4; i++)
            sprintf(&trace[strlen(trace)], "%02x", p_data[4 + i]);
        Log(trace);
        break;

    }
}

// Debug function to get the Description of a 16 bits UUID Service
QString MainWindow::GetServiceUUIDDesc(uint16_t uuid16)
{
    for (uint32_t i = 0 ; i < (sizeof(gatt_uuid16_service_desc) / sizeof(gatt_uuid16_service_desc_t)) ; i++)
    {
        if (uuid16 == gatt_uuid16_service_desc[i].uuid16)
            return gatt_uuid16_service_desc[i].service;
    }
    return "Unknown";
}

// Debug function to get the Description of a 128 bits UUID Service
QString MainWindow::GetServiceUUIDDesc(uint8_t *p_uuid128)
{

    for (uint32_t i = 0 ; i < (sizeof(gatt_uuid128_service_desc) / sizeof(gatt_uuid128_service_desc_t)) ; i++)
    {
        if (memcmp(p_uuid128, gatt_uuid128_service_desc[i].uuid128, 16) == 0)
            return gatt_uuid128_service_desc[i].service;
    }
    return "Unknown";
}



void MainWindow::on_btnHelpGATT_clicked()
{
    onClear();
    Log("GATT help topic:");
    Log("");
    Log("Apps : watch");
    Log("Peer device - BLE device or iPhone Light Blue app, Android BLE app, etc.");
    Log("");

    Log("GATT controls are provided for advertisements, discovering services, connecting");
    Log("to a GATT server, reading/writing values of handles, and discovering");
    Log("characteristics and descriptors of handles.");

    ScrollToTop();
}
