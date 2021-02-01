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
 * Sample MCU application for using WICED HCI protocol.
 * This module implements device manager (DM) funtonality that
 * includes  -
 * - Local device management
 * - Peer device management
 * - WICED HCI handling
 * - Firmware download
 * - Serial port management
 * - Serial port thread
 */

#include "app_include.h"
#include "rpc.pb.h"
#include <time.h>
#include <QTimer>

extern "C"
{
#include "app_host_dm.h"
extern void msleep(unsigned int to);
}


#define NVRAM_GROUP         "NVRAM"
#define HID_HOST_GROUP      "HID-Host"
#define HID_BATTC_GROUP     "Battery-Client"
#define HID_FINDMEL_GROUP   "FindMe-Locator"

#define HCI_VSE_OPCODE                      0xFF
#define HCI_VSE_SUBOPCODE_DBFW_DUMP         0x1B
#define HCI_VSE_TYPE_COREDUMP               0x03

#define HCI_VSE_TYPE_COREDUMP_INFO          0x01
#define HCI_VSE_TYPE_COREDUMP_CPU_REGS      0x02
#define HCI_VSE_TYPE_COREDUMP_RAM_IMAGE     0x03
#define HCI_VSE_TYPE_COREDUMP_CPU_REGS_EXT  0x05
#define HCI_VSE_TYPE_COREDUMP_CALL_STACK    0x06

extern void TraceHciPkt(BYTE type, BYTE *buffer, USHORT length, USHORT serial_port_index, int iSpyInstance);
Q_DECLARE_METATYPE( CBtDevice* )

// CBtDevice data structure for caching peer device info
CBtDevice::CBtDevice (bool paired) :
    m_nvram(nullptr), m_nvram_id(-1), m_paired(paired)
{
    memset(m_address,0,sizeof(m_address));
    memset(m_name,0,sizeof(m_name));

    m_audio_handle = NULL_HANDLE;
    m_hf_handle = NULL_HANDLE;
    m_ag_handle = NULL_HANDLE;
    m_spp_handle = NULL_HANDLE;
    m_hidh_handle = NULL_HANDLE;
    m_iap2_handle = NULL_HANDLE;
    m_avrc_handle = NULL_HANDLE;
    m_avk_handle = NULL_HANDLE;
    m_bsg_handle = NULL_HANDLE;
    m_pbc_handle = NULL_HANDLE;
    m_battc_handle = NULL_HANDLE;
    m_findmel_handle = NULL_HANDLE;
    m_mce_handle = NULL_HANDLE;
    m_conn_type = 0;
    m_bIsLEDevice = false;
    role = 0;
    address_type = 0;
    con_handle = 0;
}

CBtDevice::~CBtDevice ()
{
}

// valid baud rates
static int as32BaudRate[] =
{
    115200,
    921600,
    3000000,
#ifndef __MACH__
    4000000
#endif
};


// Initialize app
void MainWindow::InitDm()
{
    m_scan_active = m_inquiry_active = false;
    m_CommPort = nullptr;

    m_port_read_worker = nullptr;
    m_port_read_thread = nullptr;

    EnableUI(false);

    // read settings for baudrate, serial port and flow-ctrl
    //QSettings settings(m_SettingsFile, QSettings::IniFormat);
    int baud_rate = m_settings.value("Baud",3000000).toInt();
    bool flow_control = m_settings.value("FlowControl",true).toBool();
    QString comm_port = m_settings.value("port").toString();
    ui->btnPairable->setChecked(m_settings.value("pairing_mode",true).toBool());
    ui->btnDiscoverable->setChecked(m_settings.value("discoverable",true).toBool());
    ui->btnConnectable->setChecked(m_settings.value("Connectable",true).toBool());

    // setup icon in device list for paried devices
    ui->cbDeviceList->setIconSize(QSize(20,20));
    ui->cbBLEDeviceList->setIconSize(QSize(20,20));

    // get paired devices
    ReadDevicesFromSettings("devices", ui->cbDeviceList, ui->btnUnbond);
    ReadDevicesFromSettings("devicesLE", ui->cbBLEDeviceList, ui->btnBLEUnbond);

    // get list of all available serial ports
    int port_inx = -1;
    m_strComPortsIDs.clear();
    m_strComPortsIDs.append("<select port>"); // dummy string to match combo box
    QList<QSerialPortInfo> port_list = QSerialPortInfo::availablePorts();
    for (int i =0; i < port_list.size(); i++)
    {
        QString strName = port_list.at(i).portName();
        QString strDesc =  port_list.at(i).description();
        strName += " (" + strDesc + ")";
        QString strPortID = port_list.at(i).systemLocation();

        // m_strComPortsIDs contains serial port ID used to open the port
        m_strComPortsIDs.append(strPortID);

        // cbCommport contains friendly names
        ui->cbCommport->addItem(strName, strPortID);
    }

#ifndef Q_OS_WINDOWS
#ifndef Q_OS_MACOS
    // add entry in list for emulator
    ui->cbCommport->addItem(QString("host-mode"), "0");
    m_strComPortsIDs.append("0");
#endif
#endif

    if ( -1 != (port_inx = ui->cbCommport->findText(comm_port)))
    {
        ui->cbCommport->setCurrentIndex(port_inx);
    }

    // populate dropdown list of baud rates
    QString strBaud;
    int baud_inx = (sizeof(as32BaudRate) / sizeof(as32BaudRate[0])) - 1; // select default baud rate as highest allowed
    for (int i = 0; i < static_cast<int>( (sizeof(as32BaudRate) / sizeof(as32BaudRate[0]))); i++)
    {
        strBaud.sprintf( "%d", as32BaudRate[i]);
        ui->cbBaudRate->addItem(strBaud,i);
        if (as32BaudRate[i] == baud_rate)
            baud_inx = i;
    }
    ui->cbBaudRate->setCurrentIndex(baud_inx);
    ui->btnFlowCntrl->setChecked(flow_control );

    // Startup timer
    m_dmStartupTimer = new QTimer(this);
    m_dmStartupTimer->setInterval(2000);
    m_dmStartupTimer->setSingleShot(true);
    connect(m_dmStartupTimer, SIGNAL(timeout()), this, SLOT(startUpTimer()));

    // setup signals/slots
    connect(ui->btnStartDisc, SIGNAL(clicked()), this, SLOT(onStartDisc()));
    connect(ui->btnStopDisc, SIGNAL(clicked()), this, SLOT(onStopDisc()));
    connect(ui->btnReset, SIGNAL(clicked()), this, SLOT(onReset()));
    connect(ui->btnUnbond, SIGNAL(clicked()), this, SLOT(OnBnClickedBREDRUnbond()));
    connect(ui->btnBLEUnbond, SIGNAL(clicked()), this, SLOT(OnBnClickedLeUnbond()));
    connect(ui->cbDeviceList, SIGNAL(currentTextChanged(QString)), this, SLOT(onDevChange(QString)));
    connect(ui->cbBLEDeviceList, SIGNAL(currentTextChanged(QString)), this, SLOT(onLEDevChange(QString)));
    connect(ui->btnDiscoverable, SIGNAL(clicked(bool)), this, SLOT(onDiscoverable(bool)));
    connect(ui->btnConnectable, SIGNAL(clicked(bool)), this, SLOT(onConnectable(bool)));
    connect(ui->btnPairable, SIGNAL(clicked(bool)), this, SLOT(onPairable(bool)));
    connect(ui->btnDownload, SIGNAL(clicked()), this, SLOT(onDownload()));
    connect(ui->btnFindPatchFile, SIGNAL(clicked()), this, SLOT(onFindPatchFile()));
    connect(&dl_msgbox,SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(onMsgBoxButton(QAbstractButton*)));
    connect(ui->btnBLEStartDisc, SIGNAL(clicked()), this, SLOT(OnBnClickedDiscoverDevicesStart()));
    connect(ui->btnBLEStopDisc, SIGNAL(clicked()), this, SLOT(OnBnClickedDiscoverDevicesStop()));
    connect(ui->btnVersionInfo, SIGNAL(clicked()), this, SLOT(OnBnClickedVersionInfo()));

    ui->edPatchFile->setText(m_settings.value("FirmwareFile","").toString());
    m_bPortOpen = false;
    m_bPeripheralUart = false;

    m_major = 0;
    m_minor = 0;
    m_rev = 0;
    m_build = 0;
    m_chip = 0;
    m_features = 0xFF;
}

// Enable or disable UI
void MainWindow::EnableUI(bool bEnable)
{
    EnableTabs(0xFF, bEnable);

    ui->btnStartDisc->setEnabled(bEnable);
    ui->btnBLEStartDisc->setEnabled(bEnable);
    ui->btnBLEStopDisc->setEnabled(bEnable );
    ui->btnReset->setEnabled(bEnable);
    ui->btnStopDisc->setEnabled(bEnable);
    ui->btnPairable->setEnabled(bEnable);
    ui->btnDiscoverable->setEnabled(bEnable);
    ui->btnConnectable->setEnabled(bEnable);
    ui->btnVersionInfo->setEnabled(bEnable);
    ui->btnDownload->setEnabled(bEnable);

    m_bUIEnabled = bEnable;
}

// Enable UI tabs depending on supported features of embedded device
void MainWindow::EnableTabs(UINT8 feature, bool bEnable)
{
    if((feature != 0xFF) && bEnable)
    {
        switch(feature)
        {
        case HCI_CONTROL_GROUP_GATT:
            ui->tabGATT->setEnabled(bEnable);
            ui->tabGATT_DB->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabGATT);
            Log("GATT");
            break;
        case HCI_CONTROL_GROUP_HF:
            ui->tabHF->setEnabled(bEnable);
            ui->tabHF_continue->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabHF);
            Log("HF");
            break;
        case HCI_CONTROL_GROUP_SPP:
            ui->tabSPP->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabSPP);
            Log("SPP");
            break;
        case HCI_CONTROL_GROUP_AUDIO:
            ui->tabAVSRC->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAVSRC);
            Log("AV Source");
            break;
        case HCI_CONTROL_GROUP_AUDIO_DUAL_A2DP:
            ui->tabDualA2DP->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabDualA2DP);
            Log("Dual A2DP");
            break;
        case HCI_CONTROL_GROUP_HIDD:
            ui->tabHIDD->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabHIDD);
            m_b_is_hidd = true;
            Log("HID Device");
            UpdateHIDD_ui_pairing();
            UpdateHIDD_ui_host();
            app_host_hidd_get_host_info();
            break;
        case HCI_CONTROL_GROUP_AVRC_TARGET:
            ui->tabAVRCTG->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAVRCTG);
            Log("AVRC TG");
            break;
        case HCI_CONTROL_GROUP_ANCS:
            ui->tabAVRCCT->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAVRCCT);
            Log("AVRC CT (ANCS)");
            break;
        case HCI_CONTROL_GROUP_IAP2:
            ui->tabIAP2->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabIAP2);
            Log("iAP2");
            break;
        case HCI_CONTROL_GROUP_AG:
            ui->tabAG->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAG);
            Log("AG");
            break;
        case HCI_CONTROL_GROUP_BSG:
            ui->tabBSG->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabBSG);
            Log("BSG");
            break;
        case HCI_CONTROL_GROUP_PBC:
            ui->tabPBC->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabPBC);
            Log("PBC");
            break;
        case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
            ui->tabAVRCCT->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAVRCCT);
            Log("AVRC CT");
            break;
        case HCI_CONTROL_GROUP_AMS:
            ui->tabAVRCCT->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAVRCCT);
            Log("AVRC CT (AMS)");
            break;
        case HCI_CONTROL_GROUP_HIDH:
            ui->tabHIDH->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabHIDH);
            Log("HID Host");
            break;
        case HCI_CONTROL_GROUP_HK:
            ui->tabHK->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabHK);
            Log("HomeKit");
            StartHK();
            break;
        case HCI_CONTROL_GROUP_AUDIO_SINK:
            ui->tabAVSink->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAVSink);
            Log("AV Sink");
            break;
        case HCI_CONTROL_GROUP_BATT_CLIENT:
            ui->tabBATTC->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabBATTC);
            Log("Battery Client");
            break;
        case HCI_CONTROL_GROUP_FINDME_LOCATOR:
            ui->tabFINDMEL->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabFINDMEL);
            Log("FindMe Locator");
            break;
        case HCI_CONTROL_GROUP_OPS:
            ui->tabOPS->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabOPS);
            Log("OPP");
            break;
        case HCI_CONTROL_GROUP_ANS:
        case HCI_CONTROL_GROUP_ANC:
            ui->tabAlertNotfn->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabAlertNotfn);
            Log("ANP (ANS/ANC)");
            break;
        case HCI_CONTROL_GROUP_LED_DEMO:
            ui->tabLedDemo->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabLedDemo);
            Log("LED DEMO");
            break;
        case HCI_CONTROL_GROUP_LE_COC:
            ui->tabLECOC->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabLECOC);
            Log("LE COC");
            break;
        case HCI_CONTROL_GROUP_DEMO:
            ui->tabDemo->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabDemo);
            Log("DEMO");
            break;
        case HCI_CONTROL_GROUP_MCE:
            ui->tabMAPClient->setEnabled(bEnable);
            ui->tabMain->setCurrentWidget(ui->tabMAPClient);
            Log("MAP Client");
            break;
        }
    }
    else
    {
        ui->tabAG->setEnabled(bEnable);
        ui->tabHF->setEnabled(bEnable);
        ui->tabHF_continue->setEnabled(bEnable);
        ui->tabAVSRC->setEnabled(bEnable);
        ui->tabDualA2DP->setEnabled(bEnable);
        ui->tabAVRCCT->setEnabled(bEnable);
        ui->tabAVRCTG->setEnabled(bEnable);
        ui->tabHIDD->setEnabled(bEnable);
        ui->tabHIDH->setEnabled(bEnable);
        ui->tabSPP->setEnabled(bEnable);
        ui->tabIAP2->setEnabled(bEnable);
        ui->tabHK->setEnabled(bEnable);
        ui->tabGATT->setEnabled(bEnable);
        ui->tabBSG->setEnabled(bEnable);
        ui->tabPBC->setEnabled(bEnable);
        ui->tabGATT_DB->setEnabled(bEnable);
        ui->tabBATTC->setEnabled(bEnable);
        ui->tabFINDMEL->setEnabled(bEnable);
        ui->tabOPS->setEnabled(bEnable);
        ui->tabAlertNotfn->setEnabled(bEnable);
        ui->tabOTPClient->setEnabled(bEnable);
        ui->tabAVSink->setEnabled(bEnable);
        ui->tabLECOC->setEnabled(bEnable);
        ui->tabLedDemo->setEnabled(bEnable);
        ui->tabDemo->setEnabled(bEnable);
        ui->tabMAPClient->setEnabled(bEnable);
    }
}

/************** Local device management *************/

// read info of paired devices saved on disk
void MainWindow::ReadDevicesFromSettings(const char *group, QComboBox *cbDevices, QPushButton *btnUnbond)
{
    // get paired devices
    m_settings.beginGroup(group);
    QStringList  device_keys = m_settings.allKeys();

    int nvram_id=-1;
    QString dev_name_id, dev_name="";
    int bda0,bda1,bda2,bda3,bda4,bda5;
    for (int i = 0; i < device_keys.size(); i++)
    {
        dev_name_id = m_settings.value(device_keys[i],"").toString();
        if (dev_name_id.size() >= 3)
        {
            sscanf(dev_name_id.toStdString().c_str(),"%02X:", &nvram_id);
            if (dev_name_id.size() > 4)
                dev_name = dev_name_id.right(dev_name_id.size()-3);
        }

        sscanf(device_keys[i].toStdString().c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
               &bda0, &bda1, &bda2, &bda3, &bda4, &bda5);
        BYTE bda[6];
        bda[0] = static_cast<BYTE>(bda0);
        bda[1] = static_cast<BYTE>(bda1);
        bda[2] = static_cast<BYTE>(bda2);
        bda[3] = static_cast<BYTE>(bda3);
        bda[4] = static_cast<BYTE>(bda4);
        bda[5] = static_cast<BYTE>(bda5);

        CBtDevice * pDev = AddDeviceToList(bda,cbDevices, const_cast<char *>(dev_name.toStdString().c_str()),true);
        if (pDev && nvram_id != -1)
        {
            pDev->m_nvram_id = nvram_id;
        }
        if(pDev && (strcmp(group, "devicesLE") == 0))
        {
            pDev->m_bIsLEDevice = true;
        }
    }
    m_settings.endGroup();

    // get link key for paired devices
    m_settings.beginGroup("NVRAM");
    for (int i = 0; i < cbDevices->count(); i++)
    {
        QVariant var;
        var = cbDevices->itemData(i);
        CBtDevice * pDev = var.value<CBtDevice *>();
        if (nullptr == pDev)
            continue;
        if (pDev->m_nvram_id == -1)
            continue;
        QString nvram_id_key;
        nvram_id_key.sprintf("%02X",pDev->m_nvram_id);
        pDev->m_nvram = m_settings.value(nvram_id_key).toByteArray();
    }
    m_settings.endGroup();

    btnUnbond->setEnabled(false);
    if (cbDevices->count())
    {
        cbDevices->setCurrentIndex(0);
        btnUnbond->setEnabled(true);
    }
}

// Set local device to be discoverable
void MainWindow::onDiscoverable(bool)
{
    m_settings.setValue("discoverable",ui->btnDiscoverable->isChecked());
    setVis();
}

// Set local device to be connectable
void MainWindow::onConnectable(bool)
{
    m_settings.setValue("Connectable",ui->btnConnectable->isChecked());
    setVis();
}

// Set local device to be pair-able
void MainWindow::onPairable(bool)
{
    m_settings.setValue("pairing_mode",ui->btnPairable->isChecked());
    setPairingMode();
}

// set local device name
void MainWindow::setDevName(char *device_name)
{
    Log("setDevName");

    if(!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;
    Log("setDevName: %s", device_name);
    app_host_dm_set_device_name(device_name);
}

// set local device BD address
void MainWindow::setDevBda(BYTE* bda)
{
    Log("setDevBda");

    if (!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    Log("setDevBda: addr = %02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    app_host_dm_set_device_addr(bda);
}

// Set local device pairable or non pairable
void MainWindow::setPairingMode()
{
    if(!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    Log("Set Pairable:%d",ui->btnPairable->isChecked());
    app_host_dm_set_pairing_mode(ui->btnPairable->isChecked());
}

// set local device visibility
void MainWindow::setVis()
{
    if(!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    Log("Set discoverable:%d, connectable:%d",ui->btnDiscoverable->isChecked(),ui->btnConnectable->isChecked());
    app_host_dm_set_vis(ui->btnDiscoverable->isChecked(),ui->btnConnectable->isChecked());
}

// Reset local device
void MainWindow::onReset()
{
    if (m_CommPort == nullptr)
        return;

    if (!m_bPortOpen)
    {
         return;
    }

    if (QMessageBox::Yes != QMessageBox(QMessageBox::Information, "Reset", "Are you sure you want to reset? After resetting the device, you may need to download the embedded application again.", QMessageBox::Yes|QMessageBox::No).exec())
    {
        return;
    }

    // send command to reset
    app_host_dm_reset();

    // close comm port reader thread and disable UI
    QThread::msleep(10);

    ClearPort();

}

/************** Peer device management *************/

// get selected BR/EDR device
CBtDevice * MainWindow::GetSelectedDevice()
{
    int i = ui->cbDeviceList->currentIndex();
    if ( i == -1)
        return nullptr;

    QVariant var;
    var = ui->cbDeviceList->itemData(i);
    return var.value<CBtDevice *>();
}

// get selected BLE device
CBtDevice * MainWindow::GetSelectedLEDevice()
{
    int i = ui->cbBLEDeviceList->currentIndex();
    if ( i == -1)
        return nullptr;

    QVariant var;
    var = ui->cbBLEDeviceList->itemData(i);
    return var.value<CBtDevice *>();
}

// unpair a BR-EDR device
void MainWindow::OnBnClickedBREDRUnbond()
{
    onUnbond(ui->cbDeviceList);
}

// unpair a BLE device
void MainWindow::OnBnClickedLeUnbond()
{
    onUnbond(ui->cbBLEDeviceList);
}

// unpair the device
void MainWindow::onUnbond(QComboBox* cb)
{
    if (m_CommPort == nullptr)
    {
        Log("Serial port is not open");
        return;
    }

    if (!m_bPortOpen)
    {
        Log("Serial port is not open");
        return;
    }

    int i = cb->currentIndex();
    if ( i == -1)
        return;

    QVariant var;
    var = cb->itemData(i);

    CBtDevice * pDev = var.value<CBtDevice *>();
    if (pDev == nullptr)
        return;

    m_settings.beginGroup((cb != ui->cbBLEDeviceList) ? "devices" : "devicesLE");
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
        pDev->m_address[0], pDev->m_address[1], pDev->m_address[2], pDev->m_address[3], pDev->m_address[4], pDev->m_address[5]);
    m_settings.remove(strBda);
    m_settings.endGroup();

    m_settings.beginGroup("NVRAM");
    QString nvram_id_key;
    nvram_id_key.sprintf("%02X",pDev->m_nvram_id);
    m_settings.remove(nvram_id_key);
    m_settings.endGroup();

    HidHostDeviceRemove(pDev->m_address);
    BattCDeviceRemove(pDev->m_address);
    FindMeLocatorDeviceRemove(pDev->m_address);

    m_settings.sync();

    app_host_dm_delete_nvram_data(pDev->m_nvram_id);

    pDev->m_paired = false;
    pDev->m_nvram_id = -1;
    pDev->m_nvram.clear();

    if(cb == ui->cbBLEDeviceList)
        ui->btnBLEUnbond->setEnabled(false);
    else
        ui->btnUnbond->setEnabled(false);

    cb->setItemIcon(cb->currentIndex(), *new QIcon);
}


// User changed selected BR/EDR device
void MainWindow::onDevChange(QString)
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev)
    {
         ui->btnUnbond->setEnabled(pDev->m_paired);

        // switch to BR/EDR
        setRadioHIDH_BLE(false);
    }
}

// User changed selected BLE device
void MainWindow::onLEDevChange(QString)
{
    CBtDevice * pDev = GetSelectedLEDevice();
    if (pDev)
    {
        ui->btnBLEUnbond->setEnabled(pDev->m_paired);
        UpdateGattButtons(pDev);

        // switch to BLE
        setRadioHIDH_BLE(true);
    }
}

// BR/EDR discovery start
void MainWindow::onStartDisc()
{
    if (!m_inquiry_active)
    {
        m_inquiry_active = TRUE;

        // Clear all items from combo box except paired devices
        ResetDeviceList(ui->cbDeviceList);

        if (!app_host_dm_inquiry(true))
        {
            Log("Error starting inquiry");
            m_inquiry_active = FALSE;
        }
    }

    ui->btnStartDisc->setEnabled(!m_inquiry_active);
    ui->btnStopDisc->setEnabled(m_inquiry_active);

    // switch to BR/EDR
    setRadioHIDH_BLE(false);
}

// BR/EDR discovery stop
void MainWindow::onStopDisc()
{
    m_inquiry_active = FALSE;

    if (!app_host_dm_inquiry(false))
    {
        Log("Error stopping inquiry");
    }

    ui->btnStartDisc->setEnabled(!m_inquiry_active);
    ui->btnStopDisc->setEnabled(m_inquiry_active);
}

// LE scan start
void MainWindow::OnBnClickedDiscoverDevicesStart()
{
    if (!m_scan_active)
    {
        m_scan_active = true;

        // Clear all items from combo box except paired devices
        ResetDeviceList(ui->cbBLEDeviceList);

        // scan command, true = enable, false = disable
        app_host_dm_le_scan(true);
    }

    setGATTUI();

    // switch to BLE
    setRadioHIDH_BLE(true);
}

// get version info
void MainWindow::OnBnClickedVersionInfo()
{
    GetVersion();
}

// LE scan stop
void MainWindow::OnBnClickedDiscoverDevicesStop()
{
    if (m_scan_active)
    {
        m_scan_active = false;
        // scan command, len 1, enable = false
        app_host_dm_le_scan(false);
    }

    setGATTUI();
}

// Find a paired device in list of devices using BDA
CBtDevice *MainWindow::FindInList(BYTE * addr, QComboBox * pCb)
{
    int j;
    CBtDevice *device = nullptr;
    for (int i = 0; i < pCb->count(); i++)
    {
        device = static_cast<CBtDevice *>(pCb->itemData(i).value<CBtDevice *>());
        if (device == nullptr)
            continue;
        for (j = 0; j < 6; j++)
        {
            if (device->m_address[j] != addr[j])
                break;
        }
        if (j == 6)
        {
            return device;
        }
    }
    return nullptr;
}

// Find a paired device in list of devices using connection type
CBtDevice *MainWindow::FindInList(UINT16 conn_type, UINT16 handle, QComboBox * pCb)
{
    CBtDevice *device = nullptr;
    for (int i = 0; i < pCb->count(); i++)
    {
        device = static_cast<CBtDevice *>(pCb->itemData(i).value<CBtDevice *>());
        if (device == nullptr)
            continue;

        if(device->m_conn_type & conn_type)
         {
            if(device->m_conn_type & WICED_CONNECTION_TYPE_SPP)
            {
                if(handle == device->m_spp_handle)
                    return device;
            }
            else
                return device;
         }
    }
    return nullptr;
}

// Set a device as currently selected in the combo box
void MainWindow::SelectDevice(QComboBox* cb, BYTE * bda)
{
    int j;
    CBtDevice *device = nullptr;
    for (int i = 0; i < cb->count(); i++)
    {
        device = static_cast<CBtDevice *>(cb->itemData(i).value<CBtDevice *>());
        if (device == nullptr)
            continue;
        for (j = 0; j < 6; j++)
        {
            if (device->m_address[j] != bda[j])
                break;
        }
        if (j == 6)
        {
            cb->setCurrentIndex(i);
            break;
        }
    }
}

// Set device status to paired
void MainWindow::SetDevicePaired(BYTE * info, int len)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   info[0], info[1], info[2], info[3], info[4], info[5]);
    bool bLeDevice = false;
    int i;

    if (len>=7 && (info[6]==RPC_BT_DEVICE_TYPE_BT_DEVICE_TYPE_BLE || info[6]==RPC_BT_DEVICE_TYPE_BT_DEVICE_TYPE_BREDR))
    {
        // We have the BT type
        m_host_type = info[6];
        bLeDevice = m_host_type == RPC_BT_DEVICE_TYPE_BT_DEVICE_TYPE_BLE;
        QComboBox * cboxPtr = bLeDevice ? ui->cbBLEDeviceList : ui->cbDeviceList;
        i =  cboxPtr->findText(strBda,Qt::MatchStartsWith);

        // we have paired with a unknown device; add it to the paired list
        if (i == -1)
        {
            Log("Paired with new %s device %02x:%02x:%02x:%02x:%02x:%02x", bLeDevice?"LE":"BR/EDR", info[0], info[1], info[2], info[3], info[4], info[5]);
            AddDeviceToList(info, cboxPtr, nullptr, true);
            if (bLeDevice)
                ui->btnBLEUnbond->setEnabled(true);
            else
                ui->btnUnbond->setEnabled(true);
            return;
        }
    }
    else
    {
        m_host_type = 0; // We don't know host type
        i = ui->cbDeviceList->findText(strBda,Qt::MatchStartsWith);
        if (i == -1)
        {
            i = ui->cbBLEDeviceList->findText(strBda,Qt::MatchStartsWith);
            if(i >= 0)
                bLeDevice = true;
        }
        if (i == -1)
            return;
    }
    CBtDevice * device = nullptr;
    if(!bLeDevice)
        device = static_cast<CBtDevice *>(ui->cbDeviceList->itemData(i).value<CBtDevice *>());
    else
        device = static_cast<CBtDevice *>(ui->cbBLEDeviceList->itemData(i).value<CBtDevice *>());

    if (device == nullptr)
        return;

    if (device->m_paired)
        return;

    device->m_paired = true;

    if(!bLeDevice)
        ui->cbDeviceList->setItemIcon(i,m_paired_icon);
    else
        ui->cbBLEDeviceList->setItemIcon(i,m_paired_icon);

    m_settings.beginGroup(bLeDevice? "devicesLE" : "devices");
    m_settings.setValue(strBda,device->m_name);
    m_settings.endGroup();
    m_settings.sync();
    if(!bLeDevice)
        ui->btnUnbond->setEnabled( (i == ui->cbDeviceList->currentIndex()));
    else
        ui->btnUnbond->setEnabled( (i == ui->cbBLEDeviceList->currentIndex()));
}

// HID Host: VirtualUnplug a device (remove a device)
void MainWindow::VirtualUnplug(CBtDevice *pDev)
{
    QString nvram_id_key;
    m_settings.sync();

    m_settings.beginGroup(NVRAM_GROUP);
    nvram_id_key.sprintf("%02X",pDev->m_nvram_id);
    m_settings.remove(nvram_id_key);
    m_settings.endGroup();

    app_host_dm_delete_nvram_data(pDev->m_nvram_id);

    pDev->m_paired = false;
    pDev->m_nvram_id = -1;
    pDev->m_nvram.clear();

    HidHostDeviceRemove(pDev->m_address);
    BattCDeviceRemove(pDev->m_address);
    FindMeLocatorDeviceRemove(pDev->m_address);
}

// HID Host: Add an HID Device in Database
void MainWindow::HidHostDeviceAdd(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    Log("Adding HID Device in HIDH database");

    m_settings.sync();
    m_settings.beginGroup(HID_HOST_GROUP);
    m_settings.setValue(strBda, 1);
    m_settings.endGroup();
    m_settings.sync();
}

// HID Host: Remove an HID Device in Database
void MainWindow::HidHostDeviceRemove(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    Log("Removing HID Device from HIDH database");

    m_settings.sync();
    m_settings.beginGroup(HID_HOST_GROUP);
    m_settings.remove(strBda);
    m_settings.endGroup();
    m_settings.sync();
}

// Battery Client: Add a BATTC Device from Database
void MainWindow::BattCDeviceAdd(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    m_settings.sync();
    m_settings.beginGroup(HID_BATTC_GROUP);
    m_settings.setValue(strBda, 1);
    m_settings.endGroup();
    m_settings.sync();
}

// Battery Client: Remove a BATTC Device from Database
void MainWindow::BattCDeviceRemove(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    m_settings.sync();
    m_settings.beginGroup(HID_BATTC_GROUP);
    m_settings.remove(strBda);
    m_settings.endGroup();
    m_settings.sync();
}

// Battery Client: Add an FINDMEL Device from Database
void MainWindow::FindMeLocatorDeviceAdd(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    m_settings.sync();
    m_settings.beginGroup(HID_FINDMEL_GROUP);
    m_settings.setValue(strBda, 1);
    m_settings.endGroup();
    m_settings.sync();
}

// Battery Client: Remove an FINDMEL Device from Database
void MainWindow::FindMeLocatorDeviceRemove(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    m_settings.sync();
    m_settings.beginGroup(HID_FINDMEL_GROUP);
    m_settings.remove(strBda);
    m_settings.endGroup();
    m_settings.sync();
}

// Add new device to combo box
CBtDevice *MainWindow::AddDeviceToList(BYTE *addr, QComboBox * pCb, char * bd_name, bool paired)
{
    CBtDevice *device = nullptr;
    QString abuffer;

    if (bd_name)
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x (%s)",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],bd_name);
    else
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Check if device is already present
    int i = pCb->findText( abuffer,Qt::MatchStartsWith);
    if (i == -1)
    {
        QVariant qv;

        device = new CBtDevice(paired);
        if (bd_name && strlen(bd_name))
            strncpy(device->m_name, bd_name,sizeof(device->m_name)-1);

        qv.setValue<CBtDevice *>(device);

        if (paired)
        {
            pCb->addItem( m_paired_icon, abuffer, qv);
        }
        else
        {
            pCb->addItem( abuffer, qv );
        }
        i = pCb->findText( abuffer, Qt::MatchStartsWith);

        memcpy(device->m_address, addr, 6);
    }
    else
    {
        // device already in list

        // If paired, set icon
        if(paired)
            pCb->setItemIcon(i,m_paired_icon);


        device = static_cast<CBtDevice *>(pCb->itemData(i).value<CBtDevice*>());
    }

    if (paired)
    {
        // BR/EDR?
        if (pCb == ui->cbDeviceList)
        {
            ui->btnUnbond->setEnabled( (i == ui->cbDeviceList->currentIndex()));
        }
        else // must be BLE
        {
            ui->btnBLEUnbond->setEnabled( (i == ui->cbDeviceList->currentIndex()));
        }
    }

    return device;
}

// Push NVRAM data to embedded device
void MainWindow::WriteNVRAMToDevice(bool bBLEDevice)
{
    QVariant var;
    CBtDevice * pDev = nullptr;
    int len=0;

    QComboBox *cb = ui->cbDeviceList;
    if(bBLEDevice)
        cb = ui->cbBLEDeviceList;

    for (int i = 0; i < cb->count(); i++)
    {
        var = cb->itemData(i);
        if (nullptr == (pDev = var.value<CBtDevice *>()))
            continue;
        if (pDev->m_nvram_id == -1 || pDev->m_nvram.size() == 0)
            continue;

        len = pDev->m_nvram.size();
        if (len <= 998)
        {
            if (m_b_is_hidd && (pDev->m_nvram_id == 0x10))
            {
                char * nvdata = const_cast<char *>(pDev->m_nvram.constData());
                app_host_dm_push_pairing_host_info(reinterpret_cast<unsigned char*>(nvdata), len);
            }
            else
            {
                char * nvdata = const_cast<char *>(pDev->m_nvram.constData());
                app_host_dm_push_nvram_data(pDev->m_nvram_id, reinterpret_cast<unsigned char*>(nvdata), len);
            }
        }
        else
        {
            Log("Err: NVRAM chunk too big %d", len);
        }
    }
}

// Send HID Host Add command for every HID Device
void MainWindow::SendHidHostAdd(void)
{
    QStringList hid_device_list;
    int bda[BD_ADDR_LEN];
    char      trace[1024];

    m_settings.sync();
    m_settings.beginGroup(HID_HOST_GROUP);

    // Get the list of HID Devices in the HID Host group
    hid_device_list = m_settings.allKeys();

    // For every HID Device
    for (int i = 0; i < hid_device_list.size(); i++)
    {
        sscanf(hid_device_list[i].toStdString().c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
               &bda[0], &bda[1], &bda[2], &bda[3], &bda[4], &bda[5]);
        app_host_dm_hidh_add(bda);

        sprintf(trace, "HIDH Add Device address %02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        Log(trace);
    }

    m_settings.endGroup();
}

// Send Battery Client Add command for every Battery Service Device
void MainWindow::SendBattCAdd(void)
{
    QStringList battc_device_list;
    int bda[BD_ADDR_LEN];

    m_settings.sync();
    m_settings.beginGroup(HID_BATTC_GROUP);

    // Get the list of BATTC Devices in the Battery-Client Host group
    battc_device_list = m_settings.allKeys();

    // For every HID Device
    for (int i = 0; i < battc_device_list.size(); i++)
    {
        sscanf(battc_device_list[i].toStdString().c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
               &bda[0], &bda[1], &bda[2], &bda[3], &bda[4], &bda[5]);
        app_host_dm_add_battery_client(bda);
    }

    m_settings.endGroup();
}

// Send FindMe Locator Add command for every FindMe Target Device
void MainWindow::SendFindMeLocatorAdd(void)
{
    QStringList findmel_device_list;
    int bda[BD_ADDR_LEN];

    m_settings.sync();
    m_settings.beginGroup(HID_FINDMEL_GROUP);

    // Get the list of FINDMEL Devices in the FindMe Locator group
    findmel_device_list = m_settings.allKeys();

    // For every FindMe Target Device
    for (int i = 0; i < findmel_device_list.size(); i++)
    {
        sscanf(findmel_device_list[i].toStdString().c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
               &bda[0], &bda[1], &bda[2], &bda[3], &bda[4], &bda[5]);
        app_host_dm_add_findme_locator(bda);
    }

    m_settings.endGroup();
}

// Clear all items from combo box except paired devices
void MainWindow::ResetDeviceList(QComboBox *cb)
{
    CBtDevice *device = nullptr;
    QComboBox temp;
    int i = 0;
    // Save the paired devices to a temp combo box
    for(i = 0; i < cb->count(); i++)
    {
        device = static_cast<CBtDevice *>(cb->itemData(i).value<CBtDevice *>());
        if(device && device->m_paired)
        {
            QVariant qv;
            qv.setValue<CBtDevice *>(device);

            temp.addItem(cb->itemText(i), qv);
        }
    }

    // clear the device combo box
    cb->clear();

    // add the paired devices back to device combo box
    for(i = 0; i < temp.count(); i++)
    {
        device = static_cast<CBtDevice *>(temp.itemData(i).value<CBtDevice *>());
        if(device)
        {
            QVariant qv;
            qv.setValue<CBtDevice *>(device);

            cb->addItem(m_paired_icon, temp.itemText(i), qv);
        }
    }
}


/*************************WICED HCI handling*********************************/

// Send WICED HCI commmand to embedded device
bool MainWindow::SendWicedCommand(unsigned short command, unsigned char * payload, unsigned int len)
{
    return wiced_hci_send_command(command, payload, len);
}

// Send WICED HCI commmand to embedded device
int MainWindow::PortWrite(unsigned char * data, DWORD Length)
{
    DWORD dwWritten = 0;
    DWORD dwTotalWritten = 0;
    unsigned char * p = &data[0];

    if (nullptr == m_CommPort)
        return -1;
    if (!m_CommPort->isOpen())
        return -1;

    // Since the write to serial port can happen from main thread as well as
    // audio wav thread, serialize the write of WICED HCI packets
    m_write.lock();

    // Loop and write data to serial port till all is written
    while (Length && m_bPortOpen)
    {
        dwWritten = 0;
        char * data_in = reinterpret_cast<char *>(p);
        dwWritten = static_cast<unsigned long>(m_CommPort->write(data_in,static_cast<qint64>(Length)));

        m_CommPort->flush();

        if( static_cast<long>(dwWritten) == -1)
        {
            Log("error - port write error");
            break;
        }

        if(Length != static_cast<unsigned long>(dwWritten))
        {
            if(!m_bPortOpen)
                break;

            Log("port write mismatch, to write %ld, written %ld, wait and try", Length, dwWritten);

            if (!m_CommPort->waitForBytesWritten(100))
            {
                Log("error - waitForBytesWritten");
                break;
            }

            if(!m_bPortOpen)
                break;
        }
        if ( static_cast<unsigned long>(dwWritten) > Length)
            break;

        Length -=  static_cast<unsigned long>(dwWritten);

        if(Length)
            p += dwWritten;

        dwTotalWritten += dwWritten;
    }

    m_write.unlock();


    return static_cast<int>(dwTotalWritten);
}

// Command to get version and supported features of embedded app
void MainWindow::GetVersion()
{
    // send command to get version and feature info
    app_host_dm_get_version_info();
}

// Command to enable WICED HCI traces
void MainWindow::EnableAppTraces()
{
    int route = ui->cbBLEHIDDebug->currentIndex();
    if (route)
        app_host_dm_set_app_traces(true, route);  // enable traces
    else
        DisableAppTraces();
}

// Command to disable WICED HCI traces
void MainWindow::DisableAppTraces()
{
    app_host_dm_set_app_traces(false, 0);  // disable traces
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventDm(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if(!m_bUIEnabled)
    {
        EnableUI(true);
        EnableAppTraces();
        Log("Client Control app established communication with Bluetooth device.");
    }

    switch (HCI_CONTROL_GROUP(opcode))
    {
        case HCI_CONTROL_GROUP_DEVICE:
            HandleDeviceEventsDm(opcode, p_data, len);
            break;
        case HCI_CONTROL_GROUP_MISC:
            HandleDeviceEventsMisc(opcode, p_data, len);
            break;

    }
}

// Handle WICED HCI events for device management
void MainWindow::HandleDeviceEventsDm(DWORD opcode, LPBYTE p_data, DWORD len)
{
    BYTE    bda[6];
    char trace[1024];
    char num[5];
    QVariant var;
    CBtDevice * pDev;
    int dev_idx;

    switch (opcode)
    {
    // received embedded app traces
    case HCI_CONTROL_EVENT_WICED_TRACE:
        if (len >= 2)
        {
            if ((len > 2) && (p_data[len - 2] == '\n'))
            {
                p_data[len - 2] = 0;
                len--;
            }
            // If there are two or more devices connected to the computer, we
            // need a way to distingusih trace sent to BTSpy from each device.
            // When printing embedded app traces to BTSpy, prepend the index of the
            // serial port in the drop down combo box to the trace.
            sprintf(num, "%d", ui->cbCommport->currentIndex());
            trace[0] =  num[0];
            trace[1] = ' ';
            memcpy(&trace[2], p_data, len);
            TraceHciPkt(0, reinterpret_cast<unsigned char *>(trace),
                        static_cast<USHORT>(len+2),
                        static_cast<USHORT>(ui->cbCommport->currentIndex()),
                        iSpyInstance);
        }

        break;

        // received HCI traces
    case HCI_CONTROL_EVENT_HCI_TRACE:
        // If there are two or more devices connected to the computer, we
        // need a way to distingusih trace sent to BTSpy from each device.
        // When printing HCI protocol traces to BTSpy, the index of the
        // serial port in the drop down combo box will be displayed in the
        // HCI trace.
        if (p_data)
            TraceHciPkt(p_data[0] + 1, &p_data[1], static_cast<USHORT>(len - 1),
                    static_cast<USHORT>(ui->cbCommport->currentIndex()),
                    iSpyInstance);
        break;

        // received paired device data for all to save in NVRAM
    case HCI_CONTROL_EVENT_NVRAM_DATA:
    {
        QString nvram_id_key;
        nvram_id_key.sprintf("%02X", p_data[0] + (p_data[1] << 8));
        m_settings.beginGroup("NVRAM");
        // Use QByteArray constructor with length parameter otherwise a 0x00 is considered as '\0'
        char * pc = reinterpret_cast<char *>(&p_data[2]);
        QByteArray val (const_cast<const char *>(pc), static_cast<int>(len - 2));
        m_settings.setValue(nvram_id_key,val);
        m_settings.endGroup();
        m_settings.sync();

        Log("HCI_CONTROL_EVENT_NVRAM_DATA %s", nvram_id_key.toLocal8Bit().data());

        // if it is a link key add this device to the appropriate combobox
        if (len - 2)
        {
            // application should probably send type of the device, if host does not remember.
            // for now we will use the fact that BR/EDR keys are just 16 bytes.
            BOOL bLeDevice = (p_data[25] | p_data[26] | p_data[27]) != 0;

            CBtDevice * pDev = AddDeviceToList(&p_data[2], bLeDevice ? ui->cbBLEDeviceList : ui->cbDeviceList, nullptr, true);
            pDev->m_nvram_id =  p_data[0] + (p_data[1] << 8);
            pDev->m_nvram = QByteArray (const_cast<const char *>(reinterpret_cast<char *>(&p_data[2])), static_cast<int>(len - 2));
            if(bLeDevice)
                pDev->m_bIsLEDevice = true;
            pDev->m_paired = true;

            // save the paired device data to disk
            if(!bLeDevice)
                m_settings.beginGroup("devices");
            else
                m_settings.beginGroup("devicesLE");

            QString dev_key;
            dev_key.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
                pDev->m_address[0], pDev->m_address[1], pDev->m_address[2],
                pDev->m_address[3], pDev->m_address[4], pDev->m_address[5] );
            QString name_id;
            name_id.sprintf("%02X:%s", pDev->m_nvram_id, pDev->m_name);
            m_settings.setValue(dev_key,name_id);
            m_settings.endGroup();
            m_settings.sync();

            // Set Unbond button state for selected device
            if(!bLeDevice)
                onDevChange(dev_key);
            else
                onLEDevChange(dev_key);

            Log("%s device %s, ID %s", bLeDevice ? "BLE":"BR/EDR", dev_key.toLocal8Bit().data(), name_id.toLocal8Bit().data());
        }
    }
    break;

        // received event for user to confirm pairing
    case HCI_CONTROL_EVENT_USER_CONFIRMATION:
    {
        Log("Numeric Comparison %02x:%02x:%02x:%02x:%02x:%02x Value:%d", p_data[0], p_data[1], p_data[2], p_data[3], p_data[4], p_data[5], p_data[6] + (p_data[7] << 8) + (p_data[8] << 16) + (p_data[9] << 24));
        QString str;
        str.sprintf("Confirm pairing code: %d", p_data[6] + (p_data[7] << 8) + (p_data[8] << 16) + (p_data[9] << 24));
        if (QMessageBox::Yes != QMessageBox(QMessageBox::Information, "User Confirmation", str, QMessageBox::Yes|QMessageBox::No).exec())
        {
            break;
        }

        // send command to confirm user confirmation value
        app_host_dm_user_confirm(p_data, 1);  // 1 - accept, 0 - do not accept
    }
    break;

        // received event that local device started
    case HCI_CONTROL_EVENT_DEVICE_STARTED:
        Log("Device Started");
        m_scan_active = false;
        m_inquiry_active = false;
        ui->btnStartDisc->setEnabled(!m_inquiry_active);
        ui->btnStopDisc->setEnabled(m_inquiry_active);

        // Mark every BE/EDR device as disconnected
        for (dev_idx = 0 ; dev_idx < ui->cbDeviceList->count() ; dev_idx++)
        {
            var = ui->cbDeviceList->itemData(dev_idx);
            pDev = var.value<CBtDevice *>();
            if(pDev != nullptr)
            {
                pDev->m_conn_type = CONNECTION_TYPE_NONE;
                pDev->m_ag_handle = NULL_HANDLE;
                pDev->m_audio_handle = NULL_HANDLE;
                pDev->m_avk_handle = NULL_HANDLE;
                pDev->m_avrc_handle = NULL_HANDLE;
                pDev->m_hf_handle = NULL_HANDLE;
                pDev->m_hidh_handle = NULL_HANDLE;
                pDev->m_iap2_handle = NULL_HANDLE;
            }
        }

        // Mark every BLE device as disconnected
        for (dev_idx = 0 ; dev_idx < ui->cbBLEDeviceList->count() ; dev_idx++)
        {
            var = ui->cbDeviceList->itemData(dev_idx);
            pDev = var.value<CBtDevice *>();
            if(pDev != nullptr)
            {
                pDev->m_conn_type = CONNECTION_TYPE_NONE;
                pDev->m_bsg_handle = NULL_HANDLE;
                pDev->m_hidh_handle = NULL_HANDLE;
                pDev->m_iap2_handle = NULL_HANDLE;
                pDev->m_battc_handle = NULL_HANDLE;
                pDev->m_findmel_handle = NULL_HANDLE;
            }
        }

        Startup();
        // Stop the timer to prevent duplicate startup process
        m_dmStartupTimer->stop();
        break;

        // received event that pairing completed
    case HCI_CONTROL_EVENT_PAIRING_COMPLETE:
    {
        bool success = p_data[0] == 0;
        Log("Pairing status: %s, code: %d", success ? "Success" : "Fail", static_cast<int>(p_data[0]));

        if (success && (len >= 7))
        {
            uint8_t bda[6];
            for (int i=0;i<6;i++) bda[i] = p_data[6-i];

            // mark paired only if it is success
            SetDevicePaired(bda, static_cast<int>(--len));
            setHIDD_HostAddr(bda);
            setHIDD_linkChange(bda, TRUE);
        }
        break;
    }

        // received BR/EDR inquiry result
    case HCI_CONTROL_EVENT_INQUIRY_RESULT:
    {
        Log("Device found: %02x:%02x:%02x:%02x:%02x:%02x device_class %02x:%02x:%02x rssi:%d",
            static_cast<int>(p_data[5]),static_cast<int>(p_data[4]),static_cast<int>(p_data[3]),
            static_cast<int>(p_data[2]),static_cast<int>(p_data[1]),static_cast<int>(p_data[0]),
            static_cast<int>(p_data[8]),static_cast<int>(p_data[7]),static_cast<int>(p_data[6]),
            static_cast<int>(p_data[9] - 256));

        char szName[50] = {0};

#if 0
        if (ui->cbCommport->itemData(ui->cbCommport->currentIndex()).toString().compare("0") == 0)
        {
            if (len > 11)
            {
                DecodeEIR(&p_data[11], len - 11, szName,sizeof(szName));
            }
        }
        else
#endif
        {
            if (len > 10)
            {
                DecodeEIR(&p_data[10], len - 10, szName,sizeof(szName));
            }
        }

        // dump advertisement data to the trace
        /*
        if (len > 10)
        {
            trace[0] = 0;
            for (int i = 0; i < (int)len - 10; i++)
                sprintf(&trace[strlen(trace)], "%02x", p_data[10 + i]);
            Log(trace);

        }
        */
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];
        // check if this device is not present yet.
        if (FindInList(bda,ui->cbDeviceList) != nullptr)
            break;
        AddDeviceToList(bda,ui->cbDeviceList, szName);
    }
        break;

        // received event that BR/EDR inquiry is complete
    case HCI_CONTROL_EVENT_INQUIRY_COMPLETE:
        if (m_inquiry_active)
        {
            m_inquiry_active = FALSE;
            Log("Inquiry Complete");
        }
        ui->btnStartDisc->setEnabled(!m_inquiry_active);
        ui->btnStopDisc->setEnabled(m_inquiry_active);

        break;
    }
}

// handle misc event
void MainWindow::HandleDeviceEventsMisc(DWORD opcode, LPBYTE tx_buf, DWORD len)
{
    unsigned int index = 0;

    switch (opcode)
    {
    // recieved version info event
    case HCI_CONTROL_MISC_EVENT_VERSION:
        m_major = tx_buf[index++];
        m_minor = tx_buf[index++];
        m_rev =  tx_buf[index++];
        unsigned int data1 = tx_buf[index++];
        unsigned int data2 = tx_buf[index++];
        m_build = data1 | (data2 << 8);
        data1 = tx_buf[index++];
        data2 = tx_buf[index++];
        unsigned int data3 = tx_buf[index++];
        m_chip = data1 | (data2 << 8) | (data3 << 24);
        // power class obsolete, just incremet past it
        // m_power = tx_buf[index++];
        index++;
        Log("Version Info:");
        Log("Version %d.%d, rev %d, build %d",
            m_major, m_minor, m_rev, m_build);
        Log("Chip: %d", m_chip);

        if(len < 10) // old API len is 9
            break;

        Log("Tabs supported by embedded application - ");
        // Embedded device sent us supported features.
        // First disable all tabs, then enable supported
        // tabs one at a time.
        EnableTabs(0xFF, false);
        for(; index < len; index++)
        {
            EnableTabs(tx_buf[index], true);
        }

        if(m_chip == 20835)
            DisableBluetoothClassic();

        break;
    }
}

/**************Firmware download *************/
// Find firmware .hcd file to download
void MainWindow::onFindPatchFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Firmware Download File"), "", tr("OTA BIN Files (*.ota.bin *.ota.bin.signed)"));
    ui->edPatchFile->setText(fileName);
}

// Start download of firmware file
void MainWindow::onDownload()
{
    if (ui->edPatchFile->text().length() == 0)
    {
        Log("Specify valid firmware file please");
        return;
    }

    m_settings.setValue("FirmwareFile", ui->edPatchFile->text());

    // download firmware
    if (!FirmwareDownloadStart(ui->edPatchFile->text()))
        return;

    dl_msgbox.setText("Downloading new firmware.");
    dl_msgbox.setWindowTitle("Firmware Download");
    dl_msgbox.setStandardButtons(QMessageBox::Cancel);
    dl_msgbox.show();
}


// called by download thread to report progress
void MainWindow::onDlProgress(QString * msg, int pktcnt, int bytecnt)
{
    if (msg)
    {
        Log(msg->toStdString().c_str());
        delete msg;
    }

    if (pktcnt && bytecnt)
    {
        QString trace;

        trace.sprintf("Download status, record %d, total bytes: %d",pktcnt,bytecnt);
        Log(trace.toStdString().c_str());
        dl_msgbox.setText(trace);
        dl_msgbox.update();
    }

    // update screen
    update();
    qApp->processEvents();
}

// called by download thread when finished
void MainWindow::onDlDone(const QString &s)
{
    Log(s.toStdString().c_str());
    dl_msgbox.close();  // close download message box
}

// user pressed cancel button for download
void MainWindow::onMsgBoxButton(QAbstractButton*btn)
{
    UNUSED(btn);
    FirmwareDownloadStop();
}

/************** Serial port management *************/

// User clicked button to open or close serial port
void MainWindow::on_btnOpenPort_clicked()
{
    // If port is not open, open it
    if(!m_bPortOpen)
    {
        EnableUI(false);
        ui->btnOpenPort->setEnabled(false);
        bool bopen = SetupCommPort();
        ui->btnOpenPort->setEnabled(true);

        if(!bopen)
        {
            QMessageBox(QMessageBox::Information, "Serial Port", "Error opening serial port", QMessageBox::Ok).exec();
        }
        else
        {
            ui->cbCommport->setEnabled(false);
            ui->btnOpenPort->setText("Close Port");
            ui->cbBaudRate->setEnabled(false);
            ui->btnFlowCntrl->setEnabled(false);
        }
    }
    // Close port if open
    else
    {
#ifdef PCM_ALSA
        if (m_alsa_handle != nullptr)
        {
            snd_pcm_close(m_alsa_handle);
            m_alsa_handle = nullptr;
        }
#endif
        ClearPort();
    }
}

// Open and setup serial port
bool MainWindow::SetupCommPort()
{
    // If command line was specified for BAUD rate or COM port, use it
    if(!str_cmd_baud.isEmpty())
    {
        for(int i = 0; i < ui->cbBaudRate->count(); i++)
        {
            if(ui->cbBaudRate->itemText(i) == str_cmd_baud)
                ui->cbBaudRate->setCurrentIndex(i);
        }
    }
    if(!str_cmd_port.isEmpty())
    {
        for(int i = 0; i < ui->cbCommport->count(); i++)
        {
            if(ui->cbCommport->itemText(i).contains(str_cmd_port))
                ui->cbCommport->setCurrentIndex(i);
        }

    }

    int baud_rate = ui->cbBaudRate->currentText().toInt();
    m_settings.setValue("Baud",baud_rate);

    QString comm_port = ui->cbCommport->currentText();
    m_settings.setValue("port",comm_port);

    if (m_CommPort)
    {
        ClearPort();
    }

    // Find the serial port ID from the index of serial port combo box
    QString serialPortName;

    int serialPortBaudRate = as32BaudRate[ui->cbBaudRate->currentIndex()];
    bool bFlow = ui->btnFlowCntrl->isChecked();
    if (ui->cbCommport->itemData(ui->cbCommport->currentIndex()).toString().compare("0") == 0)
    {
        m_CommPort = new WicedSerialPortHostmode();
        serialPortName = "host-mode";
    }
    else
    {
         m_CommPort = new WicedSerialPort();
        // Find the serial port ID from the index of serial port combo box
        serialPortName = m_strComPortsIDs.at(ui->cbCommport->currentIndex());
    }

    m_bPortOpen = m_CommPort->open(serialPortName.toLocal8Bit().data(), serialPortBaudRate, bFlow);

    if (m_bPortOpen)
    {
        Log("Opened %s at speed: %u flow %s", serialPortName.toStdString().c_str(), serialPortBaudRate,
                ui->btnFlowCntrl->isChecked() ? "on":"off");

#if 0
        if(comm_port.contains("WICED Peripheral"))
        {
            m_bPeripheralUart = true;
            Log("Opened WICED Peripheral UART port. Only trace function is available.");
        }
#endif

        // on opening the port, set a 2 sec timer
        m_dmStartupTimer->start();
    }
    else
    {
        Log("Error opening serial port %s: Error number %d", serialPortName.toStdString().c_str(),
            static_cast<int>(m_CommPort->errorNum()));
    }

    return m_bPortOpen;

}

// After timer ticks on opening the port, set visibilit/pairing mode of embedded device
// and query supported features (using GetVersion)
void MainWindow::startUpTimer()
{
    Startup();
}

int MainWindow::FindBaudRateIndex(int baud)
{
    int index = 0;

    for (; index < static_cast<int> ((sizeof(as32BaudRate) / sizeof(as32BaudRate[0]))); index++)
        if (as32BaudRate[index] == baud)
            return index;

    return 0;
}


// Close port
void MainWindow::CloseCommPort()
{
    if (m_port_read_worker)
    {
        m_bClosing = true;
        if (m_CommPort && m_CommPort->isOpen())
            m_CommPort->indicate_close();

        serial_read_wait.wakeAll();

        if(m_port_read_thread)
        {
            m_port_read_thread->exit();
            if (!m_port_read_thread->wait(2000))
            {
                m_port_read_thread->terminate();
                m_port_read_thread->wait(1000);
            }
        }
    }

    if (m_CommPort && m_CommPort->isOpen())
        m_CommPort->close();

    m_bPortOpen = false;
    m_bPeripheralUart = false;
}


// When the UI is closed, close port and save settings
void MainWindow::closeEventDm (QCloseEvent *)
{
    CloseCommPort();

    // save settings for baudrate, serial port and flow-ctrl
    QSettings settings(m_SettingsFile, QSettings::IniFormat);
    int baud_rate = ui->cbBaudRate->currentText().toInt();
    m_settings.setValue("Baud",baud_rate);

    bool flow_control = ui->btnFlowCntrl->isChecked();
    m_settings.setValue("FlowControl",flow_control);

    QString comm_port = ui->cbCommport->currentText();
    m_settings.setValue("port",comm_port);
}

// Clear port and UI
void MainWindow::ClearPort()
{
    CloseCommPort();
    QThread::sleep(1);
    if(m_CommPort)
        delete m_CommPort;
    m_CommPort = nullptr;

    Log("Serial port closed.");
    if(m_bUIEnabled)
        Log("Client Control app disconnected from Bluetooth device.");

    EnableUI(false);
    ui->cbCommport->setEnabled(true);
    ui->cbBaudRate->setEnabled(true);
    ui->btnFlowCntrl->setEnabled(true);
    ui->btnOpenPort->setText("Open Port");
}



void MainWindow::serialPortError(QSerialPort::SerialPortError error)
{
    //Print error etc.
    qDebug("serialPortError %d", error);
}

void MainWindow::handleReadyRead()
{
    m_CommPort->handleReadyRead();
}

/****************** Serial port thread **************/

// Create a thread the read serial port
void MainWindow::CreateReadPortThread()
{
    m_port_read_thread = new QThread;
    m_port_read_worker = new Worker();
    m_port_read_worker->moveToThread(m_port_read_thread);
    m_bClosing = false;
    m_port_read_worker->m_pParent = this;


    connect(m_port_read_thread, SIGNAL(started()), m_port_read_worker, SLOT(read_serial_port_thread()));
    connect(m_port_read_worker, SIGNAL(finished()), m_port_read_thread, SLOT(quit()));
    connect(m_port_read_worker, SIGNAL(finished()), m_port_read_worker, SLOT(deleteLater()));
    connect(m_port_read_thread, SIGNAL(finished()), m_port_read_thread, SLOT(deleteLater()));
}

// Serial port read thread
void Worker::read_serial_port_thread()
{
    unsigned char au8Hdr[1024 + 6];
    memset(au8Hdr, 0, 1030);
    int           offset = 0, pktLen;
    int           packetType;

    // While the port is not closed, keep reading
    while (!m_bClosing)
    {
        memset(au8Hdr, 0, 1030);

#ifdef WIN32
        // If user open Peripheral uart, just log the traces to trace window
        if(m_pParent->m_bPeripheralUart)
        {

            // Read one chunk at a time
            DWORD dwLen = m_pParent->m_CommPort->readline((char *)au8Hdr,600);

            if (m_bClosing)
                break;
            // Read the bytes in a string, break string at \n char, print the trace to log window
            else if(dwLen)
            {
                QByteArray arr((const char *)au8Hdr, dwLen);
                while(arr.size())
                {
                    int index =  arr.indexOf('\n', 0);
                    if(index > 0)
                    {
                        QByteArray arrLeft = arr.left(index);
                        char szString[601] = {0};
                        strncpy(szString, arrLeft.data(), 600);
                        m_pParent->Log("%s", szString);
                        if(index+1 < arr.length())
                            arr = arr.right(arr.length() - index-1);
                        else
                            break;
                    }
                    else
                        break;
                }
            }
            continue;
        }
#endif

        offset = 0;
        // Read HCI packet
        pktLen = static_cast<int>(ReadNewHciPacket(au8Hdr, sizeof(au8Hdr), &offset));
        if (m_bClosing || pktLen < 0) // skip this
            break;

        if (pktLen + offset == 0)
            continue;

        packetType = au8Hdr[0];
        if (HCI_WICED_PKT == packetType)
        {
            DWORD channel_id = static_cast<DWORD>(au8Hdr[1] | (au8Hdr[2] << 8));
            DWORD len = static_cast<DWORD>(au8Hdr[3] | (au8Hdr[4] << 8));

            if (len > 1024)
            {
                m_pParent->Log("Skip bad packet %ld", len);
                continue;
            }

            // If we are streaming audio, call the method to request audio data request
            // directly instead of posting on main thread. Posting on main thread
            // can cause audio glitches because of other UI activity.
            if(m_pParent->m_audio_started && (HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA == channel_id))
            {
                if (m_pParent->m_uAudio.m_pAudioData != nullptr)
                {
                    m_pParent->HandleA2DPAudioRequestEvent(&au8Hdr[5], len);
                }
            }
            else if(HCI_CONTROL_HIDH_EVENT_AUDIO_DATA == channel_id)
            {
                m_pParent->HandleHidHAudioRxData(&au8Hdr[7], len-2);
            }
            // If we are receiving file via BSG, SPP or iAP2, call the method to recive data directly
            // instead of posting on main thread, as posting on main thread will cause UI hang
            else if((HCI_CONTROL_BSG_EVENT_RX_DATA == channel_id) && (m_pParent->m_bsg_receive_file))
            {
                m_pParent->HandleBSGEvents(HCI_CONTROL_BSG_EVENT_RX_DATA, &au8Hdr[5], len);
            }
            else if ((HCI_CONTROL_SPP_EVENT_RX_DATA == channel_id) && (m_pParent->m_spp_receive_file))
            {
                m_pParent->HandleSPPEvents(HCI_CONTROL_SPP_EVENT_RX_DATA, &au8Hdr[5], len);
            }
            else if ((HCI_CONTROL_IAP2_EVENT_RX_DATA == channel_id) && (m_pParent->m_iap2_receive_file))
            {
                m_pParent->HandleiAP2PEvents(HCI_CONTROL_IAP2_EVENT_RX_DATA, &au8Hdr[5], len);
            }
            else
            {
                BYTE * pBuf = nullptr;

                if(len)
                {
                    // malloc and create a copy of the data.
                    //  MainWindow::onHandleWicedEvent deletes the data
                    pBuf = static_cast<BYTE*>(malloc(len));
                    memcpy(pBuf, &au8Hdr[5], len);
                }

                // send it to main thread
                emit m_pParent->HandleWicedEvent(static_cast<unsigned int>(channel_id),
                    static_cast<unsigned int>(len), pBuf);
            }
        }
        else if (packetType == HCI_EVENT_PKT)
        {
            // If this is a Vendor Specific Event
            if (au8Hdr[1] == HCI_VSE_OPCODE)
            {
                // Call the VSE handler
                process_vse(au8Hdr, static_cast<DWORD>(pktLen));
            }
            else
            {
                dump_hci_data_hexa("HCI Event:", au8Hdr, static_cast<DWORD>(pktLen));
            }
        }
    }

    // When the thread exits, the m_port_read_thread is auto deleted, set the pointer to NULL
    m_pParent->m_port_read_thread = nullptr;
    emit finished();
}

// Read HCI packet
DWORD Worker::ReadNewHciPacket(BYTE * pu8Buffer, int bufLen, int * pOffset)
{
    DWORD dwLen, len = 0, offset = 0;

    dwLen = Read(&pu8Buffer[offset], 1);

    if (static_cast<int>(dwLen) <= 0 || m_bClosing)
        return (static_cast<DWORD>(-1));

    offset++;

    switch (pu8Buffer[0])
    {
    case HCI_EVENT_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 2);
        if(dwLen == 2)
        {
            len = pu8Buffer[2];
            offset += 2;
        }
        else
            m_pParent->Log("error HCI_EVENT_PKT, needed 2 got %ld", dwLen);
    }
        break;

    case HCI_ACL_DATA_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = static_cast<DWORD>(pu8Buffer[3] | (pu8Buffer[4] << 8));
            offset += 4;
            m_pParent->Log("HCI_ACL_DATA_PKT, len %ld", len);
        }
        else
            m_pParent->Log("error HCI_ACL_DATA_PKT needed 4 got %ld", dwLen);
    }
        break;

    case HCI_WICED_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = static_cast<DWORD>(pu8Buffer[3] | (pu8Buffer[4] << 8));
            offset += 4;
        }
        else
            m_pParent->Log("error HCI_WICED_PKT,  needed 4 got %ld", dwLen);
    }
        break;
    default:
    {
        if(m_pParent->m_bUIEnabled)
        {
            qDebug("error unknown packet, type %d", pu8Buffer[0]);
            //m_pParent->Log("error unknown packet, type %d", pu8Buffer[0]);
        }
    }
    }

    if(len > 1024)
    {
        m_pParent->Log("bad packet %ld", len);
        return static_cast<DWORD>(-1); // bad packet
    }

    if (len)
    {
        DWORD lenRd = m_pParent->qtmin(len, static_cast<DWORD>(static_cast<DWORD>(bufLen)-offset));
        dwLen = Read(&pu8Buffer[offset], lenRd);
        if(dwLen != lenRd)
            m_pParent->Log("read error to read %ld, read %ld", lenRd, dwLen);
    }

    *pOffset = static_cast<int>(offset);

    return len;
}

// Read from serial port
DWORD Worker::Read(BYTE *lpBytes, DWORD dwLen)
{
    BYTE * p = lpBytes;
    DWORD Length = dwLen;
    int64_t dwRead = 0;
    DWORD dwTotalRead = 0;

    if(!m_pParent->m_CommPort)
        return 0;

    QMutex mutex;
    int retry_cnt = 0;
    // Loop here until request is fulfilled or port is closed
    while (Length && !m_bClosing)
    {
        if(m_bClosing)
            return 0;

        dwRead = 0;
        char buff_temp[1030];
        memset(buff_temp, 0, 1030);

        dwRead = m_pParent->m_CommPort->read(buff_temp,static_cast<qint64>(Length));
        if(dwRead <= 0)
        {
            if(m_bClosing)
                return 0;

            if(dwRead < 0)
            {
                m_pParent->Log("Error in port read");
                return 0;
            }

            // retry 3 time with longer timeout for each subsequent
            unsigned long ulTimeout = 20;
            if(retry_cnt < 3)
                {
                    retry_cnt++;
                    ulTimeout *= static_cast<unsigned long>(retry_cnt);
                }
                else
                    ulTimeout = ULONG_MAX;

            //qDebug("serial_read wait Length %d, timeout %d, retry %d", Length, ulTimeout, retry_cnt);
            // If dwRead == 0, then we need to wait for device to send the MCU app data
            mutex.lock();
             //serial_read_wait is set when there is more data or when the serial port is closed
            m_pParent->serial_read_wait.wait(&mutex, ulTimeout);
            mutex.unlock();
            //qDebug("serial_read continue");
        }
        else
        {
            memcpy(p, buff_temp, dwRead);
            retry_cnt = 0;
        }

        if (dwRead > Length)
            break;
        p += dwRead;
        Length -= dwRead;
        dwTotalRead += dwRead;
    }

    return dwTotalRead;
}

// Dump HCI Data in the trace window
void Worker::dump_hci_data_hexa(const char *p_prefix, BYTE *pu8Buffer, DWORD packet_len)
{
    char trace[4000];
    char byte_str[10];

    memset(trace, 0, sizeof(trace));

    for(unsigned int i = 0; i < packet_len; i++)
    {
        sprintf(byte_str, "%02X ", pu8Buffer[i]);
        strcat(trace, byte_str);
    }
    if (p_prefix)
        m_pParent->Log("%s %s", p_prefix, trace);
    else
        m_pParent->Log("HCI Event:%s", trace);
}

// Function processing the Vendor Specific Events
void Worker::process_vse(BYTE *pu8Buffer, DWORD packet_len)
{
    // If this is not a Debug FW Dump event
    if ((packet_len < 5) ||
        (pu8Buffer[3] != HCI_VSE_SUBOPCODE_DBFW_DUMP) ||
        (pu8Buffer[4] != HCI_VSE_TYPE_COREDUMP))
    {
        m_pParent->Log("not DBFW VSE 0x%x 0x%x", pu8Buffer[3], pu8Buffer[4]);
        dump_hci_data_hexa("HCI Event:", pu8Buffer, packet_len);
        return;
    }

    if (packet_len < 8)
    {
        m_pParent->Log("Bad VSE len %ld", packet_len);
        dump_hci_data_hexa("HCI Event:", pu8Buffer, packet_len);
        return;
    }

    switch(pu8Buffer[7])
    {
    case HCI_VSE_TYPE_COREDUMP_INFO:
        dump_hci_data_hexa("Debug FW CoreDump Info:", pu8Buffer, packet_len);
        break;

    case HCI_VSE_TYPE_COREDUMP_CPU_REGS:
        dump_hci_data_hexa("Debug FW CoreDump CPU Registers:", pu8Buffer, packet_len);
        m_pParent->Log("SP:%08X PC:%08X LR:%08X",
                pu8Buffer[11] | (pu8Buffer[12] << 8) | (pu8Buffer[13] << 16) | (pu8Buffer[14] << 24),
                pu8Buffer[15] | (pu8Buffer[16] << 8) | (pu8Buffer[17] << 16) | (pu8Buffer[18] << 24),
                pu8Buffer[19] | (pu8Buffer[20] << 8) | (pu8Buffer[21] << 16) | (pu8Buffer[22] << 24));
        m_pParent->Log("R0:%08X R1:%08X R2:%08X",
                pu8Buffer[23] | (pu8Buffer[24] << 8) | (pu8Buffer[25] << 16) | (pu8Buffer[26] << 24),
                pu8Buffer[27] | (pu8Buffer[28] << 8) | (pu8Buffer[29] << 16) | (pu8Buffer[30] << 24),
                pu8Buffer[31] | (pu8Buffer[32] << 8) | (pu8Buffer[33] << 16) | (pu8Buffer[34] << 24));
        m_pParent->Log("R3:%08X R4:%08X R5:%08X R6:%08X",
                pu8Buffer[35] | (pu8Buffer[36] << 8) | (pu8Buffer[37] << 16) | (pu8Buffer[38] << 24),
                pu8Buffer[39] | (pu8Buffer[40] << 8) | (pu8Buffer[41] << 16) | (pu8Buffer[42] << 24),
                pu8Buffer[43] | (pu8Buffer[44] << 8) | (pu8Buffer[45] << 16) | (pu8Buffer[46] << 24),
                pu8Buffer[47] | (pu8Buffer[48] << 8) | (pu8Buffer[49] << 16) | (pu8Buffer[50] << 24));
        break;

    case HCI_VSE_TYPE_COREDUMP_CPU_REGS_EXT:
        dump_hci_data_hexa("Debug FW CoreDump Extended CPU Registers:", pu8Buffer, packet_len);
        break;

    case HCI_VSE_TYPE_COREDUMP_RAM_IMAGE:
        if (m_pParent->ui->btnLogToFile->isChecked())
        {
            dump_hci_data_hexa("Debug FW CoreDump RAM Image:", pu8Buffer, packet_len);
        }
        break;

    case HCI_VSE_TYPE_COREDUMP_CALL_STACK:
        dump_hci_data_hexa("Debug FW CoreDump Call Stack:", pu8Buffer, packet_len);
        break;

    default:
        if (m_pParent->ui->btnLogToFile->isChecked())
        {
            dump_hci_data_hexa("Unknown DBFW EVENT:", pu8Buffer, packet_len);
        }
        break;
    }
}



/****************** Script thread **************/
static MainWindow *g_parent = nullptr;

// Create a thread for scriping
void MainWindow::CreateScriptThread()
{
    m_script_read_thread = new QThread;
    m_script_read_worker = new Worker();
    m_script_read_worker->moveToThread(m_script_read_thread);
    m_bClosing = false;
    m_script_read_worker->m_pParent = this;
    g_parent = this;

#ifdef WIN32
    connect(m_script_read_thread, SIGNAL(started()), m_script_read_worker, SLOT(read_script_thread()));
#endif

    connect(m_script_read_worker, SIGNAL(finished()), m_script_read_thread, SLOT(quit()));
    connect(m_script_read_worker, SIGNAL(finished()), m_script_read_worker, SLOT(deleteLater()));
    connect(m_script_read_thread, SIGNAL(finished()), m_script_read_thread, SLOT(deleteLater()));

    m_script_read_thread->start();
}

extern "C" void hci_control_script_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t len);
// Call WICED HCI API from script

int MainWindow::CallWicedHciApi(UINT8 *p_data, UINT8 len)
{
    char s[1200] = {0};
    uint16_t opcode = 0;
    uint16_t payload_len = 0;

    for(int i = 0, j = 0; i < len; i++)
    {
        j = i*3;
        sprintf(&s[j], "x%X,", p_data[i]);
    }

    STREAM_TO_UINT16 (opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16 (payload_len, p_data);  // Gen Payload Length
    hci_control_script_handle_command (opcode, p_data, payload_len);
    return 0;
}

// Function called when COM port is opened or when client control
// receives device started event
void MainWindow::Startup()
{
    Log("Startup");

    // Set Discoverable and/or Connectable (depending on CheckBoxes)
    setVis();
    // Set Pairable (depending on CheckBox)
    setPairingMode();
    // get SDK version to and display in UI
    GetVersion();

    // read the link keys saved by app
    m_settings.sync();

    // Send the paired device keys saved by client control to WICED app
    WriteNVRAMToDevice(false);
    WriteNVRAMToDevice(true);
    SendHidHostAdd();
    SendBattCAdd();
    SendFindMeLocatorAdd();
}

void msleep(unsigned int to)
{
    QThread::msleep(to);
}

void MainWindow::DisableBluetoothClassic()
{
    ui->btnStartDisc->setEnabled(false);
    ui->btnStopDisc->setEnabled(false);
    ui->btnUnbond->setEnabled(false);
    ui->cbDeviceList->setEnabled(false);
}
