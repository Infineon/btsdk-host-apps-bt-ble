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
 *  Workaround for https://jira.cypress.com/browse/BTSDK-4891 -
 *  KP3_RTS (BT_UART_CTS) stays high when Clientcontrol com port is enabled
 *  Do addtional UART control flow handling, specific for the KP3 serial device on Windows.
 *  For FTDI based devices this call does nothing.
 */
#include "kp3uart_workaround.h"
#include <initguid.h>
#include <winioctl.h>
#include <setupapi.h>
#include <QRegularExpression>

namespace Cypress {
namespace KitProg3Sepcifics {

//
// Handling the KitProg3 UART RTS/DTR lines in a specific way via 'usbser' driver on Windows.
// Workaround for
// 1. https://jira.cypress.com/browse/BTSDK-4891
// KP3_RTS (BT_UART_CTS) stays high when Clientcontrol com port is enabled
// 2. https://jira.cypress.com/browse/CYBLUETOOL-369
// KP3_RTS (BT_UART_CTS) stays high when Bluetool com port is enabled
//

//
// This static method activates RTS/DTR control lines of KitProg3 UART device via 'usbser' driver on Windows platform.
// NOTE: For other UART devices (e.g. FTDI) it does nothing
//
void Kp3UartWorkaround::assertRtsDtrLinesForKP3OnWindows(const QString& com_port_name_or_path, HANDLE com_port_handle) {
    QString driver_name;
    GetSerialDeviceDetailsUsingSetupAPI(com_port_name_or_path, driver_name);
    Log("[Kp3UartRtsWorkaround] Detected driver_name: %s", driver_name.toStdString().c_str());

    auto isKp3Device = [] (const QString& driver_name) {
        if (driver_name == "usbser") return true;
        return false;
    };

    //
    // Do addtional RTS/DTR handling specific for the KP3 serial device
    //
    if (isKp3Device(driver_name))
    {
        Log ("[Kp3UartRtsWorkaround] Asserting RTS/DTR control lines of %s KP3 port", com_port_name_or_path.toStdString().c_str());
        EscapeCommFunction( com_port_handle, SETRTS );
        EscapeCommFunction( com_port_handle, SETDTR );
    }
}

//
// Obtains the information about Serial (Com port) device from the system using SetupAPI windows API,
// This information includes the OS driver name for the serial device.
// Workaround for
// 1. https://jira.cypress.com/browse/BTSDK-4891
// KP3_RTS (BT_UART_CTS) stays high when Clientcontrol com port is enabled
// 2. https://jira.cypress.com/browse/CYBLUETOOL-369
// KP3_RTS (BT_UART_CTS) stays high when Bluetool com port is enabled
//
void Kp3UartWorkaround::GetSerialDeviceDetailsUsingSetupAPI(const QString& portNameToFind, QString& driver_name_out)
{
    //Create a "device information set" for the specified GUID
    HDEVINFO hDevInfoSet = SetupDiGetClassDevs(&GUID_DEVINTERFACE_SERENUM_BUS_ENUMERATOR, nullptr, nullptr,  DIGCF_PRESENT);
    if (hDevInfoSet == INVALID_HANDLE_VALUE) {
        Log("Failed to get com port device details");
        return;
    }

    //do the enumeration
    bool bMoreItems = true;
    bool portFound = false;
    int nIndex = 0;
    SP_DEVINFO_DATA devInfo{};
    while (bMoreItems)
    {
        //Enumerate the current device
        devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
        bMoreItems = SetupDiEnumDeviceInfo(hDevInfoSet, nIndex, &devInfo);
        if (bMoreItems)
        {
            HKEY hKey = SetupDiOpenDevRegKey(hDevInfoSet, &devInfo, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
            CHAR szBufferHkeyPort[512];
            DWORD dwBufferSize = sizeof(szBufferHkeyPort);
            ULONG nError;
            nError = RegQueryValueExA(hKey, "PortName", 0, NULL, reinterpret_cast<LPBYTE>(szBufferHkeyPort), &dwBufferSize);
            if (nError == ERROR_SUCCESS)
            {
                QRegularExpression regex {szBufferHkeyPort};
                if (regex.match(portNameToFind).hasMatch()) {
                    portFound = true;
                    break; // the dev info for the given port found
                }
            }
        }
        ++nIndex;
    }

    if (!portFound) {
        Log("COM port's [%s] properties were not found using SetupAPI", portNameToFind.toStdString().c_str());
    }

    QString driver_name;

    if (portFound) {
        DWORD dwSize, dwPropertyRegDataType;
        char szDesc[2048] = "\0";

        // Get the serial driver name, e.g. "usbser"
        if (SetupDiGetDeviceRegistryPropertyA(hDevInfoSet, &devInfo, SPDRP_SERVICE,
                                              &dwPropertyRegDataType, reinterpret_cast<BYTE*>(szDesc),
                                              sizeof(szDesc),   // The size, in bytes
                                              &dwSize)) {
            driver_name = szDesc;
        } else {
            Log("Failed to get serial driver name using SetupAPI");
        }
    }

    //Free up the "device information set" now that we are finished with it
    SetupDiDestroyDeviceInfoList(hDevInfoSet);

    driver_name_out = driver_name;
}

void Kp3UartWorkaround::Log(const char * fmt, ...)
{
    va_list cur_arg;
    va_start(cur_arg, fmt);
    char trace[1000];
    memset(trace, 0, sizeof(trace));
    vsprintf(trace, fmt, cur_arg);
    va_end(cur_arg);

    qDebug(trace);
}

}
}
