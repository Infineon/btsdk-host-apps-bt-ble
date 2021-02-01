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
#ifndef KP3UARTRTSWORKAROUND_H
#define KP3UARTRTSWORKAROUND_H
#include <QString>
#include <windows.h>

#ifndef Q_OS_WIN
#error This header can be included only for windows platform
#endif

namespace Cypress {
namespace KitProg3Sepcifics {

//
// This module allows to handle
// the UART RTS/DTR lines of KitProg3 device in a specific way via 'usbser' driver on Windows.
//
// Workaround for
// 1. https://jira.cypress.com/browse/BTSDK-4891
// KP3_RTS (BT_UART_CTS) stays high when Clientcontrol com port is enabled
// 2. https://jira.cypress.com/browse/CYBLUETOOL-369
// KP3_RTS (BT_UART_CTS) stays high when Bluetool com port is enabled
//
class Kp3UartWorkaround
{
    Kp3UartWorkaround() = delete;
    static void Log(const char * fmt, ...);
    static void GetSerialDeviceDetailsUsingSetupAPI(const QString& portNameToFind, QString& driver_name_out);
public:
    // This static method activates RTS/DTR control lines for KitProg3 UART device
    // via 'usbser' driver on Windows platform.
    // For FTDI based devices this call does nothing
    static void assertRtsDtrLinesForKP3OnWindows(const QString& com_port_name_or_path, HANDLE com_port_handle);
};

}
}

#endif // KP3UARTRTSWORKAROUND_H
