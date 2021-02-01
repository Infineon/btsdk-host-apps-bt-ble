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



#include "wiced_hci.h"
#include "hci_control_api.h"
#include <string.h>

#define WICED_HCI_DATA_MAX 1030

// extern API, Implemented by application
extern int app_host_port_write(uint8_t *data, uint32_t len);

// Send WICED HCI command
bool wiced_hci_send_command(uint16_t command, uint8_t * payload, uint32_t len)
{
    uint8_t    data[WICED_HCI_DATA_MAX];
    uint32_t totalLength = 0;
    uint16_t    header  = 0;

    memset(data, 0, sizeof(data));

    // If command header byte is not set, set it now
    if (command)
    {
        data[header++] = HCI_WICED_PKT;
        data[header++] = command & 0xff;
        data[header++] = (command >> 8) & 0xff;
        data[header++] = len & 0xff;
        data[header++] = (len >> 8) & 0xff;
    }

    if(len)
        memcpy(&data[header], payload, len);

    totalLength = header+len;

    if(totalLength == 0)
        return false;

    return (app_host_port_write( data,  totalLength) > 0 ? true : false);
}
