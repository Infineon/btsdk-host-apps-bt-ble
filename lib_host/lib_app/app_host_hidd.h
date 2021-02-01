
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


#ifndef APP_HOST_HIDD_H
#define APP_HOST_HIDD_H

#include "wiced_hci_hidd.h"
#include "hci_control_api.h"

enum {
    KEY_IR      = 0xf0,
    KEY_AUDIO   = 0xf1,
    KEY_MOTION  = 0xf2,
    KEY_CONNECT = 0xf3,
};

// HID Device
bool app_host_hidd_connect();
bool app_host_hidd_disconnect();
bool app_host_hidd_pairing_mode(uint8_t m_pairing_mode_active);
bool app_host_hidd_send_report(uint8_t channel, uint8_t report_id, uint8_t *report, uint8_t report_len);
bool app_host_hidd_send_key(uint8_t cap_lock, uint8_t ctrl_key, uint8_t alt_key, char * buffer, uint8_t btn_up);
bool app_host_hidd_cap_lock(uint8_t cap_lock, uint8_t ctrl_key, uint8_t alt_key);
bool app_host_hidd_virtual_unplug();
bool app_host_hidd_get_host_info();
bool app_host_hidd_key(uint8_t key, uint8_t keyDown);   // key should be USB HID Usage Keyboard (page 0x07) keys. 0xf0-0xff is customzied.


#endif
