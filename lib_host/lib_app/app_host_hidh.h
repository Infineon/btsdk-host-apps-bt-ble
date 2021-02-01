
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


#ifndef APP_HOST_HIDH_H
#define APP_HOST_HIDH_H

#include "app_host.h"
#include "wiced_hci_hidh.h"
#include "hci_control_api.h"

// HID HOST
bool app_host_hidh_connect(uint8_t bda[6]);
bool app_host_hidh_set_report(uint16_t handle, uint8_t channle, uint8_t report_type, uint8_t report_id, char * string, uint32_t length);
bool app_host_hidh_get_report(uint16_t handle, uint8_t report_type, uint8_t report_id);
bool app_host_hidh_disconnect(uint8_t bda[6]);
bool app_host_hidh_virtual_unplug(uint8_t bda[6]);
bool app_host_hidh_get_desc(uint16_t nHandle);
bool app_host_hidh_set_proto(uint16_t nHandle, uint8_t protocol);
bool app_host_hidh_set_wakeup_pattern(uint8_t bda[6], uint8_t report_id, uint8_t *report_pattern, uint8_t report_len);
bool app_host_hidh_set_wakeup_control(uint8_t wakeup_gpio, uint8_t wakeup_polarity, uint8_t m_hidh_wakeup_state);
void app_host_hidh_event(uint16_t opcode, uint8_t *p_data, uint32_t len);


#endif
