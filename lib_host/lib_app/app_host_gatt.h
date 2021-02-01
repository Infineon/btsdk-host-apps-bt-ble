
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


#ifndef APP_HOST_GATT_H
#define APP_HOST_GATT_H

#include "wiced_hci_gatt.h"
#include "hci_control_api.h"


// GATT
bool app_host_gatt_connect(uint8_t address_type, uint8_t address[BDA_LEN]);
bool app_host_gatt_cancel_connect(uint8_t address_type, uint8_t address[BDA_LEN]);
bool app_host_gatt_le_disconnect(uint16_t conn_handle);
bool app_host_gatt_disc_services(uint16_t conn_handle);
bool app_host_gatt_disc_chars(uint32_t s_handle, uint32_t e_handle, uint16_t conn_handle);
bool app_host_gatt_disc_desc(uint32_t s_handle, uint32_t e_handle, uint16_t conn_handle);
bool app_host_gatt_send_notif(uint16_t con_handle, uint32_t hdlc, uint8_t * buffer, uint32_t num_bytes);
bool app_host_gatt_send_indicate(uint16_t con_handle, uint32_t hdlc, uint8_t * buffer, uint32_t num_bytes);
bool app_host_gatt_write_char(uint16_t con_handle, uint32_t hdlc, uint8_t * str, uint32_t num_bytes);
bool app_host_gatt_write_char_no_rspn(uint16_t con_handle, uint32_t hdlc, uint8_t * str, uint32_t num_bytes);
bool app_host_gatt_read_char(uint16_t con_handle, uint32_t hdlc);
bool app_host_gatt_write_response(uint8_t *p_data);
bool app_host_gatt_read_response(uint8_t *p_data);
bool app_host_gatt_start_stop_advert(bool start_stop);

#endif
