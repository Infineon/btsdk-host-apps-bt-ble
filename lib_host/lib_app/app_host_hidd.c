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


#include "app_host.h"
#include "wiced_hci_hidd.h"
#include <string.h>


bool app_host_hidd_disconnect()
{
    return wiced_hci_hidd_disconnect();
}

bool app_host_hidd_connect()
{
    return wiced_hci_hidd_connect();
}

bool app_host_hidd_send_report(uint8_t channel, uint8_t report_id, uint8_t *report, uint8_t report_len)
{
	wiced_hci_bt_hidd_report_t data;
	data.channel = channel;
	data.report_id = report_id;
	data.report_len = report_len;
	data.report = report;

	return wiced_hci_hidd_send_report(&data);
}

bool app_host_hidd_pairing_mode(uint8_t m_pairing_mode_active)
{
    wiced_hci_bt_hidd_paring_mode_data_t data;
	data.pairing_mode = m_pairing_mode_active;
	return wiced_hci_hidd_pairing_mode(&data);
}

bool app_host_hidd_key(uint8_t key, uint8_t keyDown)
{
	return wiced_hci_hidd_key(key, keyDown);
}

bool app_host_hidd_send_key(uint8_t cap_lock, uint8_t ctrl_key, uint8_t alt_key, char * buffer, uint8_t btn_up)
{
	wiced_hci_bt_hidd_send_key_data_t data;

	data.cap_lock = cap_lock;
	data.ctrl_key = ctrl_key;
	data.alt_key = alt_key;
	memcpy(data.buffer, buffer,6);
	data.btn_up = btn_up;

	return wiced_hci_bt_hidd_send_key(&data);
}

bool app_host_hidd_cap_lock(uint8_t cap_lock, uint8_t ctrl_key, uint8_t alt_key)
{
	wiced_hci_bt_hidd_cap_lock_data_t data;

	data.cap_lock = cap_lock;
	data.ctrl_key = ctrl_key;
	data.alt_key = alt_key;

	return wiced_hci_bt_hidd_cap_lock(&data);
}

bool app_host_hidd_get_host_info()
{
	return wiced_hci_hidd_get_host_info();
}

bool app_host_hidd_virtual_unplug()
{
	return wiced_hci_bt_hidd_virtual_unplug();
}
