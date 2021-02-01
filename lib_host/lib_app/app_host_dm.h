
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


#ifndef APP_HOST_DM_H
#define APP_HOST_DM_H

#include "wiced_hci_dm.h"
#include "hci_control_api.h"
#include "app_host.h"

// device manager
extern bool app_host_dm_set_device_name(char * dev_nam);
extern bool app_host_dm_set_device_addr(uint8_t * bda);
extern bool app_host_dm_set_pairing_mode(bool mode);
extern bool app_host_dm_set_vis(bool disc, bool connectable);
extern bool app_host_dm_reset();
extern bool app_host_dm_delete_nvram_data(int nvram_id);
extern bool app_host_dm_inquiry(bool start_stop);
extern bool app_host_dm_le_scan(bool enable);
extern bool app_host_dm_push_pairing_host_info(uint8_t * nvram_data, int len);
extern bool app_host_dm_push_nvram_data(int nvram_id, uint8_t * nvram_data, int len);
extern bool app_host_dm_hidh_add(int bda[BD_ADDR_LEN]);
extern bool app_host_dm_add_battery_client(int bda[BD_ADDR_LEN]);
extern bool app_host_dm_add_findme_locator(int bda[BD_ADDR_LEN]);
extern bool app_host_dm_get_version_info();
extern bool app_host_dm_set_app_traces(bool enable, uint8_t route);
extern bool app_host_dm_user_confirm(uint8_t * bda, bool accept);
extern bool app_host_dm_unbond_device(uint8_t * bda);

// device management API
void app_host_remove_device(uint8_t bda[6]);
void app_host_remove_all_devices();
wiced_hci_bt_device_t *app_host_find_device_by_connection(uint16_t conn_type, uint16_t handle);
wiced_hci_bt_device_t *app_host_find_device_by_handle(uint16_t handle);
extern wiced_hci_bt_device_t* app_host_find_device(uint8_t bda[6]);
extern wiced_hci_bt_device_t*  app_host_add_device(uint8_t bda[6]);

#endif
