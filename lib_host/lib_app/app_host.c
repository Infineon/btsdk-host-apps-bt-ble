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
#include <string.h>
#include <stdlib.h>

// Global singleton app
wiced_host_app_t g_app;

// init device profile handles
void app_host_init_device_handles(wiced_hci_bt_device_t *p_dev)
{
    p_dev->m_audio_handle = WICED_NULL_HANDLE;
    p_dev->m_hf_handle = WICED_NULL_HANDLE;
    p_dev->m_ag_handle = WICED_NULL_HANDLE;
    p_dev->m_spp_handle = WICED_NULL_HANDLE;
    p_dev->m_hidh_handle = WICED_NULL_HANDLE;
    p_dev->m_iap2_handle = WICED_NULL_HANDLE;
    p_dev->m_avrc_handle = WICED_NULL_HANDLE;
    p_dev->m_le_handle = WICED_NULL_HANDLE;
    p_dev->m_bsg_handle = WICED_NULL_HANDLE;
    p_dev->m_pbc_handle = WICED_NULL_HANDLE;
    p_dev->m_avk_handle = WICED_NULL_HANDLE;
    p_dev->m_battc_handle = WICED_NULL_HANDLE;
    p_dev->m_findmel_handle = WICED_NULL_HANDLE;
    p_dev->m_ops_handle = WICED_NULL_HANDLE;
}

// add device to list
wiced_hci_bt_device_t*  app_host_add_device(uint8_t bda[6])
{
    bool found = false;

    wiced_hci_bt_device_t *p_dev_list = g_app.p_peer_devices;
    wiced_hci_bt_device_t *p_dev_previous = 0;

    while(p_dev_list)
    {
        if(memcmp((p_dev_list->m_address), bda, BDA_LEN) == 0)
        {
            found = true;
            break;
        }
        p_dev_previous = p_dev_list;
        p_dev_list = (wiced_hci_bt_device_t *)p_dev_list->p_next;

    }

    if(!found)
    {
        p_dev_list = (wiced_hci_bt_device_t *)malloc(sizeof(wiced_hci_bt_device_t));
        memset(p_dev_list, 0, sizeof(wiced_hci_bt_device_t));
        app_host_init_device_handles(p_dev_list);
        memcpy((p_dev_list->m_address), bda, BDA_LEN);

        if(p_dev_previous)
            p_dev_previous->p_next = p_dev_list;
        else
             g_app.p_peer_devices = p_dev_list;
    }

    return p_dev_list;

}

// remove device from list
void app_host_remove_device(uint8_t bda[6])
{
    bool found = false;

    wiced_hci_bt_device_t *p_dev_list =  g_app.p_peer_devices;
    wiced_hci_bt_device_t *p_dev_previous = 0;

    while(p_dev_list)
    {
        if(memcmp((p_dev_list->m_address), bda, BDA_LEN) == 0)
        {
            found = true;
            break;

        }
        p_dev_previous = p_dev_list;
        p_dev_list= p_dev_list->p_next;
    }

    if(found)
    {
        if(p_dev_previous)
            p_dev_previous->p_next = p_dev_list->p_next;
        else
             g_app.p_peer_devices = (wiced_hci_bt_device_t *)p_dev_list->p_next;

        free(p_dev_list);
    }
}

void app_host_remove_all_devices()
{
    wiced_hci_bt_device_t *p_dev_list =  g_app.p_peer_devices;
    wiced_hci_bt_device_t *p_dev_next = 0;

    while(p_dev_list)
    {
        p_dev_next = p_dev_list->p_next;
        free(p_dev_list);
        p_dev_list = p_dev_next;
    }
     g_app.p_peer_devices = 0;

}

// Find device in list by BDA
wiced_hci_bt_device_t* app_host_find_device(uint8_t bda[6])
{
    wiced_hci_bt_device_t *p_dev_list =  g_app.p_peer_devices;

    while(p_dev_list)
    {
        if(memcmp((p_dev_list->m_address), bda, BDA_LEN) == 0)
        {
            return p_dev_list;
        }
        p_dev_list = (wiced_hci_bt_device_t *)p_dev_list->p_next;
    }

    return 0;
}

// Find a device in list of devices using connection type and handle
wiced_hci_bt_device_t *app_host_find_device_by_connection(uint16_t conn_type, uint16_t handle)
{
    wiced_hci_bt_device_t *p_dev_found = 0;
    wiced_hci_bt_device_t *p_dev_list =  g_app.p_peer_devices;

    while(p_dev_list)
    {
        if(p_dev_list->m_conn_type & conn_type)
        {
            // if connection type and handle match, return the device
            switch(conn_type)
            {
            case WICED_CONNECTION_TYPE_AG:
                if(handle == p_dev_list->m_ag_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_SPP:
                if(handle == p_dev_list->m_spp_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_AUDIO:
                if(handle == p_dev_list->m_audio_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_HF:
                if(handle == p_dev_list->m_hf_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_HIDH:
                if(handle == p_dev_list->m_hidh_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_IAP2:
                if(handle == p_dev_list->m_iap2_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_LE:
                if(handle == p_dev_list->m_le_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_AVRC:
                if(handle == p_dev_list->m_avrc_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_AVK:
                if(handle == p_dev_list->m_avk_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_PBC:
                if(handle == p_dev_list->m_pbc_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_BATTC:
                if(handle == p_dev_list->m_battc_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_FINDMEL:
                if(handle == p_dev_list->m_findmel_handle)
                    return p_dev_list;
                break;
            case WICED_CONNECTION_TYPE_OPS:
                if(handle == p_dev_list->m_ops_handle)
                    return p_dev_list;
                break;
            }
        }


        p_dev_list = (wiced_hci_bt_device_t *)p_dev_list->p_next;
    }

    // if exact handle match not found, return the connected
    // device for the connection or NULL
    return p_dev_found;
}

wiced_hci_bt_device_t *app_host_find_device_by_handle(uint16_t handle)
{
    wiced_hci_bt_device_t *p_dev_list =  g_app.p_peer_devices;
    while(p_dev_list)
    {
        if(p_dev_list->m_spp_handle == handle)
            return p_dev_list;

        p_dev_list = (wiced_hci_bt_device_t *)p_dev_list->p_next;
    }
    return 0;
}

// Stub functions to be implemented by customer applications
// app init
void app_host_init()
{
    memset(&g_app, 0, sizeof(wiced_host_app_t));
}

void app_host_deinit()
{
    app_host_remove_all_devices();
}


void app_host_handle_event(uint16_t opcode, uint8_t * p_data, uint32_t len)
{
    UNUSED(opcode);
    UNUSED(p_data);
    UNUSED(len);
}
