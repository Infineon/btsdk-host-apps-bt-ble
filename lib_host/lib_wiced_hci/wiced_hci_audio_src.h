
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
 * Definitions for WICED HCI Audio
 */

#ifndef WICED_HCI_AUDIO_H
#define WICED_HCI_AUDIO_H

#include "wiced_hci.h"

#define AUDIO_SRC_ROUTE_I2S         0
#define AUDIO_SRC_ROUTE_UART        1
#define AUDIO_SRC_ROUTE_SINE_WAVE   2

typedef struct
{
    unsigned char bda[BDA_LEN];
    unsigned char audio_route;
} wiced_hci_bt_audio_source_connect_data_t;

typedef wiced_hci_bt_handle_t wiced_hci_bt_audio_source_disconnect_data_t ;

typedef wiced_hci_bt_handle_t wiced_hci_bt_audio_source_stop_data_t ;

#define AUDIO_SRC_SAMPLING_FREQ_16      0
#define AUDIO_SRC_SAMPLING_FREQ_32      1
#define AUDIO_SRC_SAMPLING_FREQ_44_1    2
#define AUDIO_SRC_SAMPLING_FREQ_48      3

#define AUDIO_SRC_CHANNEL_MODE_MONO     0
#define AUDIO_SRC_CHANNEL_MODE_STEREO   1

typedef struct
{
    uint16_t handle;
    uint8_t sample_freq;
    uint8_t audio_mode;
} wiced_hci_bt_audio_source_start_data_t;

typedef struct
{
    uint8_t *p_data;
    uint16_t len;
} wiced_hci_bt_audio_source_audio_data_t;

#define AUDIO_SRC_AUDIO_DATA_FORMAT_PCM 0
#define AUDIO_SRC_AUDIO_DATA_FORMAT_MP3 1

typedef struct
{
    uint8_t format;
} wiced_hci_bt_audio_source_audio_data_format_t;

#define AUDIO_SRC_FEATURE_I2S_INPUT (1 << 0)
#define AUDIO_SRC_FEATURE_MP3_FORMAT (1 << 1)

bool wiced_hci_audio_src_connect(wiced_hci_bt_audio_source_connect_data_t *p_data);
bool wiced_hci_audio_src_disconnect(wiced_hci_bt_audio_source_disconnect_data_t *p_data);
bool wiced_hci_audio_src_audio_start(wiced_hci_bt_audio_source_start_data_t *p_data);
bool wiced_hci_audio_src_audio_stop(wiced_hci_bt_audio_source_stop_data_t *p_data);
bool wiced_hci_audio_src_audio_data(wiced_hci_bt_audio_source_audio_data_t *p_data);
bool wiced_hci_audio_src_audio_data_format(wiced_hci_bt_audio_source_audio_data_format_t *p_data);

#endif
