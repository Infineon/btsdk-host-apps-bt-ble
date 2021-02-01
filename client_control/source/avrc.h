
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
 * Definitions for AVRC profile
 */

#ifndef AVRC_H
#define AVRC_H

/* Media Attribute types
*/
#define APP_AVRC_MEDIA_ATTR_ID_TITLE                 0x01
#define APP_AVRC_MEDIA_ATTR_ID_ARTIST                0x02
#define APP_AVRC_MEDIA_ATTR_ID_ALBUM                 0x03
#define APP_AVRC_MEDIA_ATTR_ID_TRACK_NUM             0x04
#define APP_AVRC_MEDIA_ATTR_ID_NUM_TRACKS            0x05
#define APP_AVRC_MEDIA_ATTR_ID_GENRE                 0x06
#define APP_AVRC_MEDIA_ATTR_ID_PLAYING_TIME          0x07        /* in miliseconds */
#define APP_AVRC_MAX_NUM_MEDIA_ATTR_ID               7


/* media name definition struct */
typedef struct
{
    /* The media name, name length and character set id. */
    uint16_t             str_len;     /* String len, 255 max */
    uint8_t              *p_str;     /* String value */
} tAPP_AVRC_NAME;

/* Media attribute */
typedef struct
{
    uint8_t             attr_id;        /* Media type   AVRC_MEDIA_ATTR_ID_TITLE, AVRC_MEDIA_ATTR_ID_ARTIST, AVRC_MEDIA_ATTR_ID_ALBUM,
                                                        AVRC_MEDIA_ATTR_ID_TRACK_NUM, AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
                                                        AVRC_MEDIA_ATTR_ID_GENRE, AVRC_MEDIA_ATTR_ID_PLAYING_TIME */
    tAPP_AVRC_NAME      name;           /* Media name  */
} tAPP_AVRC_ATTR_ENTRY;

/* Media (song) info */
typedef struct
{
    uint8_t                   attr_count;     /* The number of attributes in p_attr_list */
    tAPP_AVRC_ATTR_ENTRY      p_attr_list[7]; /* Attribute entry list. */
} tAPP_AVRC_ITEM_MEDIA;

/* Define the Player Application Settings IDs */
#define APP_AVRC_MAX_APP_ATTR_SIZE                  16
#define APP_AVRC_PLAYER_SETTING_REPEAT              0x02
#define APP_AVRC_PLAYER_SETTING_SHUFFLE             0x03

/* Define the possible values of the Player Application Settings */
#define APP_AVRC_PLAYER_VAL_OFF                     0x01
#define APP_AVRC_PLAYER_VAL_ON                      0x02
#define APP_AVRC_PLAYER_VAL_SINGLE_REPEAT           0x02
#define APP_AVRC_PLAYER_VAL_ALL_REPEAT              0x03
#define APP_AVRC_PLAYER_VAL_GROUP_REPEAT            0x04
#define APP_AVRC_PLAYER_VAL_ALL_SHUFFLE             0x02
#define APP_AVRC_PLAYER_VAL_GROUP_SHUFFLE           0x03
#define APP_AVRC_PLAYER_VAL_ALL_SCAN                0x02
#define APP_AVRC_PLAYER_VAL_GROUP_SCAN              0x03


/* Player Attribute value definition struct */
typedef struct {
    uint8_t                 attr_id; /* attribute type (repeat or shuffle) */
    uint8_t                 num_val; /* number of values possible */
    uint8_t                 vals[APP_AVRC_MAX_APP_ATTR_SIZE]; /* array of values */
    uint8_t                 curr_value; /*current value set by user */
} tAPP_AVRC_META_ATTRIB;

/* Play state definitions */
#define APP_AVRC_PLAYSTATE_STOPPED                  0x00    /* Stopped */
#define APP_AVRC_PLAYSTATE_PLAYING                  0x01    /* Playing */
#define APP_AVRC_PLAYSTATE_PAUSED                   0x02    /* Paused  */


/*********************************************************************/
/* Application defined sample data */
/*********************************************************************/
#define APP_AVRC_NUMSONGS                           3
#define APP_AVRC_ATTR_SONG_LEN                      255000
#define APP_AVRC_ATTR_SONG_POS                      120000


#endif // AVRC_H
