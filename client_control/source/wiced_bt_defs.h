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
 * WICED related constants and types copied from WICED header files used in the Client Control code
 */

#ifndef __WICED_BT_DEFS_H__
#define __WICED_BT_DEFS_H__


// Marcos
#define ARRAY_TO_STREAM(p, a, len) {register int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

#define BIT16_TO_8( val ) \
    (uint8_t)(  (val)        & 0xff),/* LSB */ \
    (uint8_t)(( (val) >> 8 ) & 0xff) /* MSB */


// UUID length
#define LEN_UUID_16     2
#define LEN_UUID_128    16

#define MAX_UUID_SIZE              16  /**< Maximum UUID size - 16 bytes, and structure to hold any type of UUID. */

/** UUID Type */
typedef struct
{
#define LEN_UUID_32     4

    uint16_t        len;     /**< UUID length */

    union
    {
        uint16_t    uuid16; /**< 16-bit UUID */
        uint32_t    uuid32; /**< 32-bit UUID */
        uint8_t     uuid128[MAX_UUID_SIZE]; /**< 128-bit UUID */
    } uu;

} wiced_bt_uuid_t;


//  The permission bits for characteristic/descriptor
#define LEGATTDB_PERM_NONE                      (0x00)
#define LEGATTDB_PERM_VARIABLE_LENGTH           (0x1 << 0)
#define LEGATTDB_PERM_READABLE                  (0x1 << 1)
#define LEGATTDB_PERM_WRITE_CMD                 (0x1 << 2)
#define LEGATTDB_PERM_WRITE_REQ                 (0x1 << 3)
#define LEGATTDB_PERM_AUTH_READABLE             (0x1 << 4)
#define LEGATTDB_PERM_RELIABLE_WRITE            (0x1 << 5)
#define LEGATTDB_PERM_AUTH_WRITABLE             (0x1 << 6)

#define LEGATTDB_PERM_WRITABLE  (LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ| LEGATTDB_PERM_AUTH_WRITABLE)
#define LEGATTDB_PERM_MASK                      (0x7F)   /* All the permission bits. */
#define LEGATTDB_PERM_SERVICE_UUID_128          (0x1 << 7)


//  GATT Characteristic Property bits
#define LEGATTDB_CHAR_PROP_BROADCAST            (0x1 << 0)
#define LEGATTDB_CHAR_PROP_READ                 (0x1 << 1)
#define LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE    (0x1 << 2)
#define LEGATTDB_CHAR_PROP_WRITE                (0x1 << 3)
#define LEGATTDB_CHAR_PROP_NOTIFY               (0x1 << 4)
#define LEGATTDB_CHAR_PROP_INDICATE             (0x1 << 5)
#define LEGATTDB_CHAR_PROP_AUTHD_WRITES         (0x1 << 6)
#define LEGATTDB_CHAR_PROP_EXTENDED             (0x1 << 7)


// GATT appearance definitions
enum
{
    APPEARANCE_GENERIC_TAG = 512
};


// Standard 16-bit UUIDs for characteristics
enum
{
    UUID_CHARACTERISTIC_DEVICE_NAME                                         = 0x2A00,
    UUID_CHARACTERISTIC_APPEARANCE                                          = 0x2A01,
    UUID_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG                             = 0x2A02,
    UUID_CHARACTERISTIC_RECONNECTION_ADDRESS                                = 0x2A03,
    UUID_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS          = 0x2A04,
    UUID_CHARACTERISTIC_SERVICE_CHANGED                                     = 0x2A05,
    UUID_CHARACTERISTIC_ALERT_LEVEL                                         = 0x2A06,
    UUID_CHARACTERISTIC_TX_POWER_LEVEL                                      = 0x2A07,
    UUID_CHARACTERISTIC_DATE_TIME                                           = 0x2A08,
    UUID_CHARACTERISTIC_DAY_OF_WEEK                                         = 0x2A09,
    UUID_CHARACTERISTIC_TIME                                                = 0x2A0A,
    UUID_CHARACTERISTIC_EXACT_TIME_100                                      = 0x2A0B,
    UUID_CHARACTERISTIC_EXACT_TIME_256                                      = 0x2A0C,
    UUID_CHARACTERISTIC_DAYLIGHT_SAVING_TIME                                = 0x2A0D,
    UUID_CHARACTERISTIC_TIME_ZONE                                           = 0x2A0E,
    UUID_CHARACTERISTIC_LOCAL_TIME_INFORMATION                              = 0x2A0F,
    UUID_CHARACTERISTIC_SECONDARY_TIME_ZONE                                 = 0x2A10,
    UUID_CHARACTERISTIC_TIME_WITH_DST                                       = 0x2A11,
    UUID_CHARACTERISTIC_TIME_ACCURACY                                       = 0x2A12,
    UUID_CHARACTERISTIC_TIME_SOURCE                                         = 0x2A13,
    UUID_CHARACTERISTIC_REFERENCE_TIME_INFORMATION                          = 0x2A14,
    UUID_CHARACTERISTIC_TIME_BROADCAST                                      = 0x2A15,
    UUID_CHARACTERISTIC_TIME_UPDATE_CONTROL_POINT                           = 0x2A16,
    UUID_CHARACTERISTIC_TIME_UPDATE_STATE                                   = 0x2A17,
    UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT                                 = 0x2A18,
    UUID_CHARACTERISTIC_BATTERY_LEVEL                                       = 0x2A19,
    UUID_CHARACTERISTIC_BATTERY_POWER_STATE                                 = 0x2A1A,
    UUID_CHARACTERISTIC_BATTERY_LEVEL_STATE                                 = 0x2A1B,
    UUID_CHARACTERISTIC_TEMPERATURE_MEASUREMENT                             = 0x2A1C,
    UUID_CHARACTERISTIC_TEMPERATURE_TYPE                                    = 0x2A1D,
    UUID_CHARACTERISTIC_INTERMEDIATE_TEMPERATURE                            = 0x2A1E,
    UUID_CHARACTERISTIC_TEMPERATURE_CELSIUS                                 = 0x2A1F,
    UUID_CHARACTERISTIC_TEMPERATURE_FAHRENHEIT                              = 0x2A20,
    UUID_CHARACTERISTIC_MEASUREMENT_INTERVAL                                = 0x2A21,
    UUID_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT                          = 0x2A22,
    UUID_CHARACTERISTIC_SYSTEM_ID                                           = 0x2A23,
    UUID_CHARACTERISTIC_MODEL_NUMBER_STRING                                 = 0x2A24,
    UUID_CHARACTERISTIC_SERIAL_NUMBER_STRING                                = 0x2A25,
    UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING                            = 0x2A26,
    UUID_CHARACTERISTIC_HARDWARE_REVISION_STRING                            = 0x2A27,
    UUID_CHARACTERISTIC_SOFTWARE_REVISION_STRING                            = 0x2A28,
    UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING                            = 0x2A29,
    UUID_CHARACTERISTIC_IEEE_11073_20601_REGULATORY_CERTIFICATION_DATA_LIST = 0x2A2A,
    UUID_CHARACTERISTIC_CURRENT_TIME                                        = 0x2A2B,
    UUID_CHARACTERISTIC_MAGNETIC_DECLINATION                                = 0x2A2C,
    UUID_CHARACTERISTIC_SCAN_REFRESH                                        = 0x2A31,
    UUID_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT                         = 0x2A32,
    UUID_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT                             = 0x2A33,
    UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT                         = 0x2A34,
    UUID_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT                          = 0x2A35,
    UUID_CHARACTERISTIC_INTERMEDIATE_BLOOD_PRESSURE                         = 0x2A36,
    UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT                              = 0x2A37,
    UUID_CHARACTERISTIC_HEART_RATE_SENSOR_LOCATION                          = 0x2A38,
    UUID_CHARACTERISTIC_HEART_RATE_CONTROL_POINT                            = 0x2A39,
    UUID_CHARACTERISTIC_REMOVABLE                                           = 0x2A3A,
    UUID_CHARACTERISTIC_SERVICE_REQUIRED                                    = 0x2A3B,
    UUID_CHARACTERISTIC_SCIENTIFIC_TEMPERATURE_CELSIUS                      = 0x2A3C,
    UUID_CHARACTERISTIC_STRING                                              = 0x2A3D,
    UUID_CHARACTERISTIC_NETWORK_AVAILABILITY                                = 0x2A3E,
    UUID_CHARACTERISTIC_ALERT_STATUS                                        = 0x2A3F,
    UUID_CHARACTERISTIC_RINGER_CONTROL_POINT                                = 0x2A40,
    UUID_CHARACTERISTIC_RINGER_SETTING                                      = 0x2A41,
    UUID_CHARACTERISTIC_ALERT_CATEGORY_ID_BIT_MASK                          = 0x2A42,
    UUID_CHARACTERISTIC_ALERT_CATEGORY_ID                                   = 0x2A43,
    UUID_CHARACTERISTIC_ALERT_NOTIFICATION_CONTROL_POINT                    = 0x2A44,
    UUID_CHARACTERISTIC_UNREAD_ALERT_STATUS                                 = 0x2A45,
    UUID_CHARACTERISTIC_NEW_ALERT                                           = 0x2A46,
    UUID_CHARACTERISTIC_SUPPORTED_NEW_ALERT_CATEGORY                        = 0x2A47,
    UUID_CHARACTERISTIC_SUPPORTED_UNREAD_ALERT_CATEGORY                     = 0x2A48,
    UUID_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE                              = 0x2A49,
    UUID_CHARACTERISTIC_HID_INFORMATION                                     = 0x2A4A,
    UUID_CHARACTERISTIC_HID_REPORT_MAP                                      = 0x2A4B,
    UUID_CHARACTERISTIC_REPORT_MAP                                          = 0x2A4B,
    UUID_CHARACTERISTIC_HID_CONTROL_POINT                                   = 0x2A4C,
    UUID_CHARACTERISTIC_HID_REPORT                                          = 0x2A4D,
    UUID_CHARACTERISTIC_REPORT                                              = 0x2A4D,
    UUID_CHARACTERISTIC_HID_PROTOCOL_MODE                                   = 0x2A4E,
    UUID_CHARACTERISTIC_PROTOCOL_MODE                                       = 0x2A4E,
    UUID_CHARACTERISTIC_SCAN_INTERVAL_WINDOW                                = 0x2A4F,
    UUID_CHARACTERISTIC_PNP_ID                                              = 0x2A50,
    UUID_CHARACTERISTIC_GLUCOSE_FEATURES                                    = 0x2A51,
    UUID_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT                         = 0x2A52,
    UUID_CHARACTERISTIC_RSC_MEASUREMENT                                     = 0x2A53,
    UUID_CHARACTERISTIC_RSC_FEATURE                                         = 0x2A54,
    UUID_CHARACTERISTIC_SC_CONTROL_POINT                                    = 0x2A55,
    UUID_CHARACTERISTIC_RSC_CONTROL_POINT                                   = 0x2A55,
    UUID_CHARACTERISTIC_CSC_CONTROL_POINT                                   = 0x2A55,
    UUID_CHARACTERISTIC_DIGITAL                                             = 0x2A56,
    UUID_CHARACTERISTIC_ANALOG                                              = 0x2A58,
    UUID_CHARACTERISTIC_AGGREGATE_INPUT                                     = 0x2A5A,
    UUID_CHARACTERISTIC_CSC_MEASUREMENT                                     = 0x2A5B,
    UUID_CHARACTERISTIC_CSC_FEATURE                                         = 0x2A5C,
    UUID_CHARACTERISTIC_SENSOR_LOCATION                                     = 0x2A5D,
    UUID_CHARACTERISTIC_PLX_SPOT_CHECK_MEASUREMENT                          = 0x2A5E,
    UUID_CHARACTERISTIC_PLX_CONTINUOUS_MEASUREMENT                          = 0x2A5F,
    UUID_CHARACTERISTIC_PLX_FEATURES                                        = 0x2A60,
    UUID_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT                           = 0x2A63,
    UUID_CHARACTERISTIC_CYCLING_POWER_VECTOR                                = 0x2A64,
    UUID_CHARACTERISTIC_CYCLING_POWER_FEATURE                               = 0x2A65,
    UUID_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT                         = 0x2A66,
    UUID_CHARACTERISTIC_LOCATION_AND_SPEED                                  = 0x2A67,
    UUID_CHARACTERISTIC_NAVIGATION                                          = 0x2A68,
    UUID_CHARACTERISTIC_POSITION_QUALITY                                    = 0x2A69,
    UUID_CHARACTERISTIC_LN_FEATURE                                          = 0x2A6A,
    UUID_CHARACTERISTIC_LN_CONTROL_POINT                                    = 0x2A6B,
    UUID_CHARACTERISTIC_ELEVATION                                           = 0x2A6C,
    UUID_CHARACTERISTIC_PRESSURE                                            = 0x2A6D,
    UUID_CHARACTERISTIC_TEMPERATURE                                         = 0x2A6E,
    UUID_CHARACTERISTIC_HUMIDITY                                            = 0x2A6F,
    UUID_CHARACTERISTIC_TRUE_WIND_SPEED                                     = 0x2A70,
    UUID_CHARACTERISTIC_TRUE_WIND_DIRECTION                                 = 0x2A71,
    UUID_CHARACTERISTIC_APPARENT_WIND_SPEED                                 = 0x2A72,
    UUID_CHARACTERISTIC_APPARENT_WIND_DIRECTION                             = 0x2A73,
    UUID_CHARACTERISTIC_GUST_FACTOR                                         = 0x2A74,
    UUID_CHARACTERISTIC_POLLEN_CONCENTRATION                                = 0x2A75,
    UUID_CHARACTERISTIC_UV_INDEX                                            = 0x2A76,
    UUID_CHARACTERISTIC_IRRADIANCE                                          = 0x2A77,
    UUID_CHARACTERISTIC_RAINFALL                                            = 0x2A78,
    UUID_CHARACTERISTIC_WIND_CHILL                                          = 0x2A79,
    UUID_CHARACTERISTIC_HEAT_INDEX                                          = 0x2A7A,
    UUID_CHARACTERISTIC_DEW_POINT                                           = 0x2A7B,
    UUID_CHARACTERISTIC_DESCRIPTOR_VALUE_CHANGED                            = 0x2A7D,
    UUID_CHARACTERISTIC_AEROBIC_HEART_RATE_LOWER_LIMIT                      = 0x2A7E,
    UUID_CHARACTERISTIC_AEROBIC_THRESHOLD                                   = 0x2A7F,
    UUID_CHARACTERISTIC_AGE                                                 = 0x2A80,
    UUID_CHARACTERISTIC_ANAEROBIC_HEART_RATE_LOWER_LIMIT                    = 0x2A81,
    UUID_CHARACTERISTIC_ANAEROBIC_HEART_RATE_UPPER_LIMIT                    = 0x2A82,
    UUID_CHARACTERISTIC_ANAEROBIC_THRESHOLD                                 = 0x2A83,
    UUID_CHARACTERISTIC_AEROBIC_HEART_RATE_UPPER_LIMIT                      = 0x2A84,
    UUID_CHARACTERISTIC_DATE_OF_BIRTH                                       = 0x2A85,
    UUID_CHARACTERISTIC_DATE_OF_THRESHOLD_ASSESSMENT                        = 0x2A86,
    UUID_CHARACTERISTIC_EMAIL_ADDRESS                                       = 0x2A87,
    UUID_CHARACTERISTIC_FAT_BURN_HEART_RATE_LOWER_LIMIT                     = 0x2A88,
    UUID_CHARACTERISTIC_FAT_BURN_HEART_RATE_UPPER_LIMIT                     = 0x2A89,
    UUID_CHARACTERISTIC_FIRST_NAME                                          = 0x2A8A,
    UUID_CHARACTERISTIC_FIVE_ZONE_HEART_RATE_LIMITS                         = 0x2A8B,
    UUID_CHARACTERISTIC_GENDER                                              = 0x2A8C,
    UUID_CHARACTERISTIC_HEART_RATE_MAX                                      = 0x2A8D,
    UUID_CHARACTERISTIC_HEIGHT                                              = 0x2A8E,
    UUID_CHARACTERISTIC_HIP_CIRCUMFERENCE                                   = 0x2A8F,
    UUID_CHARACTERISTIC_LAST_NAME                                           = 0x2A90,
    UUID_CHARACTERISTIC_MAXIMUM_RECOMMENDED_HEART_RATE                      = 0x2A91,
    UUID_CHARACTERISTIC_RESTING_HEART_RATE                                  = 0x2A92,
    UUID_CHARACTERISTIC_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS     = 0x2A93,
    UUID_CHARACTERISTIC_THREE_ZONE_HEART_RATE_LIMITS                        = 0x2A94,
    UUID_CHARACTERISTIC_TWO_ZONE_HEART_RATE_LIMITS                          = 0x2A95,
    UUID_CHARACTERISTIC_VO2_MAX                                             = 0x2A96,
    UUID_CHARACTERISTIC_WAIST_CIRCUMFERENCE                                 = 0x2A97,
    UUID_CHARACTERISTIC_WEIGHT                                              = 0x2A98,
    UUID_CHARACTERISTIC_DATABASE_CHANGE_INCREMENT                           = 0x2A99,
    UUID_CHARACTERISTIC_USER_INDEX                                          = 0x2A9A,
    UUID_CHARACTERISTIC_BODY_COMPOSITION_FEATURE                            = 0x2A9B,
    UUID_CHARACTERISTIC_BODY_COMPOSITION_MEASUREMENT                        = 0x2A9C,
    UUID_CHARACTERISTIC_WEIGHT_MEASUREMENT                                  = 0x2A9D,
    UUID_CHARACTERISTIC_WEIGHT_SCALE_FEATURE                                = 0x2A9E,
    UUID_CHARACTERISTIC_USER_CONTROL_POINT                                  = 0x2A9F,
    UUID_CHARACTERISTIC_MAGNETIC_FLUX_DENSITY_2D                            = 0x2AA0,
    UUID_CHARACTERISTIC_MAGNETIC_FLUX_DENSITY_3D                            = 0x2AA1,
    UUID_CHARACTERISTIC_LANGUAGE                                            = 0x2AA2,
    UUID_CHARACTERISTIC_BAROMETRIC_PRESSURE_TREND                           = 0x2AA3,
    UUID_CHARACTERISTIC_BOND_MANAGEMENT_CONTROL_POINT                       = 0x2AA4,
    UUID_CHARACTERISTIC_BOND_MANAGEMENT_FEATURE                             = 0x2AA5,
    UUID_CHARACTERISTIC_CENTRAL_ADDRESS_RESOLUTION                          = 0x2AA6,
    UUID_CHARACTERISTIC_CGM_MEASUREMENT                                     = 0x2AA7,
    UUID_CHARACTERISTIC_CGM_FEATURE                                         = 0x2AA8,
    UUID_CHARACTERISTIC_CGM_STATUS                                          = 0x2AA9,
    UUID_CHARACTERISTIC_CGM_SESSION_START_TIME                              = 0x2AAA,
    UUID_CHARACTERISTIC_CGM_SESSION_RUN_TIME                                = 0x2AAB,
    UUID_CHARACTERISTIC_CGM_SPECIFIC_OPS_CONTROL_POINT                      = 0x2AAC,
    UUID_CHARACTERISTIC_INDOOR_POSITIONING_CONFIGURATION                    = 0x2AAD,
    UUID_CHARACTERISTIC_LATITUDE                                            = 0x2AAE,
    UUID_CHARACTERISTIC_LONGITUDE                                           = 0x2AAF,
    UUID_CHARACTERISTIC_LOCAL_NORTH_COORDINATE                              = 0x2AB0,
    UUID_CHARACTERISTIC_LOCAL_EAST_COORDINATE                               = 0x2AB1,
    UUID_CHARACTERISTIC_FLOOR_NUMBER                                        = 0x2AB2,
    UUID_CHARACTERISTIC_ALTITUDE                                            = 0x2AB3,
    UUID_CHARACTERISTIC_UNCERTAINTY                                         = 0x2AB4,
    UUID_CHARACTERISTIC_LOCATION_NAME                                       = 0x2AB5,
    UUID_CHARACTERISTIC_URI                                                 = 0x2AB6,
    UUID_CHARACTERISTIC_HTTP_HEADERS                                        = 0x2AB7,
    UUID_CHARACTERISTIC_HTTP_STATUS_CODE                                    = 0x2AB8,
    UUID_CHARACTERISTIC_HTTP_ENTITY_BODY                                    = 0x2AB9,
    UUID_CHARACTERISTIC_HTTP_CONTROL_POINT                                  = 0x2ABA,
    UUID_CHARACTERISTIC_HTTPS_SECURITY                                      = 0x2ABB,
    UUID_CHARACTERISTIC_TDS_CONTROL_POINT                                   = 0x2ABC,
    UUID_CHARACTERISTIC_OTS_FEATURE                                         = 0x2ABD,
    UUID_CHARACTERISTIC_OBJECT_NAME                                         = 0x2ABE,
    UUID_CHARACTERISTIC_OBJECT_TYPE                                         = 0x2ABF,
    UUID_CHARACTERISTIC_OBJECT_SIZE                                         = 0x2AC0,
    UUID_CHARACTERISTIC_OBJECT_FIRST_CREATED                                = 0x2AC1,
    UUID_CHARACTERISTIC_OBJECT_LAST_MODIFIED                                = 0x2AC2,
    UUID_CHARACTERISTIC_OBJECT_ID                                           = 0x2AC3,
    UUID_CHARACTERISTIC_OBJECT_PROPERTIES                                   = 0x2AC4,
    UUID_CHARACTERISTIC_OBJECT_ACTION_CONTROL_POINT                         = 0x2AC5,
    UUID_CHARACTERISTIC_OBJECT_LIST_CONTROL_POINT                           = 0x2AC6,
    UUID_CHARACTERISTIC_OBJECT_LIST_FILTER                                  = 0x2AC7,
    UUID_CHARACTERISTIC_OBJECT_CHANGED                                      = 0x2AC8,
};


// Standard 16-bit UUIDs for descriptors
enum
{
    UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES  = 0x2900,
    UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION     = 0x2901,
    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION = 0x2902,
    UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION = 0x2903,
    UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT  = 0x2904,
    UUID_DESCRIPTOR_CHARACTERISTIC_AGGREGATE_FORMAT     = 0x2905,
    UUID_DESCRIPTOR_VALID_RANGE                         = 0x2906,
    UUID_DESCRIPTOR_EXTERNAL_REPORT_REFERENCE           = 0x2907,
    UUID_DESCRIPTOR_REPORT_REFERENCE                    = 0x2908,
    UUID_DESCRIPTOR_NUMBER_OF_DIGITALS                  = 0x2909,
    UUID_DESCRIPTOR_VALUE_TRIGGER_SETTING               = 0x290A,
    UUID_DESCRIPTOR_ENVIRONMENT_SENSING_CONFIGURATION   = 0x290B,
    UUID_DESCRIPTOR_ENVIRONMENT_SENSING_MEASUREMENT     = 0x290C,
    UUID_DESCRIPTOR_ENVIRONMENT_SENSING_TRIGGER_SETTING = 0x290D,
    UUID_DESCRIPTOR_TIME_TRIGGER_SETTING                = 0x290E,
};


// BLE Advertisement data types
enum wiced_bt_ble_advert_type_e {
    BTM_BLE_ADVERT_TYPE_FLAG                     = 0x01, /* Advertisement flags */
    BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL            = 0x02, /* List of supported services - 16 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE           = 0x03, /* List of supported services - 16 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_32SRV_PARTIAL            = 0x04, /* List of supported services - 32 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_32SRV_COMPLETE           = 0x05, /* List of supported services - 32 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_128SRV_PARTIAL           = 0x06, /* List of supported services - 128 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE          = 0x07, /* List of supported services - 128 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_NAME_SHORT               = 0x08, /* Short name */
    BTM_BLE_ADVERT_TYPE_NAME_COMPLETE            = 0x09, /* Complete name */
    BTM_BLE_ADVERT_TYPE_TX_POWER                 = 0x0A, /* TX Power level  */
    BTM_BLE_ADVERT_TYPE_DEV_CLASS                = 0x0D, /* Device Class */
    BTM_BLE_ADVERT_TYPE_SM_TK                    = 0x10, /* Security manager TK value */
    BTM_BLE_ADVERT_TYPE_SM_OOB_FLAG              = 0x11, /* Security manager Out-of-Band data */
    BTM_BLE_ADVERT_TYPE_INTERVAL_RANGE           = 0x12, /* Slave connection interval range */
    BTM_BLE_ADVERT_TYPE_SOLICITATION_SRV_UUID    = 0x14, /* List of solicitated services - 16 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_128SOLICITATION_SRV_UUID = 0x15, /* List of solicitated services - 128 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_SERVICE_DATA             = 0x16, /* Service data - 16 bit UUID */
    BTM_BLE_ADVERT_TYPE_PUBLIC_TARGET            = 0x17, /* Public target address */
    BTM_BLE_ADVERT_TYPE_RANDOM_TARGET            = 0x18, /* Random target address */
    BTM_BLE_ADVERT_TYPE_APPEARANCE               = 0x19, /* Appearance */
    BTM_BLE_ADVERT_TYPE_ADVERT_INTERVAL          = 0x1a, /* Advertising interval */
    BTM_BLE_ADVERT_TYPE_32SOLICITATION_SRV_UUID  = 0x1b, /* List of solicitated services - 32 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_32SERVICE_DATA           = 0x1c, /* Service data - 32 bit UUID */
    BTM_BLE_ADVERT_TYPE_128SERVICE_DATA          = 0x1d, /* Service data - 128 bit UUID */
    BTM_BLE_ADVERT_TYPE_MANUFACTURER             = 0xFF  /* Manufacturer data */
};


// BLE ADV data flag bit definition used for BTM_BLE_ADVERT_TYPE_FLAG
#define BTM_BLE_LIMITED_DISCOVERABLE_FLAG           (0x01 << 0)
#define BTM_BLE_GENERAL_DISCOVERABLE_FLAG           (0x01 << 1)
#define BTM_BLE_BREDR_NOT_SUPPORTED                 (0x01 << 2)


/* GATT Status Codes */
enum wiced_bt_gatt_status_e
{
    WICED_BT_GATT_SUCCESS              = 0x00, /* Success */
    WICED_BT_GATT_INVALID_HANDLE       = 0x01, /* Invalid Handle */
    WICED_BT_GATT_READ_NOT_PERMIT      = 0x02, /* Read Not Permitted */
    WICED_BT_GATT_WRITE_NOT_PERMIT     = 0x03, /* Write Not permitted */
    WICED_BT_GATT_INVALID_PDU          = 0x04, /* Invalid PDU */
    WICED_BT_GATT_INSUF_AUTHENTICATION = 0x05, /* Insufficient Authentication */
    WICED_BT_GATT_REQ_NOT_SUPPORTED    = 0x06, /* Request Not Supported */
    WICED_BT_GATT_INVALID_OFFSET       = 0x07, /* Invalid Offset */
    WICED_BT_GATT_INSUF_AUTHORIZATION  = 0x08, /* Insufficient Authorization */
    WICED_BT_GATT_PREPARE_Q_FULL       = 0x09, /* Prepare Queue Full */
    WICED_BT_GATT_NOT_FOUND            = 0x0a, /* Not Found */
    WICED_BT_GATT_NOT_LONG             = 0x0b, /* Not Long Size */
    WICED_BT_GATT_INSUF_KEY_SIZE       = 0x0c, /* Insufficient Key Size */
    WICED_BT_GATT_INVALID_ATTR_LEN     = 0x0d, /* Invalid Attribute Length */
    WICED_BT_GATT_ERR_UNLIKELY         = 0x0e, /* Error Unlikely */
    WICED_BT_GATT_INSUF_ENCRYPTION     = 0x0f, /* Insufficient Encryption */
    WICED_BT_GATT_UNSUPPORT_GRP_TYPE   = 0x10, /* Unsupported Group Type */
    WICED_BT_GATT_INSUF_RESOURCE       = 0x11, /* Insufficient Resource */

    WICED_BT_GATT_ILLEGAL_PARAMETER    = 0x87, /* Illegal Parameter */
    WICED_BT_GATT_NO_RESOURCES         = 0x80, /* No Resources */
    WICED_BT_GATT_INTERNAL_ERROR       = 0x81, /* Internal Error */
    WICED_BT_GATT_WRONG_STATE          = 0x82, /* Wrong State */
    WICED_BT_GATT_DB_FULL              = 0x83, /* DB Full */
    WICED_BT_GATT_BUSY                 = 0x84, /* Busy */
    WICED_BT_GATT_ERROR                = 0x85, /* Error */
    WICED_BT_GATT_CMD_STARTED          = 0x86, /* Command Started */
    WICED_BT_GATT_PENDING              = 0x88, /* Pending */
    WICED_BT_GATT_AUTH_FAIL            = 0x89, /* Authentication Fail */
    WICED_BT_GATT_MORE                 = 0x8a, /* More */
    WICED_BT_GATT_INVALID_CFG          = 0x8b, /* Invalid Configuration */
    WICED_BT_GATT_SERVICE_STARTED      = 0x8c, /* Service Started */
    WICED_BT_GATT_ENCRYPED_MITM        = 0x00, /* Encrypted MITM */
    WICED_BT_GATT_ENCRYPED_NO_MITM     = 0x8d, /* Encrypted No MITM */
    WICED_BT_GATT_NOT_ENCRYPTED        = 0x8e, /* Not Encrypted */
    WICED_BT_GATT_CONGESTED            = 0x8f, /* Congested */

                                               /* 0xE0 ~ 0xFC reserved for future use */
    WICED_BT_GATT_CCC_CFG_ERR          = 0xFD, /* Improper Client Char Configuration */
    WICED_BT_GATT_PRC_IN_PROGRESS      = 0xFE, /* Procedure Already in Progress */
    WICED_BT_GATT_OUT_OF_RANGE         = 0xFF, /* Value Out of Range */
};

#endif // __WICED_BT_DEFS_H__
