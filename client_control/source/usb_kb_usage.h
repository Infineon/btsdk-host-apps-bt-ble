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
 * USB usage defines
 */

#ifndef USB_KB_USAGE_H
#define USB_KB_USAGE_H

/// KB USB usages
enum UsbUsage
{
    USB_USAGE_NO_EVENT=0,
    USB_USAGE_ROLLOVER=1,
    USB_USAGE_POST_FAIL=2,
    USB_USAGE_UNDEFINED_ERROR=3,

    USB_USAGE_A=4,
    USB_USAGE_B=5,
    USB_USAGE_C=6,
    USB_USAGE_D=7,
    USB_USAGE_E=8,
    USB_USAGE_F=9,
    USB_USAGE_G=10,
    USB_USAGE_H=11,
    USB_USAGE_I=12,
    USB_USAGE_J=13,
    USB_USAGE_K=14,
    USB_USAGE_L=15,
    USB_USAGE_M=16,
    USB_USAGE_N=17,
    USB_USAGE_O=18,
    USB_USAGE_P=19,
    USB_USAGE_Q=20,
    USB_USAGE_R=21,
    USB_USAGE_S=22,
    USB_USAGE_T=23,
    USB_USAGE_U=24,
    USB_USAGE_V=25,
    USB_USAGE_W=26,
    USB_USAGE_X=27,
    USB_USAGE_Y=28,
    USB_USAGE_Z=29,

    USB_USAGE_1=30,
    USB_USAGE_2=31,
    USB_USAGE_3=32,
    USB_USAGE_4=33,
    USB_USAGE_5=34,
    USB_USAGE_6=35,
    USB_USAGE_7=36,
    USB_USAGE_8=37,
    USB_USAGE_9=38,
    USB_USAGE_0=39,

    USB_USAGE_ENTER=40,
    USB_USAGE_ESCAPE=41,
    USB_USAGE_BACKSPACE=42,
    USB_USAGE_TAB=43,
    USB_USAGE_SPACEBAR=44,
    USB_USAGE_MINUS=45,
    USB_USAGE_EQUAL=46,
    USB_USAGE_LEFT_BRACKET=47,
    USB_USAGE_RIGHT_BRACKET=48,
    USB_USAGE_BACK_SLASH=49,

    USB_USAGE_NON_US_HASH=50,
    USB_USAGE_SEMICOLON=51,
    USB_USAGE_QUOTE=52,
    USB_USAGE_ACCENT=53,
    USB_USAGE_COMMA=54,
    USB_USAGE_STOP_AND_GREATER=55,
    USB_USAGE_SLASH=56,
    USB_USAGE_CAPS_LOCK=57,
    USB_USAGE_F1=58,
    USB_USAGE_F2=59,

    USB_USAGE_F3=60,
    USB_USAGE_F4=61,
    USB_USAGE_F5=62,
    USB_USAGE_F6=63,
    USB_USAGE_F7=64,
    USB_USAGE_F8=65,
    USB_USAGE_F9=66,
    USB_USAGE_F10=67,
    USB_USAGE_F11=68,
    USB_USAGE_F12=69,

    USB_USAGE_PRINT_SCREEN=70,
    USB_USAGE_SCROLL_LOCK=71,
    USB_USAGE_PAUSE=72,
    USB_USAGE_INSERT=73,
    USB_USAGE_HOME=74,
    USB_USAGE_PAGE_UP=75,
    USB_USAGE_DELETE=76,
    USB_USAGE_END=77,
    USB_USAGE_PAGE_DOWN=78,
    USB_USAGE_RIGHT_ARROW=79,

    USB_USAGE_LEFT_ARROW=80,
    USB_USAGE_DOWN_ARROW=81,
    USB_USAGE_UP_ARROW=82,
    USB_USAGE_NUM_LOCK=83,
    USB_USAGE_KP_SLASH=84,
    USB_USAGE_KP_ASTERISK=85,
    USB_USAGE_KP_MINUS=86,
    USB_USAGE_KP_PLUS=87,
    USB_USAGE_KP_ENTER=88,
    USB_USAGE_KP_1=89,

    USB_USAGE_KP_2=90,
    USB_USAGE_KP_3=91,
    USB_USAGE_KP_4=92,
    USB_USAGE_KP_5=93,
    USB_USAGE_KP_6=94,
    USB_USAGE_KP_7=95,
    USB_USAGE_KP_8=96,
    USB_USAGE_KP_9=97,
    USB_USAGE_KP_0=98,
    USB_USAGE_KP_DOT=99,

    USB_USAGE_NON_US_BACK_SLASH=100,
    USB_USAGE_APPLICATION=101,
    USB_USAGE_POWER=102,
    USB_USAGE_KP_EQUAL=103,
    USB_USAGE_F13=104,
    USB_USAGE_F14=105,
    USB_USAGE_F15=106,
    USB_USAGE_F16=107,
    USB_USAGE_F17=108,
    USB_USAGE_F18=109,

    USB_USAGE_F19=110,
    USB_USAGE_F20=111,
    USB_USAGE_F21=112,
    USB_USAGE_F22=113,
    USB_USAGE_F23=114,
    USB_USAGE_F24=115,
    USB_USAGE_EXECUTE=116,
    USB_USAGE_HELP=117,
    USB_USAGE_MENU=118,
    USB_USAGE_SELECT=119,

    USB_USAGE_STOP=120,
    USB_USAGE_AGAIN=121,
    USB_USAGE_UNDO=122,
    USB_USAGE_CUT=123,
    USB_USAGE_COPY=124,
    USB_USAGE_PASTE=125,
    USB_USAGE_FIND=126,
    USB_USAGE_MUTE=127,
    USB_USAGE_VOL_UP=128,
    USB_USAGE_VOL_DOWN=129,

    USB_USAGE_LOCKING_CAPS_LOCK=130,
    USB_USAGE_LOCKING_NUM_LOCK=131,
    USB_USAGE_LOCKING_SCROLL_LOCK=132,
    USB_USAGE_KP_COMMA=133,
    USB_USAGE_KP_EQUAL_AS400=134,
    USB_USAGE_INTL_1=135,
    USB_USAGE_INTL_2=136,
    USB_USAGE_INTL_3=137,
    USB_USAGE_INTL_4=138,
    USB_USAGE_INTL_5=139,

    USB_USAGE_INTL_6=140,
    USB_USAGE_INTL_7=141,
    USB_USAGE_INTL_8=142,
    USB_USAGE_INTL_9=143,
    USB_USAGE_LANG_1=144,
    USB_USAGE_LANG_2=145,
    USB_USAGE_LANG_3=146,
    USB_USAGE_LANG_4=147,
    USB_USAGE_LANG_5=148,
    USB_USAGE_LANG_6=149,

    USB_USAGE_LANG_7=150,
    USB_USAGE_LANG_8=151,
    USB_USAGE_LANG_9=152,
    USB_USAGE_ALT_ERASE=153,
    USB_USAGE_SYS_REQ=154,
    USB_USAGE_CANCEL=155,
    USB_USAGE_CLEAR=156,
    USB_USAGE_PRIOR=157,
    USB_USAGE_RETURN=158,
    USB_USAGE_SEPARATOR=159,

    USB_USAGE_OUT=160,
    USB_USAGE_OPER=161,
    USB_USAGE_CLEAR_AGAIN=162,
    USB_USAGE_CRSEL=163,
    USB_USAGE_EXSEL=164,

    // Reserved 165-175

    USB_USAGE_KP_00=176,
    USB_USAGE_KP_000=177,
    USB_USAGE_THOUSANDS_SEPERATOR=178,
    USB_USAGE_DECIMAL_SEPERATOR=179,

    USB_USAGE_CURRENCY_UNIT=180,
    USB_USAGE_CURRENCY_SUB_UNIT=181,
    USB_USAGE_KP_LEFT_PAREN=182,
    USB_USAGE_KP_RIGHT_PAREN=183,
    USB_USAGE_KP_LEFT_CURLY_BRACE=184,
    USB_USAGE_KP_RIGHT_CURLY_BRACE=185,
    USB_USAGE_KP_TAB=186,
    USB_USAGE_KP_BACKSPACE=187,
    USB_USAGE_KP_A=188,
    USB_USAGE_KP_B=189,

    USB_USAGE_KP_C=190,
    USB_USAGE_KP_D=191,
    USB_USAGE_KP_E=192,
    USB_USAGE_KP_F=193,
    USB_USAGE_KP_XOR=194,
    USB_USAGE_KP_CARET=195,
    USB_USAGE_KP_PERCENT=196,
    USB_USAGE_KP_LESS_THAN=197,
    USB_USAGE_KP_GREATER_THAN=198,
    USB_USAGE_KP_AMPERSAND=199,

    USB_USAGE_KP_DOUBLE_AMPERSAND=200,
    USB_USAGE_KP_VERTICAL_BAR=201,
    USB_USAGE_KP_DOUBLE_VERTICAL_BAR=202,
    USB_USAGE_KP_COLON=203,
    USB_USAGE_KP_HASH=204,
    USB_USAGE_KP_SPACE=205,
    USB_USAGE_KP_AT=206,
    USB_USAGE_KP_EXCLAMATION=207,
    USB_USAGE_KP_MEM_STORE=208,
    USB_USAGE_KP_MEM_RECALL=209,

    USB_USAGE_KP_MEM_CLEAR=210,
    USB_USAGE_KP_MEM_ADD=211,
    USB_USAGE_KP_MEM_SUBTRACT=212,
    USB_USAGE_KP_MEM_MULTIPLY=213,
    USB_USAGE_KP_MEM_DIVIDE=214,
    USB_USAGE_KP_PLUS_MINUS=215,
    USB_USAGE_KP_CLEAR=216,
    USB_USAGE_KP_CLEAR_ENTRY=217,
    USB_USAGE_KP_BINARY=218,
    USB_USAGE_KP_OCTAL=219,

    USB_USAGE_KP_DECIMAL=220,
    USB_USAGE_KP_HEX=221,
    // 222-223 reserved
    USB_USAGE_LEFT_CTL=224,
    USB_USAGE_LEFT_SHIFT=225,
    USB_USAGE_LEFT_ALT=226,
    USB_USAGE_LEFT_GUI=227,
    USB_USAGE_RIGHT_CTL=228,
    USB_USAGE_RIGHT_SHIFT=229,

    USB_USAGE_RIGHT_ALT=230,
    USB_USAGE_RIGHT_GUI=231
};

/// Modifier keys bit masks
enum
{
    USB_MODKEY_MASK_LEFT_CTL=0x01,
    USB_MODKEY_MASK_LEFT_SHIFT=0x02,
    USB_MODKEY_MASK_LEFT_ALT=0x04,
    USB_MODKEY_MASK_LEFT_GUI=0x08,
    USB_MODKEY_MASK_RIGHT_CTL=0x10,
    USB_MODKEY_MASK_RIGHT_SHIFT=0x20,
    USB_MODKEY_MASK_RIGHT_ALT=0x40,
    USB_MODKEY_MASK_RIGHT_GUI=0x80
};

#endif // USB_KB_USAGE_H
