#-------------------------------------------------
#
# Project created by QtCreator 2016-11-07T14:38:11
#
#-------------------------------------------------

#
# Description: KP3 UART workaround
#
# This static library contains a workaround for correct
# handling UART RTS/DTR control signals via usbser driver on Windows.
# This workaround is required for KitProg3 (KP3) UART HCI devices
# to work correctly with ClientControl and CyBluetool applications.
#
# 1. https://jira.cypress.com/browse/BTSDK-4891
#    KP3_RTS (BT_UART_CTS) stays high when Clientcontrol com port is enabled
# 2. https://jira.cypress.com/browse/CYBLUETOOL-369
#    KP3_RTS (BT_UART_CTS) stays high when Bluetool com port is enabled
#
QT -= gui
QT += core

TEMPLATE = lib
CONFIG += staticlib

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    kp3uart_workaround.cpp

HEADERS += \
    kp3uart_workaround.h
