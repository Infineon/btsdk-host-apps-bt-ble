#-------------------------------------------------
#
# Project created by QtCreator 2016-11-07T14:38:11
#
#-------------------------------------------------

QT += core gui
QT += serialport
QT += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ClientControl
TEMPLATE = app

win32|win64 {
    DESTDIR = $$PWD/../Windows
}

macx {
    DESTDIR = $$PWD/../OSX
}

SOURCES += main.cpp\
    audio_src_dual_a2dp.cpp \
        mainwindow.cpp \
        audio_src.cpp \
        device_manager.cpp \
        handsfree.cpp \
        pbap_client.cpp \
        spp.cpp \
        audio_gateway.cpp \
        hid_device.cpp \
        hid_host.cpp \
        home_kit.cpp \
        avrc_tg.cpp \
        avrc_ct.cpp \
        iap2.cpp \
        fw_download.cpp \
        gatt.cpp \
        audio_snk.cpp \
        bt_serial_gatt.cpp \
        battery_client.cpp \
        findme_locator.cpp \
        demo.cpp \
        opp_server.cpp \
    gatt_db.cpp \
    ../../lib_host/lib_app/app_host.c \
    ../../lib_host/lib_app/app_host_ag.c \
    ../../lib_host/lib_wiced_hci/wiced_hci.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_ag.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_audio_src.c \
	../../lib_host/lib_app/app_host_spp.c \
    ../../lib_host/lib_app/app_host_hidh.c \
    ../../lib_host/lib_app/app_host_hidd.c \
    ../../lib_host/lib_app/app_host_hf.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_spp.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_hidh.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_hidd.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_hf.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_gatt.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_dm.c \
    ../../lib_host/lib_app/app_host_audio_src.c \
    anp.cpp \
    ../../lib_host/lib_app/app_host_anp.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_anp.c \
    le_coc.cpp \
    led_demo.cpp \
    ../../lib_host/lib_app/app_host_le_coc.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_le_coc.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_gatt_db.c \
    otp_client.cpp \
    ../../lib_host/lib_app/app_host_otp_client.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_otp_client.c \
    ../../lib_host/lib_wiced_hci/wiced_hci_avrc_ct.c \
    ../../lib_host/lib_app/app_host_avrc_ct.c \
    ../../lib_host/lib_app/app_host_dm.c \
    ../../lib_host/lib_app/app_host_gatt.c \
    nanopb/pb_common.c \
    nanopb/pb_decode.c \
    nanopb/pb_encode.c \
    nanopb/rpc.pb.c \
    nanopb/wiced_hci_gatt_db.pb.c \
    nanopb/wiced_hci_spp.pb.c \
    nanopb/wiced_hci_gatt_db_rpc.c \
    nanopb/wiced_hci_spp_rpc.c \
    nanopb/protobuf_rpc.c \
    map_client.cpp

unix {
    SOURCES += serial_port_linux.cpp
    SOURCES += btspy_ux.cpp
}

win32|win64 {
    SOURCES += btspy_win32.cpp
    SOURCES += serial_port_win32.cpp
    SOURCES += kp3_uart_wa_lib/kp3uart_workaround.cpp
}


HEADERS  += mainwindow.h \
            avrc.h \
            serial_port.h \
            app_include.h \
    usb_kb_usage.h \
    wiced_bt_defs.h \
    btle_homekit2_lightbulb.h \
    include/hci_control_api.h \
    ../../lib_host/lib_app/app_host.h \
    ../../lib_host/lib_wiced_hci/wiced_hci.h \
    ../../lib_host/lib_wiced_hci/wiced_types.h \
    ../../lib_host/lib_app/app_host_spp.h \
    ../../lib_host/lib_app/app_host_otp_client.h \
    ../../lib_host/lib_app/app_host_le_coc.h \
    ../../lib_host/lib_app/app_host_hidh.h \
    ../../lib_host/lib_app/app_host_hidd.h \
    ../../lib_host/lib_app/app_host_hf.h \
    ../../lib_host/lib_app/app_host_gatt.h \
    ../../lib_host/lib_app/app_host_dm.h \
    ../../lib_host/lib_app/app_host_avrc.h \
    ../../lib_host/lib_app/app_host_audio_src.h \
    ../../lib_host/lib_app/app_host_ans_anc.h \
    ../../lib_host/lib_app/app_host_ag.h \
    ../../lib_host/lib_app/app_host.h \
    ../../lib_host/lib_wiced_hci/wiced_types.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_spp.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_otp.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_le_coc.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_hidh.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_hidd.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_hf.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_gatt_db.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_gatt.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_dm.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_avrc_ct.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_audio_src.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_anp_ans.h \
    ../../lib_host/lib_wiced_hci/wiced_hci_ag.h \
    ../../lib_host/lib_wiced_hci/wiced_hci.h \
    nanopb/pb.h \
    nanopb/pb_common.h \
    nanopb/pb_decode.h \
    nanopb/pb_encode.h \
    nanopb/rpc.pb.h \
    nanopb/wiced_hci_gatt_db.pb.h \
    protobuf_rpc.h

FORMS    += mainwindow.ui

INCLUDEPATH += common/include
INCLUDEPATH += ../../include
INCLUDEPATH += ../../../../dev-kit/btsdk-include
INCLUDEPATH += ../../lib_host/lib_app
INCLUDEPATH += ../../lib_host/lib_wiced_hci
INCLUDEPATH += ./nanopb
INCLUDEPATH += ./kp3_uart_wa_lib

# ws2_32.lib and winmm.lib path might need to be adjusted on user PC, for example
# C:\WINDDK\7600.16385.0\lib\win7\i386\ws2_32.lib
# -L"Windows" -lws2_32
win32: LIBS += -lQt5Network ..\Windows\ws2_32.lib ..\Windows\winmm.lib -lsetupapi

unix:!macx {
LIBS += -lasound -lrt
DEFINES += PCM_ALSA
}

RESOURCES     = resources.qrc

RC_ICONS = CY_Logo.ico

ICON = CY_Logo.icns

DISTFILES += \
    ../README.txt
