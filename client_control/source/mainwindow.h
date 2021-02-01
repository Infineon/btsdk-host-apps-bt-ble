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
 * Sample MCU application for using WICED HCI protocol. Main app header file.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdio.h>
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QComboBox>
#include <QSettings>
#include <QWaitCondition>
#include <QSemaphore>
#include <QMessageBox>
#include <QThread>
#include <QMutex>
#include "serial_port.h"
#include <QListWidget>

#ifdef PCM_ALSA
#include <alsa/asoundlib.h>
#endif

#ifdef Q_OS_WIN
// some earlier versions of the Microsoft compiler do not support snprintf()
// so we supply a custom funtion which is functionaly equivalent
int ms_snprintf ( char * s, size_t n, const char * fmt, ... );
#define snprintf(A,B,C,...) ms_snprintf (A,B,C,__VA_ARGS__)
#endif

#ifndef uint16_t
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned int   uint32_t;
#endif

typedef unsigned long    DWORD;
typedef unsigned char   BYTE;
typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef unsigned int    UINT32;
typedef unsigned char * LPBYTE;
typedef unsigned short  USHORT;
typedef int            BOOL;
typedef unsigned long  ULONG;
typedef wchar_t         WCHAR;
typedef char            CHAR;
typedef BYTE  BOOLEAN;
#define FALSE   false
#define TRUE    true
typedef DWORD ULONG;
typedef unsigned int UINT;
typedef unsigned long DWORD_PTR;

//#define UNUSED(x) (void)(x)

#define CONNECTION_TYPE_NONE    0x0000
#define CONNECTION_TYPE_AG      0x0001
#define CONNECTION_TYPE_SPP     0x0002
#define CONNECTION_TYPE_AUDIO   0x0004
#define CONNECTION_TYPE_HF      0x0008
#define CONNECTION_TYPE_HIDH    0x0010
#define CONNECTION_TYPE_IAP2    0x0020
#define CONNECTION_TYPE_LE      0x0040
#define CONNECTION_TYPE_AVRC    0x0080
#define CONNECTION_TYPE_AVK     0x0100
#define CONNECTION_TYPE_PBC     0x0200
#define CONNECTION_TYPE_BATTC   0x0400
#define CONNECTION_TYPE_FINDMEL  0x0800
#define CONNECTION_TYPE_OPS     0x1000

#define NULL_HANDLE             0xFF
#define LE_DWORD(p) (((DWORD)(p)[0]) + (((DWORD)(p)[1])<<8) + (((DWORD)(p)[2])<<16) + (((DWORD)(p)[3])<<24))

#define A2DP_STATS // Audio date statitstics

#define KEYRPT_SIZE     11
#define KEYRPT_BUF_SIZE 12
#define KEYRPT_MODIFIER 3
#define KEYRPT_CODE     5

typedef struct
{
    BYTE     *m_pAudioData;
    BYTE     *m_pData;
    DWORD     m_dwAudioDataLen;
    DWORD     m_dwChunkLen;
    DWORD     m_dwAudioSent;
    DWORD     m_PacketsToSend; // incremented on receiving the message to send new buffers
    DWORD     m_PacketsSent;   // incremented in the write thread
    DWORD     m_BytesPerPacket;// received
}hci_audio_sample_t;

class Worker;
// remote device information
class CBtDevice
{
public:
    CBtDevice (bool paired=false);
    ~CBtDevice ();

    UINT8 m_address[6];
    UINT8  address_type;
    UINT16 m_conn_type;
    UINT16 m_audio_handle;
    UINT16 m_hf_handle;
    UINT16 m_ag_handle;
    UINT16 m_spp_handle;
    UINT16 m_hidh_handle;
    UINT16 m_iap2_handle;
    UINT16 m_avrc_handle;
    UINT16 con_handle;
    UINT16 m_bsg_handle;
    UINT16 m_pbc_handle;
    UINT16 m_avk_handle;
    UINT16 m_battc_handle;
    UINT16 m_findmel_handle;
    UINT16 m_mce_handle;

    UINT8  role;

    char m_name[100];
    bool m_bIsLEDevice;

    QByteArray m_nvram;
    int m_nvram_id;
    bool m_paired;
};


// Main app
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    QString m_SettingsFile;

    void closeEvent (QCloseEvent *event);
    void showEvent(QShowEvent *ev);
    void HandleDeviceEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void Log(const char * tr,...);
    void DumpData(char *description, void* p, unsigned int length, unsigned int max_lines);
    BYTE ProcNibble (char n);
    USHORT GetHandle(QString &str);
    DWORD GetHexValue(LPBYTE buf, DWORD buf_size, QString &str);

    char * GetCurrentWorkingDirectory();
    DWORD qtmin(DWORD len, DWORD bufLen);

    // Device manager
    QMessageBox dl_msgbox;
    void WriteNVRAMToDevice(bool bBLEDevice);
    int FindBaudRateIndex(int baud);
    void setDevName(char * dev_name);
    void setDevBda(BYTE* bda);
    void setPairingMode();
    void setVis();
    WicedSerialPort *m_CommPort;
    int errorNumber();
    bool m_bPortOpen ;
    bool m_bPeripheralUart;
    bool SetupCommPort();
    bool SendWicedCommand(unsigned short command, unsigned char * payload, unsigned int len);
    int PortWrite(unsigned char * data, DWORD Length);
    void InitDm();
    void closeEventDm (QCloseEvent *event);
    void HandleDeviceEventsDm(DWORD opcode, LPBYTE p_data, DWORD len);
    void DecodeEIR(LPBYTE p_data, DWORD len, char * szName, int name_len);
    void EnableUI(bool bEnable);
    void EnableTabs(UINT8 feature, bool bEnable);
    bool m_bUIEnabled;
    void CloseCommPort();
    void ClearPort();
    QWaitCondition serial_read_wait;
    bool m_scan_active;
    bool m_inquiry_active;
    void GetVersion();
    void HandleDeviceEventsMisc(DWORD opcode, LPBYTE p_data, DWORD len);
    UINT8 m_major;
    UINT8 m_minor;
    UINT8 m_rev;
    UINT32 m_build;
    uint32_t m_chip;
    UINT16 m_features;
    QStringList m_strComPortsIDs;
    QIcon m_paired_icon;
    QSettings m_settings;
    FILE * m_fp_logfile;
    void VirtualUnplug(CBtDevice *pDev);
    void Startup();

    // HCI Firmware Update
    void onHandleWicedEventHciDfu(unsigned int opcode, unsigned char *p_data, unsigned int len);
    bool FirmwareDownloadStart(QString filename);
    void FirmwareDownloadStop();
    void FirmwareDownloadSendCmd(UINT8 cmd, void * p_data, int len);
    void FirmwareDownloadSendData();
    void FirmwareDownloadCleanUp();
    void FirmwareDownloadTimeout();
    FILE * m_fpDownload;
    UINT32 m_nFirmwareSize;
    UINT32 m_nFirmwareSentSize;
    UINT32 m_nDownloadSectorSize;
    UINT32 m_nDownloadCRC;
    QTimer *m_pDownloadTimer;

    void HandleA2DPEvents(DWORD opcode, DWORD len, BYTE *p_data);
    CBtDevice *AddDeviceToList(BYTE *addr, QComboBox * pCb, char * bd_name=nullptr,bool bPaired=false);
    CBtDevice * FindInList(BYTE * addr, QComboBox * pCb);
    CBtDevice * FindInList(UINT16 conn_type, UINT16 handle, QComboBox * pCb);
    void SelectDevice(QComboBox* cb, BYTE * bda);
    CBtDevice * GetSelectedDevice();
    CBtDevice * GetSelectedLEDevice();
    void ResetDeviceList(QComboBox *cb);
    void onHandleWicedEventDm(unsigned int opcode, unsigned char*p_data, unsigned int len);
    void SetDevicePaired(BYTE * info, int len=6);
    void HidHostDeviceAdd(BYTE * bda);
    void HidHostDeviceRemove(BYTE * bda);
    void SendHidHostAdd(void);
    void SendBattCAdd(void);
    void BattCDeviceAdd(BYTE * bda);
    void BattCDeviceRemove(BYTE * bda);
    void FindMeLocatorDeviceAdd(BYTE * bda);
    void FindMeLocatorDeviceRemove(BYTE * bda);
    void SendFindMeLocatorAdd(void);

    BOOL SendLaunchRam();
    BOOL SendDownloadMinidriver();
    void downlWoad(FILE * fHCD);
    void SendRecvCmd(BYTE *arHciCommandTx, int tx_sz, BYTE *arBytesExpectedRx, int rx_sz);
    DWORD ReadCommPort(BYTE *lpBytes, DWORD dwLen, QSerialPort * m_CommPort);
    void ReadDevicesFromSettings(const char *group, QComboBox *cbDevices, QPushButton *btnUnbond);

    void DisableBluetoothClassic();

    // command line
    QString str_cmd_port;
    QString str_cmd_baud;
    int iSpyInstance;

    // Scripting support
    bool scripting;
    QThread* m_script_read_thread;
    Worker* m_script_read_worker;
    void CreateScriptThread();
    QMutex m_script_write;
    int CallWicedHciApi(UINT8 *data, UINT8 len);

    // Serial port read
    QThread* m_port_read_thread;
    Worker* m_port_read_worker;
    void CreateReadPortThread();
    QMutex m_write;

    // Startup timer
    QTimer *m_dmStartupTimer;

    // audio source
    bool m_audio_connected;
    bool m_audio_started;
    bool m_audio_i2s_input_enable;
    bool m_audio_mp3_format_enable;
    uint8_t m_audio_format;     /* 0: Wav    1: MP3 */
    int m_audio_play_status_send_limit_count;
    int m_audio_play_status_send_limit_counter;
#ifdef A2DP_STATS
    int m_audio_total_sent_pkt_count;
#endif

    bool m_volMute;
    FILE * m_fpAudioFile;
    hci_audio_sample_t m_uAudio;
    void InitAudioSrc();
    void closeEventAudioSrc(QCloseEvent *event);
    void onHandleWicedEventAudioSrc(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAudioSrc(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleA2DPEventsAudioSrc(DWORD opcode, LPBYTE p_data, DWORD len);
    void setAudioSrcUI();
    BYTE * ExecuteSetAudioFile();
    void HandleA2DPAudioRequestEvent(BYTE * pu8Data, DWORD len);
    CBtDevice* GetConnectedAudioSrcDevice();
    BYTE* GetAudioDataDataChunk(BYTE *pWavData, DWORD dwWavDataLen, DWORD *pdwDataLen);
    BYTE * ExecuteSetAudioFile(char *pcFileName);
    BYTE* ReadFile(const char* FilePathName, DWORD *pdwWavDataLen);
    bool InitializeAudioFile();
    int GetSamplingFrequencyValue(int index);
    QMutex m_audio_packets;
    QWaitCondition audio_tx_wait;

    // Audio source dual A2DP
    bool m_audio_connected_dual_a2dp;
    bool m_audio_started_dual_a2dp;
    bool m_audio_i2s_input_enable_dual_a2dp;
    bool m_audio_mp3_format_enable_dual_a2dp;
    int m_audio_play_status_send_limit_count_dual_a2dp;
    int m_audio_play_status_send_limit_counter_dual_a2dp;
    int m_audio_connected_num_dual_a2dp;
    void InitAudioSrc_DualA2DP();
    void closeEventAudioSrc_DualA2DP(QCloseEvent *event);
    void onHandleWicedEventAudioSrc_DualA2DP(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAudioSrc_DualA2DP(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleA2DPEventsAudioSrc_DualA2DP(DWORD opcode, LPBYTE p_data, DWORD len);
    void setAudioSrcUI_DualA2DP();
    void HandleA2DPAudioRequestEvent_DualA2DP(BYTE * pu8Data, DWORD len);
    bool InitializeAudioFile_DualA2DP();
    BYTE* GetAudioDataDataChunk_DualA2DP(BYTE *pWavData, DWORD dwWavDataLen, DWORD *pdwDataLen);
    BYTE* ExecuteSetAudioFile_DualA2DP(char *pcFileName);
    int GetSamplingFrequencyValue_DualA2DP(int index);
    CBtDevice* GetConnectedAudioSrcDevice_DualA2DP();

    // Hands-free
    void InitHF();
    void onHandleWicedEventHF(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleHFEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void SendAtCmd(int nAtCmd, int num, char *atStr);
    CBtDevice* GetConnectedHFDevice();
    bool m_audio_connection_active;
    int m_mic_cur_pos;
    int m_speaker_cur_pos;

    // SPP
    void InitSPP();
    void onHandleWicedEventSPP(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleSPPEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    DWORD SendFileThreadSPP();
    CBtDevice* GetConnectedSPPDevice();
    DWORD   m_spp_bytes_sent;
    DWORD   m_spp_total_to_send;
    BYTE    m_spp_tx_complete_result;
    FILE   *m_spp_receive_file;
    DWORD m_hSppTxCompleteEvent;
    QWaitCondition spp_tx_wait;

    QThread* m_thread_spp;
    Worker* m_worker_spp;

    // AG
    void InitAG();
    void onHandleWicedEventAG(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleAgEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    bool m_ag_connection_active;
    CBtDevice* GetConnectedAGDevice();

    // BLE/BR HID Device
    void InitBLEHIDD();
    void onHandleWicedEventBLEHIDD(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleBLEHIDDEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void btnBLEHIDSendKeyInit();
    void btnBLEHIDSendKey();
    bool btnBLEHIDSendKeyRelease(BYTE c, QPushButton * button);
    void btnBLEHIDSendKeyDown(BYTE c, QPushButton * button);
    void btnBLEHIDSendKeyUp(BYTE c, QPushButton * button);
    void btnBLEHIDSendMedia(BYTE c, bool pressed, QPushButton * button);
    void setHIDD_buttonColor(QPushButton * button, const QColor &color);
    void UpdateHIDD_ui_host();
    void UpdateHIDD_ui_pairing();
    void setHIDD_HostAddr(unsigned char * ad);
    void setHIDD_linkChange(unsigned char * ad, bool cn);
    bool m_b_is_hidd;
    BYTE m_pairing_mode;
    unsigned char m_host_ad[6];
    BYTE keyRpt_buf[KEYRPT_BUF_SIZE];
    bool m_host_valid;
    bool m_connected;
    BYTE m_host_type;

    // HID Host
    void HidhVirtualUnplug(uint16_t handle);
    void setRadioHIDH_BLE(int ble);
    void InitHIDH();
    void onHandleWicedEventHIDH(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleHIDHEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void DumpMemory(BYTE * p_buf, int length);
    CBtDevice* GetConnectedHIDHDevice();
    CBtDevice* GetSelectedHIDDevice();
    void HandleHidHAudioStart(LPBYTE p_data, DWORD len);
    void HandleHidHAudioStop(LPBYTE p_data, DWORD len);
    void HandleHidHAudioRxData(LPBYTE p_data, DWORD len);
    bool m_hidh_wakeup_state;
    bool m_hidh_audio_started;
    bool m_hidh_audio_configured;
#ifdef PCM_ALSA // HID Host Audio based on Linux ALSA API
    snd_pcm_t *m_alsa_handle;
#endif
    uint8_t m_nb_channel;

// HID Host Audio based on Win32 media player API
#ifdef Q_OS_WIN32
    void WaveOutCallback(UINT uMsg, DWORD_PTR dwParam1);
#define WAVE_HDR_NB                 2
#define WAVE_HDR_BUFFER_SIZE        1000
    uint8_t WaveOutHeaderBuffer[WAVE_HDR_NB][WAVE_HDR_BUFFER_SIZE];
    uint8_t WaveOutBuffer[WAVE_HDR_BUFFER_SIZE * 6];
    int WaveOutBufferIn;
    int WaveOutBufferOut;
    int WaveOutBufferNb;
#endif

    // AVRC CT
    void InitAVRCCT();
    void onHandleWicedEventAVRCCT(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAVRCCT(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleAVRCControllerEvents(DWORD opcode, BYTE *p_data, DWORD len);
    void setAVRCCTUI();
    CBtDevice* GetConnectedAVRCDevice();


    // AVRC TG
    void InitAVRCTG();
    void onHandleWicedEventAVRCTG(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAVRCTG(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleAVRCTargetEvents(DWORD opcode, BYTE *p_data, DWORD len);
    void setAVRCTGUI();

    void SetTrack();
    void TrackInfo();
    void PlayerStatus();

    int m_current_volume_pct;
    uint32_t m_current_song_pos;
    uint16_t m_tg_play_status_timeout_ms;

    //GATT
    void InitGATT();
    void onHandleWicedEventGATT(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleLEEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleGattEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void setGATTUI();
    USHORT GetConHandle(QComboBox *pCombo);
    QString GetServiceUUIDDesc(uint16_t uuid16);
    QString GetServiceUUIDDesc(uint8_t *p_uuid128);

    void  SetRole(CBtDevice *pDevice, uint8_t role);
    UINT8 GetRole(CBtDevice *pDevice);
    void  UpdateGattButtons(CBtDevice *pDevice);
    BOOL  m_advertisments_active;
    ULONG m_notification_uid;

    // BSG
    void InitBSG();
    void onHandleWicedEventBSG(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleBSGEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    DWORD SendFileThreadBSG();
    CBtDevice* GetConnectedBSGDevice();
    DWORD   m_bsg_bytes_sent;
    DWORD   m_bsg_total_to_send;
    BYTE    m_bsg_tx_complete_result;
    FILE   *m_bsg_receive_file;
    DWORD m_hBsgTxCompleteEvent;
    QWaitCondition bsg_tx_wait;

    USHORT  m_bsg_sent;
    USHORT  m_bsg_acked;
    DWORD   m_uart_tx_size;

    // PBC
    void InitPBC();
    void onHandleWicedEventPBC(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void wiced_bt_pbc_pb_save(UINT8* p_buffer, int len);
    void wiced_bt_pbc_process_pb_event(LPBYTE p_data, DWORD len);
    void HandlePBCEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    CBtDevice* GetConnectedPBCDevice();
    bool m_pbc_connection_active;
    bool m_pbc_file_remove;

    // HomeKit
    void InitHK();
    void StartHK();
    void onHandleWicedEventHK(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleHkEvent(DWORD opcode, LPBYTE p_data, DWORD len);
    void SendHciCommand(UINT16 command, USHORT handle, LPBYTE p, DWORD dwLen);

    void SetLightOnOff(BOOL on);
    void UpdateUI(USHORT handle, LPBYTE p, DWORD dwLen);
    void ShowMessage();
    bool m_bLightOn;
    uint m_nLightBrightness;
    uint m_nDoorState;
    uint m_nLockState;
    uint m_nLockTargetState;
    uint m_nIdentifyTimerCounter;
    QTimer *p_timer;
    USHORT m_hIdentify;
    USHORT m_hLightOn;
    USHORT m_hLightBrightness;
    USHORT m_hDoorState;
    USHORT m_hLockState;
    USHORT m_hLockTargetState;

    // iAP2
    void InitiAP2();
    void onHandleWicedEventiAP2(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleiAP2PEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    DWORD SendFileThreadiAP2();
    CBtDevice* GetConnectediAP2Device();

    DWORD   m_iap2_bytes_sent;
    DWORD   m_iap2_total_to_send;
    BYTE    m_iap2_tx_complete_result;
    FILE    *m_iap2_receive_file;
    DWORD   m_hiap2TxCompleteEvent;
    QWaitCondition iap2_tx_wait;

    // audio sink
    void InitAudioSnk();
    void onHandleWicedEventAudioSnk(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleA2DPEventsAudioSnk(DWORD opcode, BYTE *p_data, DWORD len);
    CBtDevice* GetConnectedAudioSnkDevice();

    // GATT DB
    void InitGATT_DB();
    void onHandleWicedEventGATT_DB(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleGATT_DBEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleGattReadRequestEvent(LPBYTE p_data, DWORD len);
    void HandleGattWriteRequestEvent(LPBYTE p_data, DWORD len);

    // Battery Client
    void InitBATTC(void);
    void onHandleWicedEventBATTC(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleBATTCHEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    CBtDevice* GetConnectedBATTCDevice(void);

    // Demo
    void InitDemo();
    void onHandleWicedEventDemo(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDemoEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void ConnectWiFi(BYTE *ssid, BYTE *password);

    // FindMe Locator
    void InitFINDMEL(void);
    void onHandleWicedEventFINDMEL(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleFINDMELHEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    CBtDevice* GetConnectedFINDMELDevice(void);

    // OPS
    void InitOPS();
    void onHandleWicedEventOPS(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleOPSEvents(unsigned int opcode, unsigned char *p_data, unsigned int len);
    CBtDevice* GetConnectedOPSDevice();
    FILE *m_opp_receive_file;
    DWORD received_size;

    // ANP
    void InitANP();
    void onHandleWicedEventANP(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleANCEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleANSEvents(DWORD opcode, LPBYTE p_data, DWORD len);

    //LE COC
    void InitLecoc();
    DWORD SendFileThreadLECOC();
    void recvDataFromDevice(char *p_data, unsigned int len);
    void rcvdTxCompleteFromDevice(void);
    void stopAdv(void);
    void onHandleWicedEventLECOC(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void on_cbLecocReceiveFile_clicked(bool checked);

    DWORD   m_lecoc_bytes_sent;
    DWORD   m_lecoc_total_to_send;
    BYTE    m_lecoc_tx_complete_result;
    FILE   *m_lecoc_receive_file;

    //LED Demo
    int m_led_brightness_level;

    //OTP Client
    void InitOTPClient();
    DWORD SendOTAImageFileThreadOTPClient();
    void onHandleWicedEventOTPClient(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void rcvdTxCompleteFromOTPClientDevice();
    void rcvdUpgradeInitDoneFromOTPClientDevice();

    DWORD   m_otpc_bytes_sent;
    DWORD   m_otpc_total_to_send;
    DWORD   m_otpc_crc32;
    BYTE    m_otpc_tx_complete_result;
    QWaitCondition otpc_tx_wait;

    //MAP Client
    void InitMAPClient();
    void onHandleWicedEventMAPClient(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void onHandleMceMasInstances(unsigned char *p_data, unsigned int len);
    void onHandleMceConnected(unsigned char *p_data, unsigned int len);
    void onHandleMceDisconnected(unsigned char *p_data, unsigned int len);
    void onHandleMceFolderSet(unsigned char *p_data, unsigned int len);
    void onHandleMceFolderList(unsigned char *p_data, unsigned int len);
    void onHandleMceMessageList(unsigned char *p_data, unsigned int len);
    void onHandleMceMessage(unsigned char *p_data, unsigned int len);
    void onHandleMceMessagePushed(unsigned char *p_data, unsigned int len);
    void onHandleMceMessageStatusSet(unsigned char *p_data, unsigned int len);
    void onHandleMceNotifReg(unsigned char *p_data, unsigned int len);
    void onHandleMceNotif(unsigned char *p_data, unsigned int len);

    QString m_mce_set_folder;
    QString m_mce_cur_folder;
    QString m_mce_rcvd_text;
    int m_mce_msg_type;
    QString m_mce_push_message;
    UINT16 m_mce_list_offset;
    bool m_mce_notif_registered;
    QListWidgetItem *m_mce_delete_item;

signals:
   void HandleWicedEvent(unsigned int opcode, unsigned int len, unsigned char *p_data);
   void HandleTrace(QString *pTrace);
   void HandleLeAdvState(BYTE val);
   void ScrollToTop();
   void ListClear();

   // PBC
    void ShowPhonebookData();

public slots:
    void startUpTimer();

   // utility methods
   void processTrace(QString * trace);
   void processClear();
   void processScrollToTop();
   void onMsgBoxButton(QAbstractButton*btn);
   void onClear();
   void btnFindLogfileClicked();
   void btnLogToFileClicked(bool);
   void btnAddTraceClicked();
   void EnableAppTraces();
   void DisableAppTraces();
   void handleReadyRead();
   void serialPortError(QSerialPort::SerialPortError error);

    // Device manager
    void processHandleLeAdvState(BYTE val);
    void btnClearClicked();
    void onHandleWicedEvent(unsigned int opcode, unsigned int len, unsigned char *p_data);
    void onStartDisc();
    void onStopDisc();
    void onReset();
    void OnBnClickedBREDRUnbond();
    void OnBnClickedLeUnbond();
    void onUnbond(QComboBox* cb);
    void onDevChange(QString);
    void onLEDevChange(QString);
    void onDiscoverable(bool);
    void onConnectable(bool);
    void onPairable(bool);
    void on_btnOpenPort_clicked();
    void onDlProgress(QString *msg, int pktcnt, int bytecnt);
    void onDlDone(const QString &s);
    void onDownload();
    void onFindPatchFile();
    void OnBnClickedVersionInfo();

    // AV source
    void onDisconnectAudioSrc();
    void onConnectAudioSrc();
    void onFindAudioFile();
    void onStartAudio();
    void onStopAudio();
    void onAudioSrcSine(bool);
    void onAudioSrcFile(bool);
    void onAudioSrcI2S(bool);
    void onAudioFileFormatWav(bool);
    void onAudioFileFormatMp3(bool);
    void on_btnHelpAVSRC_clicked();

    // AV source dual A2DP
    void onDisconnectAudioSrc_DualA2DP();
    void onConnectAudioSrc_DualA2DP();
    void onFindAudioFile_DualA2DP();
    void onStartAudio_DualA2DP();
    void onStopAudio_DualA2DP();
    void onAudioSrcSine_DualA2DP(bool);
    void onAudioSrcFile_DualA2DP(bool);
    void onAudioSrcI2S_DualA2DP(bool);
    void onAudioFileFormatWav_DualA2DP(bool);
    void onAudioFileFormatMp3_DualA2DP(bool);
    void on_btnHelpAVSRC_clicked_DualA2DP();

    // Hands-free
    void on_btnConnectHF_clicked();
    void on_btnDisconnectHF_clicked();
    void on_btnHFConnectAudio_clicked();
    void on_btnHFHangup_clicked();
    void on_btnHFAnswer_clicked();
    void on_btnHFRedial_clicked();
    void on_btnHFDial_clicked();
    void on_btnHFDTMF_clicked();
    void on_btnHFVoiceReco_clicked();
    void on_btnHFCallHeld_clicked();
    void on_horizontalSliderHFMic_sliderMoved(int position);
    void on_horizontalSliderHFSpeaker_sliderMoved(int position);
    void on_btnHFBtnPress_clicked();
    void on_btnHFLongBtnPress_clicked();
    void on_btnHFActiveCalls_clicked();
    void on_btnHelpHF_clicked();
    void on_btnHFNREC_clicked();
    void on_btnHFCNUM_clicked();
    void on_btnHFBINP_clicked();
    void on_btnHFUpdate_ind_clicked();
    void on_btnHFBIA_clicked();

    // SPP
    void on_btnSPConnect_clicked();
    void on_btnSPPDisconnect_clicked();
    void on_btnSPPSend_clicked();
    void on_btnSPPBrowseSend_clicked();
    void on_btnSPPBrowseReceive_clicked();
    void on_cbSPPSendFile_clicked(bool checked);
    void on_cbSPPReceiveFile_clicked(bool checked);
    void on_cbSPPThreadComplete();
    void on_btnHelpSPP_clicked();

    // AG
    void on_btnAGConnect_clicked();
    void on_btnAGDisconnect_clicked();
    void on_btnAGAudioConnect_clicked();
    void on_btnHelpAG_clicked();

    // BLE/BR HID Device
    void on_btnBLEHIDSendReport_clicked();
    void on_btnBLEHIDPairingMode_clicked();
    void on_btnBLEHIDConnectDisconnect_clicked();
    void on_btnBLEHIDDVirtualUnplug_clicked();
    void on_cbBLEHIDCapLock_clicked();
    void on_cbBLEHIDCtrl_clicked();
    void on_cbBLEHIDAlt_clicked();
    void on_btnHelpHIDD_clicked();

    // HID Host
    void on_btnHIDHConnect_clicked();
    void on_btnHIDHDisconnect_clicked();
    void on_btnHIDHGetDesc_clicked();
    void on_cbHIDHProtocol_currentIndexChanged(int index);
    void on_btnHIDHWakeAdd_clicked();
    void on_btnHIDHWakeEnable_clicked();
    void on_radioHIDHBLE_clicked();
    void on_radioHIDHBREDR_clicked();
    void on_btnHIDHGetReport_clicked();
    void on_btnHIDHSetReport_clicked();
    void on_btnHelpHIDH_clicked();

    //AVRCP CT
    void onCTPlay();
    void onCTPause();
    void onCTStop();
    void onCTNext();
    void onCTPrevious();
    void onCTVolumeUp();
    void onCTVolumeDown();
    void onCTMute();
    void onCTConnect();
    void onCTDisconnect();
    void onCTSkipForwardPressed();
    void onCTSkipForwardReleased();
    void onCTSkipBackwardPressed();
    void onCTSkipBackwardReleased();
    void onCTUnitInfo();
    void onCTSubUnitInfo();
    void onCTRepeatMode(int index);
    void onCTShuffleMode(int index);
    void cbCTVolumeChanged(int index);
    void on_cbTGVolume_currentIndexChanged(int index);
    void on_btnAVRCTBtnPress_clicked();
    void on_btnAVRCTLongBtnPress_clicked();
    void on_btnHelpAVRC_CT_clicked();

    // AVRCP TG
    void onTGPlay();
    void TGPlay();
    void onTGPause();
    void onTGStop();
    void TGStop();
    void onTGNext();
    void onTGPrevious();
    void onTGConnect();
    void onTGDisconnect();
    void onTGRegisterNotification();
    void oncbTGShuffleCurrentIndexChanged(int index);
    void oncbTGRepeatCurrentIndexChanged(int index);
    void on_btnHelpAVRC_TG_clicked();
    void onTGMute();

    //GATT
    void OnBnClickedAncsPositive();
    void OnBnClickedAncsNegative();

    void OnBnClickedDiscoverDevicesStart();
    void OnBnClickedDiscoverDevicesStop();
    void OnBnClickedLeConnect();
    void OnBnClickedLeCancelConnect();
    void OnBnClickedLeDisconnect();
    void OnBnClickedDiscoverServices();
    void OnBnClickedDiscoverCharacteristics();
    void OnBnClickedDiscoverDescriptors();
    void OnBnClickedStartStopAdvertisements();
    void OnBnClickedSendNotification();
    void OnBnClickedSendIndication();
    void OnBnClickedCharacteristicRead();
    void OnBnClickedCharacteristicWrite();
    void OnBnClickedCharacteristicWriteWithoutResponse();
    void on_btnHelpGATT_clicked();

    //BSG
    void on_btnBSGSend_clicked();
    void on_btnBSGBrowseSend_clicked();
    void on_btnBSGBrowseReceive_clicked();
    void on_cbBSGSendFile_clicked(bool checked);
    void on_cbBSGReceiveFile_clicked(bool checked);
    void on_cbBSGThreadComplete();
    void on_btnHelpBSG_clicked();

    // PBC
    void on_btnPBCConnect_clicked();
    void on_btnPBCDisconnect_clicked();
    void on_btnPBCAbort_clicked();
    void on_btnPBCPhonebook_clicked();
    void on_btnPBCCallHistory_clicked();
    void on_btnPBCICCall_clicked();
    void on_btnPBCOCCalls_clicked();
    void on_btnPBCMissedCalls_clicked();
    void on_ShowPhonebookData();
    void on_btnHelpPBC_clicked();

    // HomeKit
    void on_btnHKRead_clicked();
    void on_btnHKWrite_clicked();
    void on_btnHKList_clicked();
    void on_btnHKSwitch_clicked();
    void on_btnHKSet_clicked();
    void on_cbDoorState_currentIndexChanged(int index);
    void on_cbLockState_currentIndexChanged(int index);
    void on_cbLockTargetState_currentIndexChanged(int index);
    void on_timer();
    void on_btnHKFactoryReset_clicked();
    void on_btnHKGetToken_clicked();
    void on_btnHelpHK_clicked();

    // iAP2
    void on_btniAPConnect_clicked();
    void on_btniAPSDisconnect_clicked();
    void on_btniAPSend_clicked();
    void on_cbiAP2SendFile_clicked();
    void on_btniAP2BrowseSend_clicked();
    void on_cbiAPReceiveFile_clicked();
    void on_btniAPBrowseReceive_clicked();
    void on_btniAPRead_clicked();
    void on_cbiAP2ThreadComplete();
    void on_btniAP2ReadCert_clicked();
    void on_btniAP2GenSign_clicked();
    void on_btnHelpIAP2_clicked();

    // audio sink
    void on_btnAVSinkConnect_clicked();
    void on_btnAvSinkDisconnect_clicked();
    void on_btnAvSinkStart_clicked();
    void on_btnAvSinkSuspend_clicked();
    void on_btnHelpAVK_clicked();

    // Battery Client
    void on_btnBATTCConnect_clicked(void);
    void on_btnBATTCDisconnect_clicked(void);
    void on_btnBATTCReadLevel_clicked(void);
    void on_btnHelpBATTC_clicked(void);

    // FindMe Locator
    void on_btnFINDMELConnect_clicked(void);
    void on_btnFINDMELDisconnect_clicked(void);
    void on_cbFINDMELLevel_currentIndexChanged(int index);
    void on_btnHelpFindMe_clicked();

    // OPS
    void on_btnOPSDisconnect_clicked();
    void on_btnHelpOPPS_clicked();

    // GATT DB
    void on_btnAddPrimarySvc_clicked();
    void on_btnAddSecondarySvc_clicked();
    void on_btnAddIncSvc_clicked();
    void on_btnAddChar_clicked();
    void on_btnAddDesc_clicked();
    void on_btnStartAdvertGATTDB_clicked();
    void on_btnInitGATTDB_clicked();
    void on_btnHelpGATT_DB_clicked();

    // ANP (ANS)
    void on_btnANSSetAlert_clicked();
    void on_btnGenerateAlert_clicked();
    void on_btnCancelAlert_clicked();
    void on_listAlertCategories_itemClicked(QListWidgetItem *item);
    void on_radioANSSetNewAlerts_clicked();
    void on_radioANSSetUnreadAlerts_clicked();
    // ANP (ANC)
    void on_radioANCReadNewAlerts_clicked();
    void on_radioANCReadUnreadAlerts_clicked();
    void on_btnACSReadAlerts_clicked();
    void on_listAlertCategoriesANC_itemClicked(QListWidgetItem *item);
    void on_btnANCControlAlerts_clicked();
    void on_btnANCEnableNewAlerts_clicked();
    void on_btnANCUnreadAlerts_clicked();
    void on_btnHelpANP_clicked();

    // Demo
    void on_btnHelpDemo_clicked();

    //LE COC
    void on_btnLecocConnect_clicked();
    void on_btnLecocDisconnect_clicked();
    void on_btnLecocSend_clicked();
    void on_cbLECOCThreadComplete();
    void on_cbLecocReceiveFile_clicked();
    void on_cbLecocSendFile_clicked();
    void on_pushButtonLecocStartAdv_clicked();
    void on_btnLecocBrowseSend_clicked();
    void on_btnLecocBrowseReceive_clicked();
    void on_lineEditLecocPsm_textChanged(const QString &arg1);

    // LED Demo
    void InitLED_Demo();
    void on_horizontalSliderLEDbrightness_sliderMoved(int position);
    void on_pushButtonLEDONOFF_clicked();

    //OTP Client
    void on_btnOTPClientConnect_clicked();
    void on_btnOTPClientDisconnect_clicked();
    void on_btnOTPClientBrowseFile_clicked();
    void on_btnOTPClientUpgrade_clicked();
    void on_cbOTPClientThreadComplete();

    // MAP Client
    void on_btnMceGetServices_clicked();
    void on_btnMceConnect_clicked();
    void on_btnMceDisconnect_clicked();
    void on_cbMceFolderList_currentIndexChanged(const QString &text);
    void on_listMceMessages_currentItemChanged(QListWidgetItem* item, QListWidgetItem* previous);
    void on_btnMceDelete_clicked();
    void on_btnMceReply_clicked();
    void on_btnMceSend_clicked();

    UINT16 MapAddTlv(UINT8 *buf, UINT8 type, UINT8 *value, UINT8 value_len);
    UINT8 *MapFindTlv(UINT8 *buf, UINT8 data_len, UINT8 type);
    UINT16 MceSendMessageData(UINT16 mce_handle);
    void MceResizeMessageWindows(bool larger);
    void MceSendGetMessageListing(UINT16 mce_handle, UINT16 start_offset, UINT16 max_count);

public:
    Ui::MainWindow *ui;

private slots:

    void on_btnHelpTab_clicked();
    void window_shown();
    void on_btnBLEHIDSendKey_1_pressed();
    void on_btnBLEHIDSendKey_1_released();
    void on_btnBLEHIDSendKey_2_pressed();
    void on_btnBLEHIDSendKey_2_released();
    void on_btnBLEHIDSendKey_3_pressed();
    void on_btnBLEHIDSendKey_3_released();
    void on_btnBLEHIDSendKey_audio_pressed();
    void on_btnBLEHIDSendKey_audio_released();
    void on_btnBLEHIDSendKey_ir_pressed();
    void on_btnBLEHIDSendKey_ir_released();
    void on_btnBLEHIDSendKey_motion_pressed();
    void on_btnBLEHIDSendKey_motion_released();
    void on_btnBLEHIDSendKey_a_pressed();
    void on_btnBLEHIDSendKey_a_released();
    void on_btnBLEHIDSendKey_b_pressed();
    void on_btnBLEHIDSendKey_b_released();
    void on_btnBLEHIDSendKey_c_pressed();
    void on_btnBLEHIDSendKey_c_released();
    void on_cbBLEHIDDebug_currentIndexChanged(int index);
    void on_btnBLEHIDSendKey_esc_pressed();
    void on_btnBLEHIDSendKey_esc_released();
    void on_btnBLEHIDSendKey_up_pressed();
    void on_btnBLEHIDSendKey_up_released();
    void on_btnBLEHIDSendKey_enter_pressed();
    void on_btnBLEHIDSendKey_enter_released();
    void on_btnBLEHIDSendKey_left_pressed();
    void on_btnBLEHIDSendKey_left_released();
    void on_btnBLEHIDSendKey_down_pressed();
    void on_btnBLEHIDSendKey_down_released();
    void on_btnBLEHIDSendKey_right_pressed();
    void on_btnBLEHIDSendKey_right_released();
    void on_cbBLEHIDHold_clicked();
};

// Thread for SPP, iAP2 and serial port read
class Worker : public QObject
 {
     Q_OBJECT

public:
    explicit Worker() {m_pParent=NULL;}
    ~Worker(){}

    // Read serial port
    DWORD Read(BYTE * lpBytes, DWORD dwLen);
    DWORD ReadNewHciPacket(BYTE * pu8Buffer, int bufLen, int * pOffset);
    MainWindow * m_pParent;

 public slots:
     void process_spp();
     void process_bsg();
     void process_iap2();
     void read_serial_port_thread();
     void process_lecoc();
     void process_OTPClient();
#ifdef WIN32
     void read_script_thread();
#endif

private:
     void dump_hci_data_hexa(const char *p_prefix, BYTE *pu8Buffer, DWORD dwLen);
     void process_vse(BYTE *pu8Buffer, DWORD dwLen);

 signals:
     void finished();
     void HandleWicedEvent(DWORD opcode, DWORD len, BYTE *p_data);

 };

class AudioFileWriter : public QThread
{
    Q_OBJECT

public:
    explicit AudioFileWriter(MainWindow * pParent);
    ~AudioFileWriter() {}
    MainWindow * m_pParent;
    void SendNextData(hci_audio_sample_t * puHci, int bytesPerPacket);
    BYTE* GetAudioDataDataChunk(BYTE *pWavData, DWORD dwWavDataLen, DWORD *pdwDataLen);
    BYTE * ExecuteSetAudioFile(char *pcFileName);


protected:
    void run() Q_DECL_OVERRIDE;
};
extern bool m_bClosing ;
extern MainWindow *g_pMainWindow;
#endif // MAINWINDOW_H
