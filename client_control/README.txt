ClientControl
=============

Overview
========
This application is used to control applications running on the WICED BT
devices via WICED HCI protocol over WICED HCI UART interface. This application
emulates an MCU application. For more information on WICED HCI,
see the "WICED HCI UART Control Protocol" document.

The ClientControl application runs on Windows, Linux, and macOS. This
application was developed using QT IDE (QT version 5.9.1).

Supported OS
============
Windows: Windows 7, Windows 8.1, Windows 10, x86, x64
Linux: Ubuntu 16.04/18 and above, 64-bit
macOS: Sierra version 10.12.1 and above. See note for macOS below.

Running the ClientControl application
=====================================
To run the ClientControl sample application, follow these steps:
1. Plug in the WICED BT kit into the computer using a USB cable.
2. On Linux PC, additional steps may be required to enable serial port access,
   see "Note for Linux" below.
3. Build and download the application to the WICED BT kit.
4. If you are using ModusToolbox, the ClientControl application is available
   under Quick Panel -> Tools.
5. To run it manually on, browse to the folder:
   mtb_shared/wiced_btsdk/tools/btsdk-host-apps-bt-ble/(version)/client_control
   Windows PC: Double click "ClientControl.exe" in the 'Windows' folder.
   Linux PC: Execute the script "RunClientControl.sh" in the Linux folder.
   macOS: Execute "ClientControl.dmg" in OSX folder. See "Note for macOS" below.
6. In the ClientControl UI, select the serial port for the WICED BT kit and
   open the port. (See note below.)
7. Once the "ClientControl" app can communicate with the embedded
   Bluetooth application, the application UI will be enabled. Then use the
   UI to perform Bluetooth operations.
8. Bluetooth application and protocol traces will be displayed in the BTSpy
   application.

Note for selecting serial port
==============================
1. The serial port on Windows OS corresponds to the WICED HCI UART COM port.
   On Linux it corresponds to /dev/ttyWICED_HCI_UARTx. On macOS the
   WICED HCI UART port is usually the lower numbered of the two
   /dev/tty-usbserial* ports.
2. If after opening the serial port the UI is not enabled, check if:
   - The correct serial port is selected
   - The embedded application (running on WICED BT kit) supports WICED HCI transport
   - The baud rate selected in the UI matches the baud rate specified in the
     embedded application. See the baud rate specified in the
     wiced_transport_cfg_t structure of the embedded application.

Note for Linux
==============
An additional step is required when connecting a WICED board to a computer
running Linux. On common Linux distributions, the serial UART ports
(such as /dev/ttySx or /dev/ttyUSBx devices, including the
/dev/ttyWICED* devices installed by ModusToolbox) belong to the root user and
the dialout group. Standard users are not allowed to access these devices.
An easy way to allow the current user access to Linux's serial ports is by
adding the user to the dialout group. This can be done using the following
command:
$sudo usermod -a -G dialout $USER
Note: For this command to take effect, the user must log out and then log back in.

Note for macOS
==============
1. The highest baud rate supported in macOS is 3,000,000. All WICED apps that
   want to communicate with the macOS ClientControl app must be built to
   use a max baud rate of 3,000,000. (See the baud rate specified in the
   wiced_transport_cfg_t structure of the embedded application.)
2. Application downloads on WICED platforms may fail on certain macOS
   versions due to an incompatibility between the macOS version and the
   installed driver for the FTDI USB to serial chip.  On macOS versions
   10.10 and earlier, a specific version of the FTDI driver must be installed
   using the instructions available here:
   https://learn.sparkfun.com/tutorials/how-to-install-ftdi-drivers/mac
   On macOS versions 10.11 and greater, the Apple version of the FTDI driver
   must be used, and any previous instance of the FTDI version of the driver
   must be removed from the system, by executing the following command in xterm:
   > sudo rm -rf /Library/Extensions/FTDIUSBSerialDriver.kext /System/Library/Extensions/FTDIUSBSerialDriver.kext
   Reboot the system after performing the rm command.
3. Opening multiple instances of the app on macOS is not as easy as it is in
   Windows, you cannot simply double click on the app like in Windows. This is
   a limitation of the macOS, not the ClientControl app.  However, it is possible
   to run multiple instances on macOS using one of these two methods:
   Method 1:
      1.  Open the .dmg, drag the ClientControl application to the Desktop or any folder.
      2.  Open a terminal window, go to the location of ClientControl.app.
          Repeat the command below for each new instance of ClientControl application.
      3.  $open -n ClientControl.app
   Method 2:
      1.  Open the .dmg, drag the ClientControl application to the Desktop or any folder.
      2.  Right-click on the app and select "Duplicate".
      3.  Open the duplicated app.
4. If you see an error message when trying to execute ClientControl saying it
   is damaged and cannot be opened, perform the following step to workaround:
   Open an xterm window and execute:
      xattr -cr mtb_shared/wiced_btsdk/tools/btsdk-host-apps-bt-ble/(version)/client_control/OSX/ClientControl.app

Instructions for building the application
=========================================
To make a change to the ClientControl application and rebuild the application,
follow these steps:
1. On Linux OS, install gcc, gcc-c++, kernel-devel-$(uname -r),
   "Development Tools", "Development Libraries", libGLU-devel, before
   installing QT Creator
2. Download installer for QT Creator from https://www.qt.io/download/
3. On Windows OS, select the option for MinGW 32-bit compiler
4. Open the QT project file "ClientControl.pro" with QT Creator
5. Build the QT project
6. On Windows OS, ws2_32.lib path might need to be adjusted in the project
   file (ClientControl.pro).

User Interface
==============
The UI is divided in three sections:
1. Device manager UI (area above the tabs) is for controlling the local device
   settings as well as discovering peer devices.
2. The tabs are Bluetooth profiles or features that provide specific functions
   as suggested by the tab name.  Each tab is usable only with certain embedded
   applications and peer devices as mentioned in the 'Setup' description below
   for each tab.
3. The tracing UI (area below the tabs) shows ClientControl application traces.
   It also can log traces to a file.  Use the 'Browse' button to select the
   file to log to and enable 'Log traces to file' checkbox. You can add
   custom traces using the 'Add Trace' button.

Device Manager
++++++++++++++
- Open port:
  This button is used to open or close the serial port used by WICED Eval board.
  Opening the serial port is the first step for using the application
  downloaded to the board by WICED SDK. Select the "Baud rate" and "Flow
  Control" options as appropriate to match the settings for the embedded app.
- Pairable:
  This option enables or disables pairing from a peer device.
- Connectable:
  This option enables or disables connectability from a peer device.
- Discoverable:
  This option enables or disables discoverability from a peer device.
- Reset Device:
  Close the serial port and reset the device. The embedded application may
  need to be downloaded again.
- BLE Discovery:
  Search for BLE devices.
- BR/EDR Discovery:
  Search for BR/EDR devices.
- Unbond:
  Un-pair the selected device. Paired devices will show 'link' icon.
- Firmwre download:
  ClientControl application can be used to upgrade the firmware on the board.
  After an embedded application is built, there will be a .OTA.BIN file in the
  build folder (the same file that is used in OTA upgrade). Click the 'Browse'
  button to select the .OTA.BIN file, then click the 'Download' button to
  start the download. The serial port needs to be opened before this operation.
  Please note this firmware download does not clean up parameters saved on the
  board. It only upgrades the firmware.

AV Source (A2DP Source):
+++++++++++++++++++++++
Setup: app - watch
Peer device - headset, speaker, car-kit
- Connect
  Connect to the selected BR/EDR device to create an A2DP connection. The peer
  selected device should be an audio sink capable device such as a headset,
  speaker, or car-kit. Select the 'Media' and 'Mode' to stream before creating
  the A2DP connection.
- Disconnect
  Disconnect an existing A2DP connection from selected BR/EDR.

AVRC TG (AVRCP Target):
+++++++++++++++++++++++
Setup: app - watch
Peer device - headset, speaker, car-kit
- Connect
  Connect to the selected BR/EDR device to create an AVRCP connection. The peer
  selected device should support the AVRCP Controller role, such as a headset,
  speaker, or car-kit. After creating the A2DP Source connection, the peer
  device may automatically create an AVRCP connection. This option is available
  only when the "PTS_TEST_ONLY" compiler option is enabled in the embedded app.
- Disconnect
  Disconnect an existing AVRCP connection from the selected BR/EDR. This option
  is available only when the "PTS_TEST_ONLY" compiler option is enabled in the
  embedded app.
- Play, Pause, Stop, Forward, Back
  These buttons send 'player status' and 'track info' attribute notifications to
  the peer device.  The peer device should support AVRCP 1.3 or higher. Note that
  these buttons are not controlling the media in the AV Source UI, and are meant
  for demonstrating the API usage. The forward/back buttons will change the
  current track of the built in the play list displayed in the AVRCP UI.
  Note that the built-in playlist is for display only and does not match the
  streaming media in the A2DP Source. The current track is displayed by peer
  devices supporting AVRCP Controller 1.3 or higher.
- Shuffle/Repeat
  These controls change the AVRCP player settings and are used when a peer
  device supports AVRCP Controller 1.3 or higher. These controls do not change
  the media in the A2DP source UI.
- Volume
  This control sets or displays the absolute volume. (Used when a peer device
  supports AVRCP Controller 1.4 or higher).


AVRC CT (AVRCP Controller):
++++++++++++++++++++++++++
Setup: app - watch
Peer device - iPhone or Android phone
Discover the phone from the "BR/EDR Discovery" control and pair the phone
- Connect
  Create an AVRC connection with a peer device. Note that some peer devices may
  not support AVRC CT connection without the A2DP Sink profile.
- Disconnect
  Disconnect an AVRCP connection with a peer device.
- Repeat, Shuffle
  These controls change the AVRCP player settings and are used when a peer
  device supports AVRCP Controller 1.3 or higher.
- Volume
  This control displays the absolute volume. (Used when a peer device supports
  AVRCP Controller 1.4 or higher).
- AMS and ANCS
  For AMS and ANCS, reset the board and download the 'watch' app. From the 'GATT'
  tab click 'Start Advertisements'. From an iPhone app such as 'Light Blue',
  discover the 'watch' app running on the board and pair. Play media on the
  iPhone and control it with the AVRC CT controls. Incoming calls and messages
  will be displayed on the ANCS controls.

Audio Gateway:
++++++++++++++
Setup: app - audio_gateway
Peer device - headset, speaker, car-kit supporting hands-free profile
- Connect
  Connect to a peer device supporting the HF profile
- Disconnect
  Disconnect with a peer device
- Audio Connect
  Open a SCO audio channel with a peer device

Hands-free:
+++++++++++
Setup: app - handsfree
Peer device - phone or device supporting the Audio Gateway profile
- Connect
  Connect to a peer device supporting the AG profile
- Disconnect
  Disconnect with a peer device
- Connect Audio
  Open a SCO audio channel with a peer device
- Hangup, Answer, Redial, etc.
  These controls send the HF profile AT commands for various operations.

GATT:
+++++
Setup: app - watch
Peer device - BLE device or iPhone Light Blue app, Android BLE app, etc.
GATT controls are provided for advertisements, discovering services, connecting
to a GATT server, reading/writing values of handles, and discovering
characteristics and descriptors of handles.

Serial Port Profile:
+++++++++++++++++++
Setup: app - spp
Peer device - any device supporting the SPP server role
- Connect
  Connect to an SPP server
- Disconnect
  Disconnect from an SPP server
- Send
  Send characters typed in the edit control, or a file, to the peer device
- Receive
  Receive data in the edit control (first 50 bytes) or receive and save data
  to a file

iAP2:
+++++
Setup: app - hci_iap2_spp
Peer device: iPhone, iOS 10.2 or SPP server capable device
Note: This app is available only for Apple MFI licensees
iAP2 app sends and receives data over RFCOMM for iOS devices, or over SPP for
non-iOS devices. For UI description, see "Serial Port Profile" above.

OPP Server
++++++++++
Setup: app - opp_server
Peer device: PC, Android phone, etc. supporting OPP Client profile.

From peer device, find the 'OPP server' device, pair, and send a file.

- Disconnect
  Disconnects from OPP client


Phonebook Client
++++++++++++++++
Setup: app - pbap_client
Peer device - phone supporting PBAP server (any recent iPhone or Android phone)
- From the phone, initiate device discovery, find the pbap_client application,
  and perform pairing.
- From the ClientControl UI, select the paired phone and click Connect
- Use the UI to download phone book contacts, and incoming, outgoing, or missed
  calls.
Note: only the first 100 records will be displayed. The application can be
updated to get more records if desired.


HID Device:
++++++++++
Setup: app - dual_mode_keyboard for BR-EDR or BLE remote/mouse for BLE HOGP
Peer device - Windows PC or any HID host
- Enter Pairing Mode
  Sets the local device to pairable
- Connect
  Connect with a HID host
- Send Key
  Sends the specified key from the drop down with options such as button up,
  Caps Lock, etc.
- Send Report
  Send report for Interrupt or Control channel.
