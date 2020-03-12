QT based "WICED Client Control" application
===========================================

This application was developed using QT IDE.

Supported OS:
- Windows: x86, x64 (Windows 7, Windows 8.1, Windows 10)
- Linux: x64 (64 bit Ubuntu 16.04/18)
- Mac: OSx (Sierra version 10.12.1)
  (See note for Mac OS below.)

Version:
- QT version 5.8

Instructions for running the application
----------------------------------------
To run the built-in "Client Control" sample application, follow these steps -
1. Plug in the WICED Evaluation Board into the computer using a USB cable.
2. On Linux PC, additional step may be required to enable serial port access, see
   the WICED Kit Guide for your device in the 'doc' folder.  (Appendix C:
   Connecting to Linux Platforms).
3. Build and download an embedded application to the WICED evaluation board.
4. On Windows PC, double click "ClientControl.exe" in the 'Windows' folder.
5. On Linux PC, execute the script "RunClientControl.sh" in the Linux folder.
6. On Mac OS, execute "ClientControl.dmg" in OSX folder. See note below for Mac OS.
7. In the "Client Control" UI, select the serial port for the WICED
   Evaluation Board and open the port. (See note below.)
8. Once the "Client Control" app is able to communicate with the embedded Bluetooth
   application, the application UI will be enabled. Then use the UI to perform
   Bluetooth operations.
9. Bluetooth application and protocol traces will be displayed in BTSpy application
   (wiced_tools\BTSpy)

Note for selecting serial port:
-------------------------------
1. The serial port on Windows OS corresponds to the WICED HCI UART COM port. On
   Linux OS it corresponds to /dev/ttyWICED_HCI_UARTx. On Mac OS the WICED HCI
   UART port is usually the lower numbered of the two /dev/tty-usbserial* ports.
2. If after opening the serial port the UI is not enabled, check if the correct
   serial port is selected, embedded appplication supports WICED HCI transport
   and that the baud rate selected in the UI matches the baud rate specified in
   the embedded application. See the baud rate specified in the
   wiced_transport_cfg_t structure of the embedded application.
3. See the chip specific readme.txt for the baud rate of the WICED board.

Note for Mac OS:
----------------
1. The highest baud rate supported in Mac xOS is 3,000,000 so all WICED embedded
   apps that want to communicate with the Mac ClientControl app must be built to
   use a baud rate of 3,000,000 max. (See the baud rate specified in the
   wiced_transport_cfg_t structure of the embedded application.)
2. Application downloads on WICED platforms may fail on certain OSX versions due
   to an incompatibility between the OSX version and the installed driver for
   the FTDI USB to serial chip.  On OSX versions 10.10 and earlier, a specific
   version of the FTDI driver must be installed using the instructions available
   here: https://learn.sparkfun.com/tutorials/how-to-install-ftdi-drivers/mac
   On OS X versions 10.11 and greater, the Apple version of the FTDI driver must
   be used, and any previous instance of the FTDI version of the driver must be
   removed from the system, by executing the following command in xterm:
   > sudo rm -rf /Library/Extensions/FTDIUSBSerialDriver.kext /System/Library/Extensions/FTDIUSBSerialDriver.kext
   Reboot the system after performing the rm command
3. Opening multiple instances of the app on MacOS is not as easy as it is in Windows,
   you can’t simply double click on the app like in Windows.  This is a limitation of
   the OS not the ClientControl app.  It is possible however to run multiple instances
   on MacOS using one of these two methods:
   Method 1:
      1.  Open .dmg, drag the clientcontrol application to desktop or any folder.
      2.  Open a terminal window; go to the location of ClientControl.app; Repeat below for each new instance of ClientControl application.
      3.  $open -n ClientControl.app
   Method 2:
      1.  Drag app from .dmg to desktop or other folder
      2.  Right click on app and select ‘Duplicate’
      3.  Open the duplicated app
4. If you see an error message when trying to execute ClientControl saying it is damaged
   and cannot be opened, perform the following step to workaround:
   Open an xterm window and execute:
      xattr -cr <MT_WORKSPACE_ROOT>/Utils_BLE_BT_Host_Apps/bt-ble/client_control/OSX/ClientControl.app
   where <MT_WORKSPACE_ROOT> is the location of your workspace. By default
   this is the mtw folder in the user home directory.

Instructions for building the application
-----------------------------------------
To make a change to the sample "Client Control" application and rebuild the
application, follow these steps:
1. On Linux OS, install gcc, gcc-c++, kernel-devel-$(uname -r),
   "Development Tools", "Development Libraries", libGLU-devel, before installing
   QT Creator
2. Download installer for QT Creator from https://www.qt.io/download/
3. On Windows OS, select the option for MinGW 32 bit compiler
4. Open the QT project file "ClientControl.pro" with QT Creator
5. Build the QT project
6. On Windows OS, ws2_32.lib path might need to be adjusted in the project file
   (ClientControl.pro).

User Interface
--------------
The UI is dividied in three sections:
1. Device manager UI (area above the tabs) is for controlling the local device
   settings as well as discovering peer devices.
2. The tabs are Bluetooth profiles or features that provide specific functions
   as suggested by the tab name.  Each tab is usable only with certain embedded
   applications and peer devices as mentioned in the 'Setup' description below
   for each tab.
3. The tracing UI (area below the tabs) shows Client Control application traces.
   It also can log traces to a file.  Use the 'Browse' button to select the file
   to log to and enable 'Log to file' check box. You can add custom traces using
   the 'Add Trace' button.

Device Manager:
++++++++++++++
- Open port
  This button is used to open or close serial port used by by WICED Eval board.
  Opening the serial port is the first step for using the application downloaded
  to the board by WICED SDK.
  Select the "Baud rate" and "Flow Control" options as appropriate to match the
  settings for the embedded app.
- Pair-able
  This option enables or disables pairing from a peer device.
- Connectable
  This option enables or disables connectablity from a peer device.
- Discoverable
  This option enables or disables discoverability from a peer device.
- Reset Device
  Close the serial port and reset the device. The embedded application may need
  to to be downloaded again.
- BLE Discovery
  Search for BLE devices.
- BR/EDR Discovery
  Search for BR/EDR devices.
- Unbond
  Un-pair the selected device. Paired devices will show 'link' icon.
- Patch file download
  *** NOTE *** 20719/20721/208xx devices do not support the DIRECT_LOAD option.
  *** NOTE *** CYW9M2BASE-43012BT board *ONLY* supports the DIRECT_LOAD option.
  Normally the embedded application is downloaded to the board through WICED
  SDK (using a make target with the 'download' command).  If desired, the
  Client Control application can be used to directly load an embedded application
  built with DIRECT_LOAD option.
  This produces a patch (.hcd) file in the build folder containing
  the embedded application.
  To download a patch file to a board, click the 'Browse' button to select the
  hcd file.
  The 'Local BD Address' will be programmed to the board during the patch file
  download process.
  Before clicking on the 'Download' button, close the serial port if open and
  select the desired port number in the drop down box. Prepare the WICED Eval
  board using the reset/recovery procedure.
  Then click the 'Download' button.

Tabs and Local Application Functionality:
+++++++++++++++++++++++++++++++++++++++++
See the README.txt file in a appropriate apps\ subfolder in WICED SDK for
detailed information on each of the local apps.

Note:  Local apps listed may not be included in all versions of WICED SDK.

AV Source (A2DP Source):
+++++++++++++++++++++++
Setup: Local apps - 'watch'.
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
Setup: Local apps - 'watch'
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

AV Sink:
++++++++
Not implemented in the current release

AVRC CT (AVRCP Controller):
++++++++++++++++++++++++++
Setup: Local apps - 'watch'
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
Setup: Local apps - audio_gateway
Peer device - headset, speaker, car-kit supporting hands-free profile
- Connect
  Connect to a peer device supporting the HF profile
- Disconnect
  Disconnect with a peer device
- Audio Connect
  Open a SCO audio channel with a peer device

Hands-free:
+++++++++++
Setup: Local apps - 'hci_handsfree'
Peer device - phone or device supporting the Audio Gateway profile
- Connect
  Connect to a peer device supporting the AG profile
- Disconnect
  Disconnect with a peer device
- Connect Audio
  Open a SCO audio channel with a peer device
- Hangup, Answer, Redial, etc.
  These controls send the HF profile AT commands for various operations.

Serial Port Profile:
+++++++++++++++++++
Setup: Local apps - 'spp'
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

HID Host:
+++++++++
Setup: Local apps - 'hci_hid_host' for BR-EDR or
       'hci_ble_hid_host' for BLE HOGP
Peer device - BT keyboard or mouse
- Connect
  Connect to a HID device such as a keyboard or mouse. After connection, the
  input from the HID device will be displayed in BTSpy.exe
- Disconnect
  Disconnect from a peer device
- Get Desc
  Get Descriptor from the peer device. The output is displayed in BTSpy.exe.
- HID Protocol
  Set the protocol as Report mode or Boot mode

HID Device:
++++++++++
Setup: Local apps - 'hci_hid_device' for BR-EDR or
       'hci_ble_hid_dev', BLE remote control/mouse/keyboard for BLE HOGP
Peer device - Windows PC or any HID host
- Enter Pairing Mode
  Sets the local device to pair-able
- Connect
  Connect with a HID host
- Send Key
  Sends the specified key from the drop down with options such as button up,
  Caps Lock, etc.
- Send Report
  Send report for Interrupt or Control channel.

GATT:
+++++
Setup: Local apps - 'watch'
Peer device - BLE device or iPhone Light Blue app, Android BLE app, etc.
GATT controls are provided for advertisements, discovering services, connecting
to a GATT server, reading/writing values of handles, and discovering
characteristics and descriptors of handles.

HomeKit:
++++++++
Setup: Local apps - 'homekit/lightbulb', 'homekit/lock'
Peer device : iPhone, iOS 10.2, My Home app
Note: This app is available only for Apple MFI licensees
Sample applications are provided for 'Lock' and 'Light Bulb' control.

iAP2:
+++++
Setup: Local apps - 'hci_iap2_spp'
Peer device: iPhone, iOS 10.2 or SPP server capable device
Note: This app is available only for Apple MFI licensees
iAP2 app sends and receives data over RFCOMM for iOS devices, or over SPP for
non-iOS devices. For UI description, see "Serial Port Profile" above.

BLE Serial Gatt:
++++++++++++++++
Setup: Local apps - 'hci_serial_gatt_serivce'
Peer device : Windows 10, Android or iPhone running 'peerapps' application
found in the WICED SDK under 'serial_gatt_serivce' folder.
This application uses a Cypress BLE GATT service to send and receive data
over GATT. This is similar to Serial Port Profile application.
For UI description, see "Serial Port Profile" above.

Phonebook Client
++++++++++++++++
Apps : pbap_client
Peer device - phone supporting PBAP server (any recent iPhone or Android phone)
- From the phone, initiate device discovery and find the pbap_client application
  and perform pairing.
- From the Client Control UI, select the paired phone and click Connect
- Use the UI to download phone book contacts, and incoming, outgoing or missed
  calls.
Note: only the first 100 records will be displayed. The application can be
updated to get more records if desired.

OPP Server
++++++++++
Apps : opp_server
Peer device : PC, Android phone, etc supporting OPP Client profile.

From peer device, find the 'OPP server' device, pair and send file.

- Disconnect
  Disconnects from OPP client
