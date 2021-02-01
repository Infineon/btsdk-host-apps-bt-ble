# btsdk-host-apps-bt-ble

### Overview

This repo contains the ClientControl app for Bluetooth LE and BR/EDR embedded apps, and demonstrates use of the WICED HCI protocol to control embedded apps. Binary and source code are included.

The following apps use ClientControl:

- Audio: watch, audio_gateway
- HID: ble\_mouse, ble\_keyboard, ble\_remote
- LE: anc, ans, hrc, hrs, bac, bas, le\_coc
- RFCOMM: pbap\_client, map\_client, opp\_server

Any embedded app that configures traces over WICED HCI would also need to use the ClientControl app to view traces in the BTSpy utility.

Supported OS: Windows, Linux, and Android.
