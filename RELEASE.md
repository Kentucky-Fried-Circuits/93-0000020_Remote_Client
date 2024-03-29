93-0000020 Remote Client 
--------------
Release Notes
--------------
## V1.7.0
### Minor changes
TEST Added support for ATSC cabinet

### Bugfixes
TEST doesn't crash if the device is not connected

## V1.6.0
### Minor changes
Some incomplete changes for Magnethereal.

### Bugfixes
Fixed: Select PRO-Verter 5000-AFF1. Command Entry changes to Absorb exit amps. Pres "Run". Program crashes with unhandled exception "'NoneType' has no attribut 'write'.

### Known Issues
FIXME Attempting to "Run Script" on a file that doesn't exist generates an unhandled exception
TODO use sg.print()/sg.cprint() method to write to multiline for cleaner code
TODO Add capability to clear filebrowser history.
TEST exit cleanly on window close
TODO save window size and position
TODO separate Read and Write input text fields
TODO catch errors to window and eliminate console window
TODO crashes app on USB disconnection
TODO read the log in a task separate from the GUI read loop so we can separate the read interval from the reporting interval.
TODO rename Save As to something like "Log file name"
TODO Add support for other RS485 adapters. Currently assumes a Silicone Labs CP210x-compatible USB to serial chip.

## V1.5.0
Added selection for products: 24VDC PRO-Verter 5000-120 AR-4034 (20-0104034)
See V1.4.0 for KNOWN ISSUES which all persist in this version.

## V1.4.0
Renamed from "HyPR 6000 Client" to "Remote Client"
Added selection for products: 20-0104124 24VDC HyPR 6000 and 20-0104033 24VDC Pro-Verter 5000-220 AFF1
Added support for remote monitoring and remote control of 20-0104033
Moved to GitHub for official repository

### KNOWN ISSUES
Pressing "Run Script" when not connected crashed the program.
Unable to build CAN bus support into exe using pyinstaller.

## V1.3.0
BUILD 220809
FIXME null characters at beginning of datalog
Added support for new registers in Control Board Firmware 1.2.2 and RM/AGS version 2.2.0
Added then disabled CAN bus monitoring of pid_input, pid_output and pid_set_point. Works, but it's not clean.
Adopts semantic versioning
Partially updated to 99-0000011 V1.2.2 API
Changes baud to 57600.
Improved exception handling
Moved to Semantic Versioning per Technology Handbook.
TODO move logging to a thread
TODO on Read, reduce # of Registers to legal value instead returning None
TODO Added deciphering of node address
TODO stop PID logger on CAN window exit.
FIXME pyinstaller fails trying to use Matplotlib and can.interfaces.pcan.basic. CAN feature commented out.

## V1.2
Adds datalogging to file.
Adds additional decoding of CAN messages
FIXED Long wait if serial adapter can't find a server.

## V1.01 BUILD 20220216
FIXED bug where script would report no errors even if there were errors
FIXED issue where not having the ENG-00000480 cable caused a crash with no error message


V1.0 BUILD 20220210
Runs scripts
Added ASSERT capability to script
Added a logging window

License
--------
Portions of this software are licensed under the Apache License, Version 2.0.
93-0000020.py is (C) 2022 Solar Stik, Inc. All rights reserved.

