# MOTOMAN DX200 Motion Primitive Control

Controls DX200 with FTP Ethernet functions

## Settings
* Maintaince Mode: Press ** and boot
* Set IP: (Maintaince Mode) -> HOST SETUP -> IP ADDRESS -> `192.168.1.31`
* Management Mode: {MAIN MENU} -> {SECURITY} -> {MANAGEMENT MODE}, password is 999999999
* Safety Mode: {MAIN MENU} -> {SECURITY} -> {SAFETY MODE}, password is 555555555
* Command Remote: {MAIN MENU} -> {IN/OUT} -> {PSEUDO INPUT SIGNAL} -> *CMD REMOTE SEL*, press [INTERLOCK]+[SELECT]
* Enable FTP: {Main Menu} -> {EX.MEMORY} (Username is `ftp` and arbitrary password, use the IP set above.)
* PC IP: Change computer's IP to manual, under the same subnet as DX200's IP
* Pulse to Deg: {Main Menu} -> {SETUP} -> {FUNCTION ENABLE}. Set ALL AXES ANGLE DISP Function to VALID. {ROBOT} -> {CURRENT POSTION}: 
Toggle DISPLAY to Pulse/Absolute Degrees
* Tool Data: Under Safety Mode,  {Main Menu} -> {ROBOT} -> {TOOL}

## Welder Setting
Set IP and Gateway of Fronius Welder to communicate with DX200
* Fronius IP: Deselect DHCP, IP Address: `192.168.1.55`, Standard Gateway: `192.168.1.31`
* DX200 (Management): Weldcom Options -> ARC DIGITAL I/F FUNC. -> Power Source IP Address: `192.168.1.51`, HTTP POWER SOURCE IP ADDRESS: `192.168.1.55`

Welder Interface could be accessed through browser.


## Usage
Connect Ethernet from DX200 to computer, and make sure you can ping DX200 from your computer. 
`python dx200_motion_program_exec_client.py`, the program will generate `.JBI` file, send to DX200 and execute.
