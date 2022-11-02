# MOTOMAN DX200 Motion Primitive Control

Controls DX200 with FTP Ethernet functions

## Settings
* Maintaince Mode: Press ** and boot
* Set IP: 
* Management Mode: {MAIN MENU} -> {SECURITY} -> {MANAGEMENT MODE}, password is 999999999
* Command Remote: {MAIN MENU} -> {IN/OUT} -> {PSEUDO INPUT SIGNAL} -> *CMD REMOTE SEL*, press [INTERLOCK]+[SELECT]
* Enable FTP: {Main Menu} -> {EX.MEMORY}
* PC IP: Change computer's IP to manual, under the same subnet as DX200's IP

Username is `ftp` and arbitrary password, use the IP set above.

## Usage
Connect Ethernet from DX200 to computer, and make sure you can ping DX200 from your computer. 
`python dx200_motion_program_exec_client.py`, the program will generate `.JBI` file, send to DX200 and execute.
