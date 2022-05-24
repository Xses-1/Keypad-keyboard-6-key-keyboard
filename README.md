# Arrow Pad Keyboard


<img src="https://github.com/Xses-1/Keypad-keyboard-6-key-keyboard/blob/main/Peek%202022-04-15%2017-38.gif">
<img src="https://github.com/Xses-1/Keypad-keyboard-6-key-keyboard/blob/main/STM042/Schematic_VIM%20_%20arrow%20keyboard%20STM32F042_2022-05-24.png">
<img src="https://github.com/Xses-1/Keypad-keyboard-6-key-keyboard/blob/main/STM042/Bottom.png">
<img src="https://github.com/Xses-1/Keypad-keyboard-6-key-keyboard/blob/main/STM042/Top.png">


There are 2 versions of PCB design due to chip shortage.
* STM32F042F
* STM32F042G


Directory "Firmware" contains usbd_hid.c file which contains custom USB descriptor, usbd_hid.h which contains USB configuration, Keypad_keyboard.ioc which is configuration file for STM32Cube IDE, and main.c which is self explenatory.


All THT compoents should be on the top side (without solder mask) and all the SMD on the bottom side. U4 is ST-Link programmer socket U3 is USB socket.


Directory "Case" contains Keypad_Keyboard_Case.FCStd which is FreeCAD project and prepared STL files that can be directly sliced and printed.
