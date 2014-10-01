GliderWinch
===========
Master Controller: 

Implements the master controller for semi-automated winch control, using a ST Discovery F4 board (STM32F407 processor) with a custom Control Panel that has LEDs, LCD, switches, and a control lever.  The master controller communicates with other units via CAN, and the PC via either a CAN<->PC gateway, or usb-serial port.

The gcc launchpad toolchain is currently being used: https://launchpad.net/gcc-arm-embedded

Flashing is via openocd: http://openocd.sourceforge.net/

To set ST-LINK permissions, compile and flash--
cd mc/etmc/etmc0/trunk; ./p; ./mm && make flash



