# Tatrabot-mapping
Tatrabot is mapping area using ultrasonic sensor

## Requirements:

* Tatrabot Robot with STM32F103
* ChibiStudio 19 (or compatible)
   https://sourceforge.net/projects/chibios/files/ChibiStudio/

## Building:

* Open ChibiStudio using start_gcc63.bat
* Switch workspace to Workspace176
* Unzip Tatrabot-mapping project to some directory
* Import project from Chibistudio from that directory, remember to tick "copy projects to workspace" option
* Build (CTRL-B)

## Running:

* Connect Tatrabot to USB port, check port number in Device Manager (i.e. COM3)
* Run stm32flash -w ch.hex com3<br>from build directory of the project in workspace176

