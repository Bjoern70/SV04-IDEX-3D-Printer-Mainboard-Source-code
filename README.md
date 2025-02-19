# Introduction

Based on the great work of bphillips09 & johncarlson21 and by inspiration of Jyers UI some changes were applied to the Sovol SV04 Marlin firmware:

 - added bed levelling map visualization for 8x8 grid
 - added soft power off button functionality (supports separate suicide switch to execute g-code command M81)
 - changed preheat settings from filament type ABS to PETG
 - corrected tramming pitch
 - increased probing speed
 - added simple DGUS display onboard buzzer handling: g-code command M300 will play beep sound omitting parameters
 - fixed print mode hickup when pausing / resuming
 - cleaned host temperature report
 - fixed M1 stop wait loop bug
 - fixed toolchange and dualmode override bug at end of file
 - adjusted extruder movement speeds
 - cleaned up STOP, PAUSE & FINISH PRINT processing
 - merged "standard" and "SIZZLING" software version
 - changed nozzle temperature limits to maximum 300°C - be sure to have a matching heat blocks & use at your own risk!
 - temperatures above OEM maximum of 260°C will be highlighted red
 - fixed deadlock bug after filament runout and resume
 - fixed pause + user interface deadlock bugs
 - fixed unpredictable movement behaviour in move menu & added warning in case of moving axes uncalibrated axes
 - added basic M423 "X Twist Compensation" implementation: compensation process is currently not supported via display
 - fixed toolhead selection issue: selecting single mode will activate corresponding toolhead
 - cleaned up filename + directory colors
 - fixed M423 implementation: process is supported via host terminal
 - fixed x twist compensation probe bug
 - added adjustment of fan speed &  flow rate during printing
 - added extra settings dialog: PID tuning for nozzles and hot-bed (Auto-PID & manual input are supported)
 - added extra settings dialog: extruder E-step adjustment
 - added extra settings dialog: define presets (pre-heat temperatures, stepper deactivation time and auto power-off enable/disable)
 - changed filename display to rolling text with adjustable text speed; first 130 chars of filenames are displayed, including umlaute
 - minor changes in display handler iot avoid display hickups during host print
 - added new G-Code command M1900 "Restart RTS display variables & reload from EEPROM"
 - added new G-Code command M1901 "Call RTS display screen directly"
 - more changes in display handler iot avoid display hickups during host print
 - modified G-Code command M1901
 - fixed display software version in printer info screen
 - fixed offset float to integer conversion bug
 - cleared preset values for X- and Y-offset: both offsets will be 0.0 after EEPROM reset
 - added X-axis homing after tramming
 - modified tramming points & Z-alignment probe points for higher precison
 - fixed touchscreen E-step adjustment not recognized bug
 - fixed right extruder not moving bug in refuel & extruder E-step adjustment screens.


This SV04 Marlin firmware requires matching touchscreen software v1.15.2 that is available here:
https://github.com/Bjoern70/SV04-Touchscreen/releases/tag/v1.15.2
(Yes, the display software has not been touched since firmware v1.15.2)

# About the SV04 IDEX 3D Printer

Sovol SV04 IDEX 3D Printer comes with dual independent direct drive extruders, both are metal titan style extruders,
which make it to print with two materials very easily such as TPU, TPE, HIPS, ABS, PETG, WOOD, PC, PA, PVA, ASA.
Combined with auto-leveling and extruder 2 leveling knob solved the a lot of IDEX users’ leveling trouble.
Two extruders work independently will save time， save money， and save effort in 3d printing production,
The large build volume satisfy most of using demand compared with the full enclosed IDEX 3d printer.
There are  4 print modes，single-mode，dual-mode, duplicate mode, and mirror mode.

# Related tutorials

- Firmware wiki.  [Click here](https://github.com/Bjoern70/SV04-IDEX-3D-Printer-Mainboard-Source-code/wiki).
- User Manual.  [Click here](https://drive.google.com/file/d/1QpIDenqIKmsA2blAhKkOxhp2SKL8hwoI/view).
- Firmware Download. [Click here](https://sovol3d.com/pages/download).
- SD Card Content, including tutorials and stl files. [Click here](https://drive.google.com/drive/folders/1LNCtBA045Xo5z7Gd4n1M2aDEVry_wHCH?fbclid=IwAR3y-_OYa_VTG4Bz68GR5JdGMLE_ROVfIQRevPL4WpTEDLQ1nXzUEtDMaqs)

# How to buy

on Sovol Official Website:  https://sovol3d.com/products/sv04

# Code Editor

PlatformIO IDE for VSCode

# Technical Specifications

- Extrusion Tech: Independent Dual Direct Drive  Metal Extruder (IDEX)
- Build Size: 300mm x 300mm x 400mm
- Build Plate Dimension: 310mm*320mm
- Printer Dimensions: 653mm(L) *625mm(W) *803mm(H)(Include filament holder)
- Package Dimensions: 735mm (L) x 700mm (W) x 290mm (H)
- Net Weight: 15kg
- Layer Resolution: 0.1mm
- Positioning Accuracy: X/Y 0.012mm Z 0.001mm
- Supported Print Materials: PLA, TPU, TPE,HIPS, ABS, PETG, WOOD, PC, PA, PVA, ASA.
- Print Speed: 20~120mm/s(suggested 60mm/s)
- Stock Nozzle Diameter: 0.4 mm
- Operational Extruder Temperature: Max 260ºC <=== PLEASE ENSURE THAT YOUR HARDWARE SUPPORTS TEMPERATURES ABOVE!
- Input Formats: .STL, .OBJ, DAE, AMF
- Ambient Operating Temperature:8ºC - 40ºC
- Connectivity: SD Card or Data Cable
- 7 Step Motors: 42-40 Step Motor*1 42-34 Step Motor*4 42-28 *2

# Sovol Community

- Sovol Forum website:  https://forum.sovol3d.com/
- German Sovol users forum website:  https://forum.drucktipps3d.de/forum/board/42-sovol/
