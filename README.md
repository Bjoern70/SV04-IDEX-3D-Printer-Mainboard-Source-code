# Introduction

Based on the great work of bphillips09 & johncarlson21 and by inspiration of Jyers UI some changes were applied to the Sovol SV04 Marlin firmware:

 - added bed levelling map visualization for 8x8 grid
 - added soft power off button functionality (supports separate suicide switch to execute g-code command M81)
 - hanged preheat settings from filament type ABS to PETG
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
 - fixed pause deadlock bug
 - fixed user interface deadlock bug
 - fixed unpredictable movement behaviour in move menu & added warning in case of moving axes uncalibrated axes
 - longer filenames supported.

This SV04 Marlin firmware requires matching touchscreen software that is available here: 
https://github.com/Bjoern70/SV04-Touchscreen/releases/tag/v1.14.3

Leveling map screen:

![BedLeveling](https://user-images.githubusercontent.com/72707632/219985619-2219b7f9-08b4-4207-ab1e-c8c373784af7.JPG)


# About the SV04 IDEX 3D Printer

Sovol SV04 IDEX 3D Printer comes with dual independent direct drive extruders, both are metal titan style extruders,  
which make it to print with two materials very easily such as TPU, TPE, HIPS, ABS, PETG, WOOD, PC, PA, PVA, ASA. 
Combined with auto-leveling and extruder 2 leveling knob solved the a lot of IDEX users’ leveling trouble. 
Two extruders work independently will save time， save money， and save effort in 3d printing production, 
The large build volume satisfy most of using demand compared with the full enclosed IDEX 3d printer. 
There are  4 print modes，single-mode，dual-mode, duplicate mode, and mirror mode. 

# Related tutorials 

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
