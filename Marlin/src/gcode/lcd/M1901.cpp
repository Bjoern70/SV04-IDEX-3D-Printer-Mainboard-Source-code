/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"

#include "../../MarlinCore.h"   // for marlin_state

#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/e3v2/creality/LCD_RTS.h" // for RTS_restart
#endif

/**
 * M1901 - Call RTS display screen directly (Requires RTS_AVAILABLE)
 *
 * M1901          - report current screen number
 *
 * M1901 S<int>   - Display RTS screen #S
 *                  #S  : Primary screen      Secondary screens
 *                  ******************************************************************
 *                    0 :                     Boot screen
 *                    1 : Main screen
 *                    2 : File browser
 *                    8 :                     Notify filament change
 *                    9 :                     Notify print finished
 *                   11 : Info while printing
 *                   12 : Info while pausing
 *                   14 : Adjustment while printing
 *                   15 : Temperatures
 *                   16 : Left nozzle settings
 *                   17 : Right nozzle settings
 *                   18 : Hot-bed settings
 *                   19 :                     Query disable stepper
 *                   20 :                     Notify filament runout
 *                   21 : Settings
 *                   22 : Leveling
 *                   23 : Filament refuel
 *                   24 :                     Notify left nozzle temperature
 *                   25 :                     Notify right nozzle temperature
 *                   26 :                     Info while refueling
 *                   27 :                     Notify nozzle heating
 *                   28 : Aux leveling
 *                   29 : Move 10mm
 *                   30 : Move 1mm
 *                   31 : Move 0.1mm
 *                   32 :                     Info while auto homing
 *                   33 : Printer info
 *                   34 : Print mode
 *                   35 : Set hot-end offset
 *                   36 :                     Query resume print
 *                   38 :                     Info while auto-leveling
 *                   39 :                     Query filament change
 *                   40 :                     Info while processing
 *                   42 :                     Query cool down
 *                   46 :                     Notify SD card
 *                   47 :                     Query SD card
 *                   48 :                     Query EEPROM
 *                   50 :                     Numeric keypad
 *                   52 :                     Warning thermal runway
 *                   53 :                     Warning heating failed
 *                   54 :                     Warning thermistor error
 *                   55 :                     Warning auto-home failed
 *                   56 :                     Query start print
 *                   57 : Tramming
 *                   58 : Move 100mm
 *                   60 :                     Query pause print
 *                   62 :                     Query low temperature
 *                   81 : Mesh view
 *                   86 :                     Query restart printer
 *                   88 :                     Query power off
 *                   90 : Extra settings
 *                   91 : Left extruder E-steps
 *                   92 : Right extruder E-steps
 *                   93 : Left nozzle PID
 *                   94 : Right nozzle PID
 *                   95 : Hot-bed PID
 *                   96 : Manual PID
 *                   97 : Presets
 *                   98 :                     Query low temperature left
 *                   99 :                     Query low temperature right
 *                  101 :                     Query low temperature left
 *                  102 :                     Query low temperature right
 *                  103 :                     Query low temperature hot-bed
 *                  any : Other screens than above will likely freeze RTS display
 *
 * M1901 R<bool>   - Force display RTS screen #S & reset screen locks
 *                   S<int>   - Display RTS screen #S as above
 * Examples:
 *
 *   M1901 R1 S1   - Force main screen
 *
 *   M1901 S11     - Display in-printing information screen
 *
 *   M1901         - Report current screen #
 */
#if ENABLED(RTS_AVAILABLE)
  void GcodeSuite::M1901() {
    if (parser.seen('S'))
    {
      const int8_t screen = parser.byteval('S');
      if ((screen>=0) || (screen<=140))
      {
        if (parser.seenval('R'))
        {
          TERN_(RTS_DEBUG, SERIAL_ECHOLNPGM("RTS => Force call screen #",screen));
          RTS_currentScreen = screen;
          RTS_waitway = 0;  //reset screenlock
          RTS_heatway = 0;   //reset screenlock
          rtscheck.RTS_SndData(ExchangePageBase + screen, ExchangepageAddr);;
        }
        else
        {
          TERN_(RTS_DEBUG, SERIAL_ECHOLNPGM("RTS => Call screen #",screen));
          RTS_currentScreen = screen;
          rtscheck.RTS_SndData(ExchangePageBase + screen, ExchangepageAddr);
        }
      }
    }
    else
    {
      //Report RTS_currentScreen number
      SERIAL_ECHOLNPGM("Current RTS screen # is ",RTS_currentScreen);
    }
  }
#endif



