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
 * M1900 - Restart RTS Display variables & reload from EEPROM
 *
 * Default behavior is to set back all RTS display variables to
 * power-up default state & reload saved EEPROM.
 *
 */
#if ENABLED(RTS_AVAILABLE)
  void GcodeSuite::M1900() {
    // Check the integrity of data offsets.
    TERN_(RTS_DEBUG, SERIAL_ECHOLNPGM("RTS => Restarting printer now..."));
    rtscheck.RTS_Restart();
  }
#endif
