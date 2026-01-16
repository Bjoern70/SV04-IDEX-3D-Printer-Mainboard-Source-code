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
#include "../../module/stepper.h"
#include "../../lcd/marlinui.h"
#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/e3v2/creality/LCD_RTS.h"
#endif

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
void GcodeSuite::G4() {
  millis_t dwell_ms = 0;

  if (parser.seenval('P')) dwell_ms = parser.value_millis(); // milliseconds to wait
  if (parser.seenval('S')) dwell_ms = parser.value_millis_from_seconds(); // seconds to wait

  planner.synchronize();
  #if ENABLED(NANODLP_Z_SYNC)
    SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
  #endif

  if (!ui.has_status()) LCD_MESSAGEPGM(MSG_DWELL);

  dwell(dwell_ms);
  #if ENABLED(RTS_AVAILABLE)
    if (rtscheck.RTS_presets.debug_enabled)  //get debug state
    {
      //Debug enabled
      SERIAL_ECHOLNPGM("RTS =>  G4. Return to display screen #", rtscheck.RTS_currentScreen);
      sprintf(rtscheck.RTS_infoBuf, "G4: Last[%d] Goto Cur[%d] waitW=%d DXC=%d", rtscheck.RTS_lastScreen, rtscheck.RTS_currentScreen, RTS_waitway, dualXPrintingModeStatus);
      rtscheck.RTS_Debug_Info();
    }
    rtscheck.RTS_SndData(ExchangePageBase + rtscheck.RTS_currentScreen, ExchangepageAddr);//Display update
  #endif
}
