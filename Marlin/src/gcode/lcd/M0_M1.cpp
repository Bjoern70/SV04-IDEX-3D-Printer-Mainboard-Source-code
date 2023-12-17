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

#include "../../inc/MarlinConfigPre.h"

#if HAS_RESUME_CONTINUE

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#include "../../module/planner.h" // for synchronize()
#include "../../MarlinCore.h"     // for wait_for_user_response()

#if HAS_LCD_MENU
  #include "../../lcd/marlinui.h"
#elif ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD_ENHANCED)
  #include "../../lcd/e3v2/enhanced/dwin.h"
#elif ENABLED(RTS_AVAILABLE)
  #include "../../lcd/e3v2/creality/LCD_RTS.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../../feature/host_actions.h"
#endif


/**
 * M0: Unconditional stop - Wait for user button press on LCD
 * M1: Conditional stop   - Wait for user button press on LCD
 */
void GcodeSuite::M0_M1() {
  millis_t ms = 0;
  if (parser.seenval('P')) ms = parser.value_millis();              // Milliseconds to wait
  if (parser.seenval('S')) ms = parser.value_millis_from_seconds(); // Seconds to wait

  planner.synchronize();

  #if HAS_LCD_MENU

    if (parser.string_arg)
      ui.set_status(parser.string_arg, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        ui.reset_progress_bar_timeout();
      #endif
    }

  #elif ENABLED(EXTENSIBLE_UI)
    if (parser.string_arg)
      ExtUI::onUserConfirmRequired(parser.string_arg); // Can this take an SRAM string??
    else
      ExtUI::onUserConfirmRequired_P(GET_TEXT(MSG_USERWAIT));
  #elif ENABLED(DWIN_CREALITY_LCD_ENHANCED)
    DWIN_Popup_Confirm(ICON_BLTouch, parser.string_arg ?: GET_TEXT(MSG_STOPPED), GET_TEXT(MSG_USERWAIT));

  #elif ENABLED(RTS_AVAILABLE)
     /*
      * //show user dialog to stop/resume after pause
    rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
    for (int j = 0; j < 20; j ++)
    {
      rtscheck.RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
    }
    if (parser.string_arg)
       rtscheck.RTS_SndData(parser.string_arg, PRINT_FILE_TEXT_VP);
    else
      rtscheck.RTS_SndData(PSTR("PRESS [NO] TO STOP"), PRINT_FILE_TEXT_VP);

    rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
    */
    //skip display screen call to prevent blocking when M0 is called without a running print file
    rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
    dwell(250);
    rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
    dwell(250);
    rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
    if (parser.string_arg) {
      SERIAL_ECHO_START();
      SERIAL_ECHOLN(parser.string_arg);
    }

  #else

    if (parser.string_arg) {
      SERIAL_ECHO_START();
      SERIAL_ECHOLN(parser.string_arg);
    }

  #endif

  TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_USER_CONTINUE, parser.codenum ? PSTR("M1 Stop") : PSTR("M0 Stop"), CONTINUE_STR));

  TERN_(HAS_RESUME_CONTINUE, wait_for_user_response(ms));

  TERN_(HAS_LCD_MENU, ui.reset_status());
}

#endif // HAS_RESUME_CONTINUE
