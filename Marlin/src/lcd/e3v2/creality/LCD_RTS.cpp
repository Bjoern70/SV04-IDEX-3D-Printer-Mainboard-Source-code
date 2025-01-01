#include "LCD_RTS.h"
#include <WString.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include "../../../MarlinCore.h"
#include "../../../inc/MarlinConfig.h"
#include "../../../module/settings.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"

#include "../../fontutils.h"
#include "../../../sd/cardreader.h"
#include "../../../feature/powerloss.h"
#include "../../../feature/babystep.h"
#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../gcode/queue.h"
#include "../../../gcode/gcode.h"
#include "../../../module/probe.h"

#include "../../../feature/bedlevel/abl/abl.h"

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../../../feature/pause.h"
  #include "../../../gcode/queue.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

#if ENABLED(RTS_AVAILABLE)
RTS_preset_data RTS_presets; // Initialized by settings.load()
float zprobe_zoffset;
float last_zoffset = 0.0;

const float manual_feedrate_X = MMM_TO_MMS(XY_PROBE_FEEDRATE);      //180*60 mm/min
const float manual_feedrate_Y = MMM_TO_MMS(XY_PROBE_FEEDRATE);      //180*60 mm/min
const float manual_feedrate_Z = MMM_TO_MMS(Z_PROBE_FEEDRATE_FAST);  // 16*60 mm/min
const float manual_feedrate_E = FILAMENT_CHANGE_FAST_LOAD_FEEDRATE; //     6 mm/s

int startprogress = 0;
bool sdcard_pause_check = true;

float current_position_x0_axis = X_MIN_POS;
float current_position_x1_axis = X2_MAX_POS;
int PrintFlag = 0;

int heatway = 0;
millis_t next_rts_update_ms = 0;
int last_target_temperature[4] = {0};
int last_target_temperature_bed;
char RTS_waitway = 0;
unsigned char Percentrecord = 0;

bool pause_action_flag = false;
int power_off_type_yes = 0;
// represents to update file list
bool CardUpdate = false;
bool hasSelected = false;
short previousSelectionIndex = 0;

extern CardReader card;
// represents SD-card status, true means SD is available, false means opposite.
bool lcd_sd_status;

char cmdbuf[20] = {0};
float Filament0LOAD = 0.0;
float Filament1LOAD = 0.0;

float EstepFeedRate = 100.0; //100 mm/min
float EstepFeedDistance = 100.0; //100mm
float ESteps0 = 415.0; // 415 steps/mm
float ESteps1 = 415.0; // 415 steps/mm
uint32_t ConvertLong = 0;
uint16_t ConvertH = 0;
uint16_t ConvertL = 0;
uint16_t TuneCycles = 5; // 5 cycles default
float PIDparamKp0 = 0.0; //Left head PID Kp param
float PIDparamKi0 = 0.0; //Left head PID Ki param
float PIDparamKd0 = 0.0; //Left head PID Kd param
float PIDparamKp1 = 0.0; //Right head PID Kp param
float PIDparamKi1 = 0.0; //Right head PID Ki param
float PIDparamKd1 = 0.0; //Right head PID Kd param
float PIDparamKpB = 0.0; //Hot-bed PID Kp param
float PIDparamKiB = 0.0; //Hot-bed PID Ki param
float PIDparamKdB = 0.0; //Hot-bed PID Kd param
char BackupPID=0;
float BackupKp = 0.0; //Backup Kp param
float BackupKi = 0.0; //Backup Ki param
float BackupKd = 0.0; //Backup Kd param
uint16_t FileScrollSpeed = 1;
char RTS_cyclesIcon = 0;
int RTS_currentScreen = 0;

// 1 for 0.1mm, 2 for 1mm, 3 for 10mm, 4 for 100mm
unsigned char AxisUnitMode;
float axis_unit = 10;
unsigned char AutoHomeIconNum;
RTSSHOW rtscheck;
int Update_Time_Value = 0;

bool PoweroffContinue = false;
char commandbuf[30];
bool active_extruder_flag = false;

static int change_page_number = 0;

char save_dual_x_carriage_mode = 0;

uint16_t remain_time = 0;

static bool last_card_insert_st;
bool card_insert_st;
bool sd_printing;
bool sd_printing_autopause;

inline void RTS_line_to_current(AxisEnum axis, float manual_feedrate_mms)
{
  if (!planner.is_full())
  {
    planner.buffer_line(current_position, manual_feedrate_mms, active_extruder);
  }
}

RTSSHOW::RTSSHOW()
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf, 0, sizeof(databuf));
}

void RTSSHOW::ShowFilesOnCardPage(int page) {
  if (page < 1)
  {
    page = 1;
  }
  else if (page > fileInfo.pages)
  {
    page = fileInfo.pages;
  }

  fileInfo.currentPage = page;

  char pageCount[3];
  char maxPages[3];
  itoa(page, pageCount, 10);
  itoa(fileInfo.pages, maxPages, 10);
  char statStr[7];
  strcpy(statStr, pageCount);
  strcat(statStr, "/");
  strcat(statStr, maxPages);

  for (int h = 0; h < 7; h++) {
    RTS_SndData(0, PAGE_STATUS_TEXT_VP + h);
  }

  RTS_SndData(statStr, PAGE_STATUS_TEXT_VP);

  int max = fileInfo.currentPage*5;
  int min = max-4;
  if (max > fileInfo.fileCount) {
    max = fileInfo.fileCount;
  }

  //for (int i = 0; i < 5*FileNameLen; i += FileNameLen) {
  for (int i = 0; i < 1280; i += 256)
  {
    for (int j = 0; j < FileNameLen; j++)
    {
      RTS_SndData(0, FILE1_TEXT_VP + i + j);
    }
  }
  uint16_t buttonIndex = FILE1_TEXT_VP;
  uint16_t textIndex = 0;

  //enumerate files

  for (int k = min-1; k < max; k++)
  {
    card.selectFileByIndex(k);
    char shortFileName[FileNameLen];
    strncpy(shortFileName, card.longest_filename(), FileNameLen);
    RTS_SndData(shortFileName, buttonIndex);

    if (EndsWith(card.filename, "GCO"))
    {
      RTS_SndData((unsigned int)0xA514, FilenameNature + (textIndex + 2) * 32); //gcode file
    }
    else
    {
      //change color if dir
      RTS_SndData((unsigned int)0x0400, FilenameNature + (textIndex + 2) * 32);
    }

    textIndex++;
    //buttonIndex += FileNameLen;
    buttonIndex += 256;
  }
}

void RTSSHOW::InitCardList() {
  for(int j = 0; j < MAXPATHNAMELENGTH; j++)
  {
    fileInfo.currentDir[j] = 0;
  }

  card.getAbsFilenameInCWD(fileInfo.currentDir);
  char shortDirName[FileNameLen];
  for(int j = 0; j < FileNameLen; j++)
  {
    RTS_SndData(0, SELECT_DIR_TEXT_VP + j);
  }

  hasSelected = false;

  strncpy(shortDirName, fileInfo.currentDir, FileNameLen);
  RTS_SndData(shortDirName, SELECT_DIR_TEXT_VP);

  fileInfo.fileCount = card.get_num_Files();
  fileInfo.pages = ((fileInfo.fileCount-1) / 5)+1;

  for (uint16_t i = 0; i < 4; i++)
  {
    delay(3);
    RTS_SndData((unsigned int)0xA514, FilenameNature + (i + 2) * 32);
  }

  ShowFilesOnCardPage(1);
}

void RTSSHOW::RTS_SDCardInit(void)
{
  if(RTS_SD_Detected())
  {
    card.mount();
  }
  if(CardReader::flag.mounted)
  {
    card.cdroot();
    InitCardList();
    lcd_sd_status = IS_SD_INSERTED();
  }
  else
  {
    if(sd_printing_autopause == true)
    {
      RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
      card.mount();
    }
    else
    {
      for(int j = 0;j < FileNameLen;j ++)
      {
        // clean screen.
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
      }
    }
  }
}

bool RTSSHOW::RTS_SD_Detected(void)
{
  static bool last;
  static bool state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if(tmp != last)
  {
    flag_stable = false;
  }
  else
  {
    if(!flag_stable)
    {
      flag_stable = true;
      stable_point_time = millis();
    }
  }

  if(flag_stable)
  {
    if(millis() - stable_point_time > 30)
    {
      state = tmp;
    }
  }

  last = tmp;

  return state;
}

void RTSSHOW::RTS_SDCardUpate(void)
{
  const bool sd_status = RTS_SD_Detected();
  if (sd_status != lcd_sd_status)
  {
    if (sd_status)
    {
      // SD card power on
      card.mount();
      RTS_SDCardInit();
    }
    else
    {
      card.release();
      if(sd_printing_autopause == true)
      {
        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
      }
      else
      {
        for(int j = 0;j < FileNameLen;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          RTS_SndData(0, SELECT_DIR_TEXT_VP + j);
        }
      }
    }
    lcd_sd_status = sd_status;
  }

  // represents to update file list
  if(CardUpdate && lcd_sd_status && RTS_SD_Detected())
  {
    card.cdroot();
    InitCardList();
    CardUpdate = false;
  }
}

void RTSSHOW::RTS_Init()
{
  SERIAL_ECHOLNPGM("RTS Init");
  //enable Auto Power Off
  autoPowerOffEnabled = RTS_presets.auto_power_off_enable; //get auto power-off preset
  if (autoPowerOffEnabled == true)
  {
    RTS_SndData(0, AUTO_POWER_OFF_ICON_VP);
    RTS_SndData(0, PRESET_AUTO_POWER_OFF_VP);
  }
  else
  {
    RTS_SndData(1, AUTO_POWER_OFF_ICON_VP);
    RTS_SndData(1, PRESET_AUTO_POWER_OFF_VP);
  }
  AxisUnitMode = 3;

  #if ENABLED(DUAL_X_CARRIAGE)
    save_dual_x_carriage_mode = dualXPrintingModeStatus; //get last mode before power-off
    SERIAL_ECHOLNPGM("Debug Mode: RTS Init; dualXPrintingModeStatus=",dualXPrintingModeStatus);

    if(save_dual_x_carriage_mode == 1)      //DXC_AUTO_PARK_MODE => DUAL MODE
    {
      RTS_SndData(1, PRINT_MODE_ICON_VP);   //DUAL MODE icon
      RTS_SndData(1, SELECT_MODE_ICON_VP);  //DUAL MODE icon
    }
    else if(save_dual_x_carriage_mode == 2) //DXC_DUPLICATION_MODE => COPY MODE
    {
      RTS_SndData(2, PRINT_MODE_ICON_VP);   //COPY MODE icon
      RTS_SndData(2, SELECT_MODE_ICON_VP);  //COPY MODE icon
    }
    else if(save_dual_x_carriage_mode == 3) //DXC_MIRRORED_MODE => MIRROR MODE
    {
      RTS_SndData(3, PRINT_MODE_ICON_VP);   //MIRROR MODE icon
      RTS_SndData(3, SELECT_MODE_ICON_VP);  //MIRROR MODE icon
    }
    else if(save_dual_x_carriage_mode == 4)
    {
      RTS_SndData(5, PRINT_MODE_ICON_VP);    //SINGLE MODE R icon
      RTS_SndData(5, SELECT_MODE_ICON_VP);   //SINGLE MODE R icon
    }
    else
    {
      RTS_SndData(4, PRINT_MODE_ICON_VP);    //SINGLE MODE L icon
      RTS_SndData(4, SELECT_MODE_ICON_VP);   //SINGLE MODE L icon
    }
  #endif
  last_zoffset = zprobe_zoffset = probe.offset.z;
  RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
  RTS_SndData((hotend_offset[1].x - X2_MAX_POS) * 100, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
  RTS_SndData(hotend_offset[1].y * 100, TWO_EXTRUDER_HOTEND_YOFFSET_VP);

  last_target_temperature_bed = thermalManager.temp_bed.target;
  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  last_target_temperature[1] = thermalManager.temp_hotend[1].target;
  feedrate_percentage = 100;
  RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
  /***************Flow_percentage*****************/
  RTS_SndData(planner.flow_percentage[0], HEAD0_FLOW_RATE_VP);
  RTS_SndData(planner.flow_percentage[1], HEAD1_FLOW_RATE_VP);
  /***************E-steps*****************/
  EstepFeedRate = 100.0; //100 mm/min
  RTS_SndData(EstepFeedRate*10, E_STEPS_FEED_RATE_VP);
  EstepFeedDistance = 100.0; //100mm
  RTS_SndData(EstepFeedDistance*10, E_STEPS_FEED_DIST_VP);
  ESteps0 = planner.settings.axis_steps_per_mm[3];
  ESteps1 = planner.settings.axis_steps_per_mm[4];
  ConvertLong = 0;
  ConvertH = 0;
  ConvertL = 0;
  /***************PID settings*****************/
  TuneCycles = 5; //5 cycles default
  RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP);
  RTS_SndData(0, TUNE_PROGRESS_ICON_VP);
  PIDparamKp0 = PID_PARAM(Kp, 0); //Left head PID Kp param
  PIDparamKi0 = unscalePID_i(PID_PARAM(Ki, 0)); //Left head PID Ki param
  PIDparamKd0 = unscalePID_d(PID_PARAM(Kd, 0)); //Left head PID Kd param
  PIDparamKp1 = PID_PARAM(Kp, 1); //Right head PID Kp param
  PIDparamKi1 = unscalePID_i(PID_PARAM(Ki, 1)); //Right head PID Ki param
  PIDparamKd1 = unscalePID_d(PID_PARAM(Kd, 1)); //Right head PID Kd param
  PIDparamKpB = thermalManager.temp_bed.pid.Kp; //Hot-bed PID Kp param
  PIDparamKiB = unscalePID_i(thermalManager.temp_bed.pid.Ki); //Hot-bed PID Ki param
  PIDparamKdB = unscalePID_d(thermalManager.temp_bed.pid.Kd); //Hot-bed PID Kd param
  FileScrollSpeed = 1;
  RTS_cyclesIcon = 0;

  //************Presets dialog****************/
  RTS_SndData(RTS_presets.pla_hotend_t, PRESET_PLA_HOTEND_VP);           //nozzle temperature, type int16_t
  RTS_SndData(RTS_presets.pla_bed_t, PRESET_PLA_BED_VP);                 //hot-bed temperature, type int16_t
  RTS_SndData(RTS_presets.petg_hotend_t, PRESET_PETG_HOTEND_VP);         //nozzle temperature, type int16_t
  RTS_SndData(RTS_presets.petg_bed_t, PRESET_PETG_BED_VP);               //hot-bed temperature, type int16_t
  RTS_SndData(RTS_presets.motor_hold_time, PRESET_MOTOR_HOLD_TIME_VP);   //DEFAULT_STEPPER_DEACTIVE_TIME in sec
  if (RTS_presets.auto_power_off_enable)                                 //auto power-off enabled, type boolean
  {
    RTS_SndData(0, PRESET_AUTO_POWER_OFF_VP);
  }
  else
  {
    RTS_SndData(1, PRESET_AUTO_POWER_OFF_VP);
  }

  // clean screen.
  for (int j = 0; j < FileNameLen; j ++)
  {
    RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
  }
  RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
  // clean screen.
  delay(2);
  /***************turn off motor*****************/
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  RTS_SndData(0, HEAD0_SET_ICON_VP);
  RTS_SndData(0, HEAD0_SET_TEMP_VP);
  RTS_SndData(0, HEAD1_SET_ICON_VP);
  RTS_SndData(0, HEAD1_SET_TEMP_VP);
  RTS_SndData(0, BED_SET_TEMP_VP);
  if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);} //black
  else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);} //red data background
  RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
  if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
  else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
  RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
  RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

  /***************transmit Fan speed to screen*****************/
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  thermalManager.set_fan_speed(1, 0);
  RTS_SndData(0, HEAD0_FAN_SPEED_VP);
  RTS_SndData(0, HEAD1_FAN_SPEED_VP);
  delay(5);

  /*********cleanup miscellanous variables***************/
  RTS_waitway = 0;
  heatway = 0;
  sdcard_pause_check = true;
  next_rts_update_ms = 0;
  Percentrecord = 0;
  pause_action_flag = false;
  CardUpdate = false;
  hasSelected = false;
  previousSelectionIndex = 0;
  Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
  Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
  AutoHomeIconNum = 0 ;
  memset(commandbuf, 0, sizeof(commandbuf));
  remain_time = 0;
  card_insert_st = IS_SD_INSERTED();
  last_card_insert_st = card_insert_st;
  sd_printing = false;

  /*********transmit SD card filename to screen***************/
  RTS_SDCardInit();

  /***************transmit Printer information to screen*****************/
  char sizebuf[20] = {0};
  sprintf(sizebuf, "%d X %d X %d", X_MAX_POS - 2, Y_MAX_POS - 2, Z_MAX_POS);
  RTS_SndData(MACHINE_NAME, PRINTER_MACHINE_TEXT_VP);
  RTS_SndData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
  RTS_SndData(DISPLAY_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
  RTS_SndData(sizebuf, PRINTER_PRINTSIZE_TEXT_VP);
  RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
  memset(sizebuf, 0, 20);
  sprintf(sizebuf, "  .    ");
  RTS_SndData(sizebuf, SCROLL_DIS_VP);
  /**************************some info init*******************************/
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  if(CardReader::flag.mounted)
  {
    change_page_number = 1;
  }
  else
  {
    change_page_number = 0;
  }
}

int RTSSHOW::RTS_RecData()
{
  int frame_index = 0;
  int timeout = 0;
  int framelen = 0;
  bool frame_flag = false;
  if (LCD_SERIAL.available() <= 0)
  {
    return -1;
  }
  do
  {
    if (LCD_SERIAL.available() > 0)
    {
      databuf[frame_index] = LCD_SERIAL.read();
      timeout = 0;
      /* 0x5A */
      if ((frame_index == 0) && (databuf[frame_index] == FHONE))
      {
        frame_index++;
        continue;
      }
      /* 0xA5 */
      else if (frame_index == 1)
      {
        if (databuf[frame_index] == FHTWO)
        {
          frame_index++;
        }
        else
        {
          frame_index = 0;
        }
        continue;
      }
      /* length */
      else if (frame_index == 2)
      {
        framelen = databuf[frame_index];
        frame_index++;
        continue;
      }
      else if (frame_index != 0)
      {
        frame_index++;
        /* After one frame of data is extracted, the remaining serial port data will be processed the next time it enters this function */
        if(frame_index == (framelen + 3))
        {
          frame_flag = true;
          break;
        }
      }
    }
    else
    {
      timeout++;
      delay(1);
    }
  }
  while (timeout < 50); /* Timeout-function */
  //	MYSERIAL0.write(0xBB);
  if (frame_flag == true)
  {
    recdat.head[0] = databuf[0];
    recdat.head[1] = databuf[1];
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    for(int idx = 0; idx < frame_index; idx++)
    {
    }
  }
  else
  {
    return -1;
  }
  // response for writing byte
  if ((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))
  {
    memset(databuf, 0, sizeof(databuf));
    return -1;
  }
  else if (recdat.command == 0x83)
  {
    // response for reading the data from the variable
    recdat.addr = databuf[4];
    recdat.addr = (recdat.addr << 8) | databuf[5];
    recdat.bytelen = databuf[6];
    for (unsigned int i = 0; i < recdat.bytelen; i += 2)
    {
      recdat.data[i / 2] = databuf[7 + i];
      recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
    }
  }
  else if (recdat.command == 0x81)
  {
    // response for reading the page from the register
    recdat.addr = databuf[4];
    recdat.bytelen = databuf[5];
    for (unsigned int i = 0; i < recdat.bytelen; i ++)
    {
      recdat.data[i] = databuf[6 + i];
      // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
    }
  }
  else
  {
    memset(databuf, 0, sizeof(databuf));
    // receive the wrong data
    return -1;
  }
  memset(databuf, 0, sizeof(databuf));
  return 2;
}

void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;
    // to write data to the register
    if(snddat.command == 0x80)
    {
      databuf[4] = snddat.addr;
      for(int i = 0;i <(snddat.len - 2);i ++)
      {
        databuf[5 + i] = snddat.data[i];
      }
    }
    else if((snddat.len == 3) && (snddat.command == 0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command == 0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if((snddat.len == 4) && (snddat.command == 0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
     for(int i = 0;i < (snddat.len + 3); i ++)
     {
       LCD_SERIAL.write(databuf[i]);
     }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if(len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i = 0;i < len;i ++)
    {
      databuf[6 + i] = str[i];
    }

    for(int i = 0;i < (len + 6);i ++)
    {
      LCD_SERIAL.write(databuf[i]);
      //delay(1); //delay Microseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
  }
}

void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) { RTS_SndData((char *)str, addr, cmd); }

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) { RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SDcardStop()
{
  RTS_waitway = 7;
  #if ENABLED(REALTIME_REPORTING_COMMANDS)
    planner.quick_pause();
    print_job_timer.stop();
    queue.clear();
    planner.clear_block_buffer();
    planner.quick_resume();
  #else
    planner.quick_stop();
    print_job_timer.stop();
    queue.clear();
    planner.clear_block_buffer();
  #endif
  /*
  //triple beep stop notification
  RTS_SndData(StartSoundSet, SoundAddr);
  wait_idle(250);
  RTS_SndData(StartSoundSet, SoundAddr);
  wait_idle(250);
  RTS_SndData(StartSoundSet, SoundAddr);
  */
  PoweroffContinue = false;
  RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
  //RTS_waitway = 7;
  Update_Time_Value = 0;
  PrintFlag = 0;
  //queue.enqueue_one_P(PSTR("M77"));//will be called when executing M77
  change_page_number = 1;
  #if ENABLED(DUAL_X_CARRIAGE)
    extruder_duplication_enabled = false;
    dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
    active_extruder = 0;
  #endif
  card.flag.abort_sd_printing = true;
  print_job_timer.reset();
  #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
    if(CardReader::flag.mounted)
    {
      card.removeJobRecoveryFile();
    }
  #endif
  #ifdef EVENT_GCODE_SD_ABORT
    //QUEUETEST
    //queue.enqueue_now_P(PSTR(EVENT_GCODE_SD_ABORT));
     queue.inject_P(PSTR(EVENT_GCODE_SD_ABORT));
  #endif
  //cooldown
  thermalManager.setTargetHotend(0, 0);
  RTS_SndData(0, HEAD0_SET_ICON_VP);
  RTS_SndData(0, HEAD0_SET_TEMP_VP);
  thermalManager.setTargetHotend(0, 1);
  RTS_SndData(0, HEAD1_SET_ICON_VP);
  RTS_SndData(0, HEAD1_SET_TEMP_VP);
  if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);} //black
  else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);} //red data background
  if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
  else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
  thermalManager.setTargetBed(0);
  RTS_SndData(0, BED_SET_TEMP_VP);
  thermalManager.zero_fan_speeds();
  wait_for_heatup = wait_for_user = false;
  sd_printing_autopause = false;
  wait_idle(2);
  // shut down the stepper motor.
  queue.enqueue_now_P(PSTR("M84"));
  RTS_SndData(0, MOTOR_FREE_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_VP);
  RTS_SndData(0, PRINT_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_TIME_MIN_VP);
  RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
  wait_idle(2);
  startprogress = 0;
  power_off_type_yes = 0;
  recovery.info.recovery_flag = false;
  //RTS_SndData(ExchangePageBase, ExchangepageAddr);
  RTS_Init(); //ensure defined printer state is set
}

void RTSSHOW::RTS_SDcardFinish()
{
  //cleanup
  RTS_waitway = 7;
  wait_for_heatup = wait_for_user = false;
  PoweroffContinue = false;
  recovery.info.recovery_flag = false;
  if(CardReader::flag.mounted)
  {
    card.removeJobRecoveryFile();
  }
  RTS_SndData(100, PRINT_PROCESS_VP);
  delay(1);
  RTS_SndData(100, PRINT_PROCESS_ICON_VP);
  delay(1);
  rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
  delay(1);
  rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
  delay(1);
  RTS_currentScreen = 1;
  RTS_waitway = 0;
  change_page_number = 1;
  rtscheck.RTS_SndData(ExchangePageBase + 9, ExchangepageAddr);
  //wait_idle(3000);
  active_extruder = active_extruder_font;
  print_job_timer.reset();
  //cooldown
  thermalManager.setTargetHotend(0, 0);
  RTS_SndData(0, HEAD0_SET_ICON_VP);
  RTS_SndData(0, HEAD0_SET_TEMP_VP);
  thermalManager.setTargetHotend(0, 1);
  RTS_SndData(0, HEAD1_SET_ICON_VP);
  RTS_SndData(0, HEAD1_SET_TEMP_VP);
  if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);} //black
  else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);} //red data background
  if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
  else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
  thermalManager.setTargetBed(0);
  RTS_SndData(0, BED_SET_TEMP_VP);
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  thermalManager.set_fan_speed(1, 0);
  RTS_SndData(0, HEAD0_FAN_SPEED_VP);
  RTS_SndData(0, HEAD1_FAN_SPEED_VP);
}

void RTSSHOW::RTS_Restart()
{
  // Finish
  PrintFlag = 0;
  planner.synchronize();
  queue.exhaust();
  print_job_timer.stop();
  planner.clear_block_buffer();
  delay(5);
  //cleanup miscellanous variables
  heatway = 0;
  sdcard_pause_check = true;
  next_rts_update_ms = 0;
  Percentrecord = 0;
  pause_action_flag = false;
  CardUpdate = false;
  hasSelected = false;
  previousSelectionIndex = 0;
  Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
  Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
  AutoHomeIconNum = 0 ;
  memset(commandbuf, 0, sizeof(commandbuf));
  remain_time = 0;
  card_insert_st = IS_SD_INSERTED();
  last_card_insert_st = card_insert_st;
  sd_printing = false;
  change_page_number = 1;
  PoweroffContinue = false;
  card.flag.abort_sd_printing = true;
  sd_printing_autopause = false;
  print_job_timer.stop();
  planner.clear_block_buffer();
  print_job_timer.reset();
  queue.clear();
  #if ENABLED(DUAL_X_CARRIAGE)
    extruder_duplication_enabled = false;
    dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
    active_extruder = 0;
  #endif
  //cooldown
  thermalManager.setTargetHotend(0, 0);
  RTS_SndData(0, HEAD0_SET_ICON_VP);
  RTS_SndData(0, HEAD0_SET_TEMP_VP);
  thermalManager.setTargetHotend(0, 1);
  RTS_SndData(0, HEAD1_SET_ICON_VP);
  RTS_SndData(0, HEAD1_SET_TEMP_VP);
  if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);} //black
  else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);} //red data background
  if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
  else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
  thermalManager.setTargetBed(0);
  RTS_SndData(0, BED_SET_TEMP_VP);
  thermalManager.zero_fan_speeds();
  wait_for_heatup = wait_for_user = false;
  Update_Time_Value = 0;
  wait_idle(2);
  //queue.enqueue_one_P(PSTR("M77"));
  //shut off stepper motors
  //queue.enqueue_now_P(PSTR("M84"));
  RTS_SndData(0, MOTOR_FREE_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_VP);
  RTS_SndData(0, PRINT_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_TIME_MIN_VP);
  RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
  RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
  wait_idle(2);
  startprogress = 100;
  power_off_type_yes = 0;
  #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
    if(CardReader::flag.mounted)
    {
      card.removeJobRecoveryFile();
    }
  #endif
  recovery.info.recovery_flag = false;
  //RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  RTS_Init(); //ensure defined printer state is set
}


void RTSSHOW::RTS_ProcessPause()
{
  RTS_waitway = 1;
  RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
  //pause command
  card.pauseSDPrint();
  pause_action_flag = true;
  Update_Time_Value = 0;
  sdcard_pause_check = false;
  PrintFlag = 1;
  change_page_number = 12;
  //QUEUETEST
  //queue.enqueue_one_P(PSTR("M76"));
  queue.inject_P(PSTR("M76"));
}

void RTSSHOW::RTS_ProcessResume()
{
  RTS_waitway = 0;
  //queue.enqueue_one_P(PSTR("M117 Resuming..."));
  RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
  #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
    pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
  #endif
  wait_for_user = false;
  //RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
  card.startOrResumeFilePrinting();
  //print_job_timer.start();
  Update_Time_Value = 0;
  sdcard_pause_check = true;
  pause_action_flag = false;
  RTS_currentScreen = 11;
  RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
  PrintFlag = 2;
  //QUEUETEST
  //queue.enqueue_one_P(PSTR("M75"));
  queue.inject_P(PSTR("M75"));
  TERN_(HOST_PAUSE_M76, host_action_resume());
}
void RTSSHOW::RTS_ProcessM25()
{
  //pause command
  RTS_waitway = 0;
  RTS_SndData(PSTR("Pausing..."), PRINT_FILE_TEXT_VP);
  card.pauseSDPrint();
  pause_action_flag = true;
  Update_Time_Value = 0;
  sdcard_pause_check = false;
  PrintFlag = 1;
  RTS_currentScreen = 12;
  RTS_SndData(ExchangePageBase + 60, ExchangepageAddr);
}

void RTSSHOW::RTS_ProcessM24()
{
  //resume command
  RTS_waitway = 0;
  RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
  #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
    pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
  #endif
  wait_for_user = false;
  card.startOrResumeFilePrinting();
  //print_job_timer.start();
  Update_Time_Value = 0;
  sdcard_pause_check = true;
  pause_action_flag = false;
  //return to previous RTS screen
  RTS_SndData(ExchangePageBase + RTS_currentScreen, ExchangepageAddr);
  PrintFlag = 2;
}

void RTSSHOW::RTS_HandleData()
{
  int Checkkey = -1;
  // for waiting
/*RTS_waitway states:
  0: no waiting
  1: wait while pause                          => back to #11
  2: unused
  3: wait while auto lev./aligning             => back to #22
  4: wait while homing or moving axes          => back to #29,#30,#31 or #58
  5: unused
  6: wait while leveling menu                  => back to #22
  7: wait while stopping or finishing SD card  => back to #1
*/
  if(RTS_waitway > 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    //catch unwanted RTS_waitway behaviour
    if (RTS_waitway > 7)
    {
      RTS_waitway = 0;
    }
    SERIAL_ECHOLNPGM("Ignoring RTS input. RTS_waitway: ", RTS_waitway);
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i ++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      if(Addrbuf[i] >= ChangePageKey)
      {
        Checkkey = i;
      }
      break;
    }
  }

  if(Checkkey < 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  switch(Checkkey)
  {
    case MainPageKey:
      if(recdat.data[0] == 1)
      {
        CardUpdate = true;
        if(CardReader::flag.mounted)
        {
          for (int j = 0; j < FileNameLen; j ++)
          {
            RTS_SndData(0, SELECT_DIR_TEXT_VP + j);
          }
          for (int j = 0; j < 7; j++) //activate scroll text
          {
            RTS_SndData((unsigned int)FileScrollSpeed, PRINT_FILE_TEXT_SP + (j * 32));
          }
          RTS_currentScreen = 1;
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        }
        else
        {
          RTS_currentScreen = 1;
          RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 2) //from screen #9 print finished
      {
        //clean screen
        Update_Time_Value = 0;
        wait_idle(2);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_VP);
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
        for (int j = 0; j < FileNameLen; j ++)
        {
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        }
        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
        delay(2);
        change_page_number = 1;
        RTS_currentScreen = 1;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(thermalManager.fan_speed[0], HEAD0_FAN_SPEED_VP);
        RTS_SndData(thermalManager.fan_speed[1], HEAD1_FAN_SPEED_VP);
        RTS_currentScreen = 15;
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if(recdat.data[0] == 4)
      {
        if (stepper.is_awake() == true)
        {
          RTS_SndData(0, MOTOR_FREE_ICON_VP); //motors enabled
        }
        else
        {
          RTS_SndData(1, MOTOR_FREE_ICON_VP); //motors disabled
        }
        RTS_currentScreen = 21;
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
      }
      else if(recdat.data[0] == 5)
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          save_dual_x_carriage_mode = dualXPrintingModeStatus;
          SERIAL_ECHOLNPGM("Debug Mode: MainPageKey=5");
          SetExtruderMode(save_dual_x_carriage_mode, true);
          RTS_currentScreen = 34;
          RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
        #endif
      }
      else if(recdat.data[0] == 6) //stop key from #60
      {
        //go to main screen
        RTS_currentScreen = 1;
        SERIAL_ECHOLNPGM("Stop - go to main screen #1" );
        RTS_SDcardStop();
      }
      else if(recdat.data[0] == 7) //resume key from #60
      {
        //return to previous RTS screen
        SERIAL_ECHOLNPGM("Resume - go to screen #", RTS_currentScreen );
        RTS_SndData(ExchangePageBase + RTS_currentScreen, ExchangepageAddr);
      }
      break;

    case AdjustmentKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(thermalManager.fan_speed[0], HEAD0_FAN_SPEED_VP);
        RTS_SndData(thermalManager.fan_speed[1], HEAD1_FAN_SPEED_VP);
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        RTS_currentScreen = 14;
        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
      }
      else if(recdat.data[0] == 2)
      {
        if(PrintFlag == 1)
        {
          RTS_currentScreen = 12;
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
        }
        else
        {
          RTS_currentScreen = 11;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 5) //Toggle Auto Power-Off
      {
        if(autoPowerOffEnabled == true)
        {
          RTS_SndData(1, AUTO_POWER_OFF_ICON_VP);
          autoPowerOffEnabled = false;
        }
        else
        {
          RTS_SndData(0, AUTO_POWER_OFF_ICON_VP);
          autoPowerOffEnabled = true;
        }
        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
      }
      break;

    case PrintSpeedKey:
      feedrate_percentage = recdat.data[0];
      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      break;

    case StopPrintKey:
      if((recdat.data[0] == 1) || (recdat.data[0] == 0xF1)) //stop from change filament screen #8 OR "yes, stop" from screen #13
      {
        RTS_SDcardStop();
      }
      else if(recdat.data[0] == 0xF0) //"no stop"
      {
        if(card.isPrinting)
        {
          if (print_job_timer.isRunning)
          {
            RTS_currentScreen = 11;
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          } else {
            RTS_currentScreen = 12;
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
        }
        else if(sdcard_pause_check == false)
        {
          RTS_currentScreen = 12;
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
        }
      }
      break;

    case PausePrintKey:
      if(recdat.data[0] == 0xF0) //"no pause"
      {
        break;
      }
      else if(recdat.data[0] == 0xF1) //"yes, pause"
      {
        RTS_ProcessPause();
      }
      break;

    case ResumePrintKey:
      if(recdat.data[0] == 1)
      {
        //resume from paused
        RTS_ProcessResume();
      }
      else if(recdat.data[0] == 2)
      {
        queue.enqueue_one_P(PSTR("M117 Resuming..."));
        #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
        #endif
        wait_for_user = false;
        //change filament and resume
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        //card.startOrResumeFilePrinting();
        //print_job_timer.start();
        Update_Time_Value = 0;
        pause_action_flag = false;
        sdcard_pause_check = true;
        PrintFlag = 2;
        if (card.isPrinting)
        {
          rtscheck.RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
        }
        //sd_printing_autopause = true;
        RTS_currentScreen = 11;
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        //queue.enqueue_one_P(PSTR("M75"));
        TERN_(HOST_PAUSE_M76, host_action_resume());
      }
      else if(recdat.data[0] == 3) //from screen #39 heat up
      {
        queue.enqueue_one_P(PSTR("M117 Resuming..."));
        #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
        #endif
        wait_for_user = false;
        //heat and change filament, and resume
        if(PoweroffContinue == true)
        {
          RTS_currentScreen = 8;
          RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
          PoweroffContinue = false; // added by John Carlson to reset the flag
        }
        else if(PoweroffContinue == false)
        {
          char cmd[MAX_CMD_SIZE+16];
          char *c;
          sprintf_P(cmd, PSTR("M23 %s"), fileInfo.currentFilePath);
          for (c = &cmd[4]; *c; c++)
            *c = tolower(*c);

          queue.enqueue_one_now(cmd);
          delay(20);
          queue.enqueue_now_P(PSTR("M24"));
          // clean screen.
          for (int j = 0; j < FileNameLen; j ++)
          {
            RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }
          RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
          delay(2);
          #if ENABLED(BABYSTEPPING)
            RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
          #endif
          feedrate_percentage = 100;
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          zprobe_zoffset = last_zoffset;
          RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          PoweroffContinue = true;
          RTS_currentScreen = 11;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          sdcard_pause_check = true;
        }
      }
      else if(recdat.data[0] == 4) //from screen #46 SDcard removed
      {
        queue.enqueue_one_P(PSTR("M117 Resuming..."));
        #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
        #endif
        wait_for_user = false;
        //replaced SD card and continue
        if(!CardReader::flag.mounted)
        {
          CardUpdate = true;
          RTS_SDCardUpate();
          RTS_currentScreen = 46;
          RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);

          card.startOrResumeFilePrinting();
          //print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          pause_action_flag = false;
          PrintFlag = 2;

          if (card.isPrinting) {
            rtscheck.RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
          }

          sd_printing_autopause = true;
          RTS_currentScreen = 11;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          queue.enqueue_one_P(PSTR("M75"));
          TERN_(HOST_PAUSE_M76, host_action_resume());
        }
      }
      break;

    case HotBedTempEnterKey:
      if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7))
      {
        //double beep error notification
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);
        break;
      }
      thermalManager.temp_bed.target = recdat.data[0];
      thermalManager.setTargetBed(thermalManager.temp_bed.target);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      SERIAL_ECHOLNPGM("HotBedEnterKey: ", recdat.data[0]);
      break;

    case TempScreenKey:
      if (recdat.data[0] == 1)
      {
        RTS_SndData(thermalManager.fan_speed[0], HEAD0_FAN_SPEED_VP);
        RTS_currentScreen = 16;
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2)
      {
        RTS_SndData(thermalManager.fan_speed[1], HEAD1_FAN_SPEED_VP);
        RTS_currentScreen = 17;
        RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
      }
      else if (recdat.data[0] == 3)
      {
        RTS_currentScreen = 18;
        RTS_SndData(ExchangePageBase + 18, ExchangepageAddr);
      }
      else if (recdat.data[0] == 4)
      {
        RTS_currentScreen = 15;
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if (recdat.data[0] == 0xF1)
      {
        #if FAN_COUNT > 0
          for (uint8_t i = 0; i < FAN_COUNT; i++)
          {
            thermalManager.fan_speed[i] = 255;
          }
        #endif
        RTS_SndData(thermalManager.fan_speed[0], HEAD0_FAN_SPEED_VP);
        RTS_SndData(thermalManager.fan_speed[1], HEAD1_FAN_SPEED_VP);
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD0_SET_ICON_VP);
        RTS_SndData(0, HEAD0_SET_TEMP_VP);
        delay(1);
        thermalManager.setTargetHotend(0, 1);
        RTS_SndData(0, HEAD1_SET_ICON_VP);
        RTS_SndData(0, HEAD1_SET_TEMP_VP);
        delay(1);
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
        delay(1);

        RTS_currentScreen = 15;
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if (recdat.data[0] == 0xF0)
      {
        RTS_currentScreen = 15;
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      break;

    case CoolScreenKey:
      if (recdat.data[0] == 1)
      {
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD0_SET_ICON_VP);
        RTS_SndData(0, HEAD0_SET_TEMP_VP);
        thermalManager.fan_speed[0] = 255;
        RTS_SndData(255, HEAD0_FAN_SPEED_VP);
      }
      else if (recdat.data[0] == 2)
      {
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 3)
      {
        thermalManager.setTargetHotend(0, 1);
        RTS_SndData(0, HEAD1_SET_ICON_VP);
        RTS_SndData(0, HEAD1_SET_TEMP_VP);
        thermalManager.fan_speed[1] = 255;
        RTS_SndData(255, HEAD0_FAN_SPEED_VP);
      }
      else if (recdat.data[0] == 4)
      {
        RTS_currentScreen = 15;
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if (recdat.data[0] == 5)
      {
        thermalManager.temp_hotend[0].target = RTS_presets.pla_hotend_t;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        thermalManager.temp_bed.target = RTS_presets.pla_bed_t;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 6)
      {
        thermalManager.temp_hotend[0].target = RTS_presets.petg_hotend_t;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        thermalManager.temp_bed.target = RTS_presets.petg_bed_t;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 7)
      {
        thermalManager.temp_hotend[1].target = RTS_presets.pla_hotend_t;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        thermalManager.temp_bed.target = RTS_presets.pla_bed_t;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 8)
      {
        thermalManager.temp_hotend[1].target = RTS_presets.petg_hotend_t;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        thermalManager.temp_bed.target = RTS_presets.petg_bed_t;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      break;

    case Heater0TempEnterKey:
      if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7))
      {
        //double beep error notification
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);
        break;
      }
      thermalManager.temp_hotend[0].target = recdat.data[0];
      thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
      if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
      else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
      SERIAL_ECHOLNPGM("Heater0TempEnterKey: ", recdat.data[0]);
      break;

    case Heater1TempEnterKey:
      if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7))
      {
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);
        break;
      }
      thermalManager.temp_hotend[1].target = recdat.data[0];
      thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
      if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
      else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
      RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
      SERIAL_ECHOLNPGM("Heater1TempEnterKey: ", recdat.data[0]);
      break;

    case SettingScreenKey: //'Settings' screen #21
      SERIAL_ECHOLNPGM("Settings Button ID: ", recdat.data[0]);
      if(recdat.data[0] == 1) //'Leveling' button
      {
        // Motor Icon
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        // only for prohibiting to receive massage
        RTS_waitway = 6;
        AutoHomeIconNum = 0;
        active_extruder = 0;
        active_extruder_flag = false;
        active_extruder_font = active_extruder;
        Update_Time_Value = 0;
        queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("G0 F600 Z0.0"));
        RTS_currentScreen = 32;
        RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);

        if (active_extruder == 0)
        {
          RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
        }
        else
        {
          RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
        }
      }
      else if(recdat.data[0] == 2) //'Refuel' button
      {
        Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
        Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
        RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
        RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
        if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);

        if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);

        delay(2);
        RTS_currentScreen = 23;
        RTS_SndData(ExchangePageBase + 23, ExchangepageAddr); //call 'Refuel' screen
      }
      else if (recdat.data[0] == 3) //'Move' button
      {
        if(active_extruder == 0)
        {
          RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
          active_extruder_flag = false;
        }
        else if(active_extruder == 1)
        {
          RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
          active_extruder_flag = true;
        }
        AxisUnitMode = 3;
        if (all_axes_trusted())
        {
          //Hide hazard warning
          RTS_SndData(0, COLLISION_HAZARD_ICON_VP);
        }
        else
        {
          //Show hazard warning
          RTS_SndData(1, COLLISION_HAZARD_ICON_VP);
          //home x axis in order to avoid damage
          queue.enqueue_now_P(PSTR("G28 X"));
        }
        if(active_extruder == 0)
        {
          if(TEST(axis_trusted, X_AXIS))
          {
            current_position_x0_axis = current_position[X_AXIS];
          }
          else
          {
            current_position[X_AXIS] = current_position_x0_axis;
          }
          RTS_SndData(10 * current_position_x0_axis, AXIS_X_COORD_VP);
          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x0_axis);
          queue.enqueue_one_now(commandbuf);
        }
        else if(active_extruder == 1)
        {
          if(TEST(axis_trusted, X_AXIS))
          {
            current_position_x1_axis = current_position[X_AXIS];
          }
          else
          {
            current_position[X_AXIS] = current_position_x1_axis;
          }
          RTS_SndData(10 * current_position_x1_axis, AXIS_X_COORD_VP);
          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x1_axis);
          queue.enqueue_one_now(commandbuf);
        }
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        RTS_currentScreen = 29;
        RTS_SndData(ExchangePageBase + 29, ExchangepageAddr); //call 'Move' screen
      }
      else if (recdat.data[0] == 4) //'Set hotend offset' button
      {
        RTS_currentScreen = 35;
        RTS_SndData(ExchangePageBase + 35, ExchangepageAddr); //call 'Set hotend offset' screen
      }
      else if (recdat.data[0] == 5) //'Printer info' button
      {
        RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
        RTS_currentScreen = 33;
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr); //call 'Printer info' screen
      }
      else if (recdat.data[0] == 6) //'Disable motor' button
      {
        queue.enqueue_now_P(PSTR("M84"));
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
      }
      else if (recdat.data[0] == 7) //Return button
      {
        RTS_currentScreen = 1;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr); //call main screen
      }
      //'Power Off' button has ID "SuicideKey" & calls popup menu screen #88 'Power off?' that returns key codes 0xF1 (Yes) / 0xF0 (No)
      else if (recdat.data[0] == 8) //Extra settings button
      {
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //call 'Extra settings' screen
      }
      else if (recdat.data[0] == 11) //From extra settings screen: Nozzle 1 E-steps button
      {
        //temperatures
        if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
        if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        //feedrate
        RTS_SndData(EstepFeedRate*10, E_STEPS_FEED_RATE_VP);
        //distance
        RTS_SndData(EstepFeedDistance*10, E_STEPS_FEED_DIST_VP);
        //e-steps
        ConvertLong = ESteps0 * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_E_STEPS_VP + 1);
        RTS_currentScreen = 91;
        RTS_SndData(ExchangePageBase + 91, ExchangepageAddr); //call 'Nozzle 1 E-steps' screen
      }
      else if (recdat.data[0] == 12) //From extra settings screen: Nozzle 2 E-steps button
      {
        //temperatures
        if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
        if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        //feedrate
        RTS_SndData(EstepFeedRate*10, E_STEPS_FEED_RATE_VP);
        //distance
        RTS_SndData(EstepFeedDistance*10, E_STEPS_FEED_DIST_VP);
        //e-steps
        ConvertLong = ESteps1 * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_E_STEPS_VP + 1);
        RTS_currentScreen = 92;
        RTS_SndData(ExchangePageBase + 92, ExchangepageAddr); //call 'Nozzle 2 E-steps' screen
      }
      else if (recdat.data[0] == 13) //From extra settings screen: Nozzle 1 PID button
      {
        //temperatures
        if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
        //cycles
        RTS_SndData(0, TUNE_CURRENT_CYCLE_VP);
        RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP); //default #of tune cycles
        RTS_SndData(0, TUNE_PROGRESS_ICON_VP);
        //Kp
        ConvertLong = thermalManager.temp_hotend[0].pid.Kp * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KP_VP + 1);
        //Ki
        ConvertLong = unscalePID_i(thermalManager.temp_hotend[0].pid.Ki) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KI_VP + 1);
        //Kd
        ConvertLong = unscalePID_d(thermalManager.temp_hotend[0].pid.Kd) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KD_VP + 1);
        RTS_currentScreen = 93;
        RTS_SndData(ExchangePageBase + 93, ExchangepageAddr); //call 'Nozzle 1 PID' screen
      }
      else if (recdat.data[0] == 14) //From extra settings screen: Nozzle 2 PID button
      {
        //temperatures
        if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
        RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
        //cycles
        RTS_SndData(0, TUNE_CURRENT_CYCLE_VP);
        RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP); //default #of tune cycles
        RTS_SndData(0, TUNE_PROGRESS_ICON_VP);
        //Kp
        ConvertLong = thermalManager.temp_hotend[1].pid.Kp * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KP_VP + 1);
        //Ki
        ConvertLong = unscalePID_i(thermalManager.temp_hotend[1].pid.Ki) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KI_VP + 1);
        //Kd
        ConvertLong = unscalePID_d(thermalManager.temp_hotend[1].pid.Kd) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KD_VP + 1);
        RTS_currentScreen = 94;
        RTS_SndData(ExchangePageBase + 94, ExchangepageAddr); //call 'Nozzle 2 PID' screen
      }
      else if (recdat.data[0] == 15) //From extra settings screen: Hot-bed PID button
      {
        //temperatures
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
        //cycles
        RTS_SndData(0, TUNE_CURRENT_CYCLE_VP);
        RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP); //default #of tune cycles
        RTS_SndData(0, TUNE_PROGRESS_ICON_VP);
        //KpBed
        ConvertLong = thermalManager.temp_bed.pid.Kp * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KP_VP + 1);
        //KiBed
        ConvertLong = unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KI_VP + 1);
        //KdBed
        ConvertLong = unscalePID_d(thermalManager.temp_bed.pid.Kd) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KD_VP + 1);
        RTS_currentScreen = 95;
        RTS_SndData(ExchangePageBase + 95, ExchangepageAddr); //call 'Hot-bed PID' screen
      }
      else if (recdat.data[0] == 16) //From extra settings screen: Manual PID button
      {
        //Kp0
        ConvertLong = thermalManager.temp_hotend[0].pid.Kp * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KP_VP + 1);
        //Ki0
        ConvertLong = unscalePID_i(thermalManager.temp_hotend[0].pid.Ki) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KI_VP + 1);
        //Kd0
        ConvertLong = unscalePID_d(thermalManager.temp_hotend[0].pid.Kd) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KD_VP + 1);
        //Kp1
        ConvertLong = thermalManager.temp_hotend[1].pid.Kp * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KP_VP + 1);
        //Ki1
        ConvertLong = unscalePID_i(thermalManager.temp_hotend[1].pid.Ki) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KI_VP + 1);
        //Kd1
        ConvertLong = unscalePID_d(thermalManager.temp_hotend[1].pid.Kd) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KD_VP + 1);
        //KpBed
        ConvertLong = thermalManager.temp_bed.pid.Kp * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KP_VP + 1);
        //KiBed
        ConvertLong = unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KI_VP + 1);
        //KdBed
        ConvertLong = unscalePID_d(thermalManager.temp_bed.pid.Kd) * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KD_VP + 1);
        RTS_currentScreen = 96;
        RTS_SndData(ExchangePageBase + 96, ExchangepageAddr); //call 'Manual PID' screen
      }
      else if(recdat.data[0] == 17) //From extra settings screen: Presets button
      {
        //send current presets
        RTS_SndData(RTS_presets.pla_hotend_t, PRESET_PLA_HOTEND_VP);           //nozzle temperature, type int16_t
        RTS_SndData(RTS_presets.pla_bed_t, PRESET_PLA_BED_VP);                 //hot-bed temperature, type int16_t
        RTS_SndData(RTS_presets.petg_hotend_t, PRESET_PETG_HOTEND_VP);         //nozzle temperature, type int16_t
        RTS_SndData(RTS_presets.petg_bed_t, PRESET_PETG_BED_VP);               //hot-bed temperature, type int16_t
        RTS_SndData(RTS_presets.motor_hold_time, PRESET_MOTOR_HOLD_TIME_VP);   //DEFAULT_STEPPER_DEACTIVE_TIME in sec
        if (RTS_presets.auto_power_off_enable)                                 //auto power-off enabled, type boolean
        {
          RTS_SndData(0, PRESET_AUTO_POWER_OFF_VP);
        }
        else
        {
          RTS_SndData(1, PRESET_AUTO_POWER_OFF_VP);
        }
        RTS_currentScreen = 97;
        RTS_SndData(ExchangePageBase + 97, ExchangepageAddr); //call 'Presets' screen;
      }
      break;

    case SettingBackKey:
      SERIAL_ECHOLNPGM("Settings Button ID: ", recdat.data[0]);
      if (recdat.data[0] == 1)
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        SetExtruderMode(save_dual_x_carriage_mode, true);
        if (stepper.is_awake() == true)
        {
          RTS_SndData(0, MOTOR_FREE_ICON_VP); //motors enabled
        }
        else
        {
          RTS_SndData(1, MOTOR_FREE_ICON_VP); //motors disabled
        }
        RTS_currentScreen = 21;
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2) //from aux leveling screen #22
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(HAS_LEVELING)
            RTS_currentScreen = 22;
        RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
          #else
            if (stepper.is_awake() == true)
            {
              RTS_SndData(0, MOTOR_FREE_ICON_VP); //motors enabled
            }
            else
            {
              RTS_SndData(1, MOTOR_FREE_ICON_VP); //motors disabled
            }
            RTS_currentScreen = 21;
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          #endif
          queue.enqueue_now_P(PSTR("M420 S1"));
        }
      }
      else if (recdat.data[0] == 3)
      {
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 X%4.1f"), hotend_offset[1].x);
        queue.enqueue_now_P(commandbuf);
        delay(5);
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 Y%4.1f"), hotend_offset[1].y);
        queue.enqueue_now_P(commandbuf);
        delay(5);
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 Z%4.1f"), hotend_offset[1].z);
        queue.enqueue_now_P(commandbuf);
        //settings.save();
      }
      else if (recdat.data[0] == 5) //Return button fom mesh view #81
      {
        RTS_currentScreen = 22;
        RTS_SndData(ExchangePageBase + 22, ExchangepageAddr); //return to leveling mode screen
      }
      else if (recdat.data[0] == 6) //Return button fom extra settings screen #90
      {
        RTS_currentScreen = 21;
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr); //return to settings screen
      }
      else if (recdat.data[0] == 7) //Return button fom E-steps left extruder screen #91
      {
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      else if (recdat.data[0] == 8) //Return button fom E-steps right extruder screen #92
      {
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      else if (recdat.data[0] == 9) //Return button fom PID tune left extruder screen #93
      {
        if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7)) //check if tuning is in process
        {
          //only abort key will be processed during tuning
          //double beep error notification
          RTS_SndData(StartSoundSet, SoundAddr);
          wait_idle(150);
          RTS_SndData(StartSoundSet, SoundAddr);
          break;
        }
        //reset Autotuning indicator
        RTS_cyclesIcon = 0;
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      else if (recdat.data[0] == 10) //Return button fom PID tune right extruder screen #94
      {
        if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7)) //check if tuning is in process
        {
          //only abort key will be processed during tuning
          //double beep error notification
          RTS_SndData(StartSoundSet, SoundAddr);
          wait_idle(150);
          RTS_SndData(StartSoundSet, SoundAddr);
          break;
        }
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      else if (recdat.data[0] == 11) //Return button fom PID tune hot-bed screen #95
      {
        if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7)) //check if tuning is in process
        {
          //only abort key will be processed during tuning
          //double beep error notification
          RTS_SndData(StartSoundSet, SoundAddr);
          wait_idle(150);
          RTS_SndData(StartSoundSet, SoundAddr);
          break;
        }
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      else if (recdat.data[0] == 12) //Return button fom PID tune hot-bed screen #96
      {
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      else if (recdat.data[0] == 13) //Return button fom Presets screen #97
      {
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //return to extra settings screen
      }
      break;

    case BedLevelFunKey: //'Leveling mode' screen #22
      if (recdat.data[0] == 1) //Home button
      {
        RTS_waitway = 6;
        if((active_extruder == 1) || (!TEST(axis_trusted, X_AXIS)) || (!TEST(axis_trusted, Y_AXIS)))
        {
          AutoHomeIconNum = 0;
          active_extruder = 0;
          active_extruder_flag = false;
          active_extruder_font = active_extruder;
          queue.enqueue_now_P(PSTR("G28"));
          RTS_currentScreen = 32;
        RTS_SndData(ExchangePageBase + 32, ExchangepageAddr); //call 'Auto home, wait...' screen
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28 Z"));
        }
        queue.enqueue_now_P(PSTR("G0 F1500 Y150.0")); // added to move nozzle to probe position - John Carlson
        queue.enqueue_now_P(PSTR("G0 F600 Z0.0"));
      }
      else if (recdat.data[0] == 2) //'+Z' arrow button
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset + 0.01), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.01);
            zprobe_zoffset = zprobe_zoffset + 0.00001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 3) //'-Z' arrow button
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.01), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset - 0.01);
            zprobe_zoffset = zprobe_zoffset - 0.00001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 4) //'AUX leveling' button
      {
        RTS_currentScreen = 28;
        RTS_SndData(ExchangePageBase + 28, ExchangepageAddr); //call 'AUX leveling' screen
        if(active_extruder == 0)
        {
          RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
        }
        else if(active_extruder == 1)
        {
          RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
        }
        queue.enqueue_now_P(PSTR("M420 S0"));
      }
      else if (recdat.data[0] == 5) //'Auto leveling' button, screen #22
      {
        #if ENABLED(BLTOUCH)
          RTS_waitway = 3;
          RTS_SndData(0, AUTO_BED_LEVEL_ICON_VP); //prepare 'Auto leveling, wait...' screen
          RTS_currentScreen = 38;
          RTS_SndData(ExchangePageBase + 38, ExchangepageAddr); //call 'Auto leveling, wait...' screen
          if (!all_axes_trusted()) {
            queue.enqueue_now_P(PSTR("G28"));
          }
          queue.enqueue_now_P(PSTR("G29"));
        #endif
      }
      if (recdat.data[0] == 6) //'1' button, AUX leveling, screen #28
      {
        // Assitant Level ,  Centre 1
        if(!planner.has_blocks_queued())
        {
          RTS_waitway = 4;
          queue.enqueue_now_P(PSTR("G0 F600 Z3"));
          queue.enqueue_now_P(PSTR("G0 X150 Y150 F3000"));
          queue.enqueue_now_P(PSTR("G0 F600 Z0"));
          RTS_waitway = 0;
        }
      }
      else if (recdat.data[0] == 7) //'2' button, AUX leveling, screen #28
      {
        // Assitant Level , Front Left 2
        if(!planner.has_blocks_queued())
        {
          RTS_waitway = 4;
          queue.enqueue_now_P(PSTR("G0 F600 Z3"));
          queue.enqueue_now_P(PSTR("G0 X30 Y30 F3000"));
          queue.enqueue_now_P(PSTR("G0 F600 Z0"));
          RTS_waitway = 0;
        }
      }
      else if (recdat.data[0] == 8) //'3' button, AUX leveling, screen #28
      {
        // Assitant Level , Front Right 3
        if(!planner.has_blocks_queued())
        {
          RTS_waitway = 4;
          queue.enqueue_now_P(PSTR("G0 F600 Z3"));
          queue.enqueue_now_P(PSTR("G0 X275 Y30 F3000"));
          queue.enqueue_now_P(PSTR("G0 F600 Z0"));
          RTS_waitway = 0;
        }
      }
      else if (recdat.data[0] == 9) //'4' button, AUX leveling, screen #28
      {
        // Assitant Level , Back Right 4
        if(!planner.has_blocks_queued())
        {
          RTS_waitway = 4;
          queue.enqueue_now_P(PSTR("G0 F600 Z3"));
          queue.enqueue_now_P(PSTR("G0 X275 Y275 F3000"));
          queue.enqueue_now_P(PSTR("G0 F600 Z0"));
          RTS_waitway = 0;
        }
      }
      else if (recdat.data[0] == 10) //'5' button, AUX leveling, screen #28
      {
        // Assitant Level , Back Left 5
        if(!planner.has_blocks_queued())
        {
          RTS_waitway = 4;
          queue.enqueue_now_P(PSTR("G0 F600 Z3"));
          queue.enqueue_now_P(PSTR("G0 X30 Y275 F3000"));
          queue.enqueue_now_P(PSTR("G0 F600 Z0"));
          RTS_waitway = 0;
        }
      }
      else if (recdat.data[0] == 11) //'Auto Z-align' button from screen #22
      {
        RTS_waitway = 3;
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr); //call 'Processing, wait...' screen
        //queue.enqueue_now_P(PSTR("G28 X"));
        active_extruder_flag = false;
        active_extruder = 0;
        active_extruder_font = active_extruder;
        RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);        queue.enqueue_now_P(PSTR("G34"));
        Update_Time_Value = 0;
        RTS_waitway = 0;
      }
      else if (recdat.data[0] == 12) //'Tramming' key or 'Run Again' key
      {
        //RTS_waitway = 3;
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr); //call 'Processing, wait...' screen
        //queue.enqueue_now_P(PSTR("G28 X"));
        //queue.enqueue_now_P(PSTR("T0"));
        active_extruder_flag = false;
        active_extruder = 0;
        active_extruder_font = active_extruder;
        RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        queue.enqueue_now_P(PSTR("G35"));
        //RTS_waitway = 0;
      }
      else if (recdat.data[0] == 13) //'Mesh viewer' button
      {
        RTS_ViewMesh();
        RTS_waitway = 0;
      }
      else if (recdat.data[0] == 14) //'++Z' arrow button
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset + 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.1);
            zprobe_zoffset = zprobe_zoffset + 0.00001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 15) //'--Z' arrow button
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset - 0.1);
            zprobe_zoffset = zprobe_zoffset - 0.00001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      break;

    case AxisPageSelectKey:
      if(recdat.data[0] == 5)
      {
        AxisUnitMode = 4;
        axis_unit = 100.0;
        RTS_currentScreen = 58;
        RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
      }
      else if(recdat.data[0] == 1)
      {
        AxisUnitMode = 3;
        axis_unit = 10.0;
        RTS_currentScreen = 29;
        RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_currentScreen = 30;
        RTS_SndData(ExchangePageBase + 30, ExchangepageAddr);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 1;
        axis_unit = 0.1;
        RTS_currentScreen = 31;
        RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
      }
      else if(recdat.data[0] == 4) //'Home'-Button from screen #29, #30, #31 or #58
      {
        if(!planner.has_blocks_queued())
        {
          // Motor Icon
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
          RTS_waitway = 4;
          AutoHomeIconNum = 0;
          queue.enqueue_now_P(PSTR("G28"));
          Update_Time_Value = 0;
          RTS_currentScreen = 32;
        RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
        }
      }
      break;

    case XaxismoveKey:
    if(!planner.has_blocks_queued())
      {
        RTS_waitway = 4;
        if(active_extruder == 0)
        {
          active_extruder_flag = false;
        }
        else if(active_extruder == 1)
        {
          active_extruder_flag = true;
        }

        if(active_extruder == 1)
        {
          if(recdat.data[0] >= 32768)
          {
            current_position_x1_axis = ((float)recdat.data[0] - 65536) / 10;
          }
          else
          {
            current_position_x1_axis = ((float)recdat.data[0]) / 10;
          }

          if(current_position_x1_axis > X2_MAX_POS)
          {
            current_position_x1_axis = X2_MAX_POS;
          }
          else if((TEST(axis_trusted, X_AXIS)) && (current_position_x1_axis < (current_position_x0_axis - X_MIN_POS)))
          {
            current_position_x1_axis = current_position_x0_axis - X_MIN_POS;
          }
          else if(current_position_x1_axis < (X_MIN_POS - X_MIN_POS))
          {
            current_position_x1_axis = X_MIN_POS - X_MIN_POS;
          }
          current_position[X_AXIS] = current_position_x1_axis;
        }
        else if(active_extruder == 0)
        {
          if(recdat.data[0] >= 32768)
          {
            current_position_x0_axis = ((float)recdat.data[0] - 65536) / 10;
          }
          else
          {
            current_position_x0_axis = ((float)recdat.data[0]) / 10;
          }

          if(current_position_x0_axis < X_MIN_POS)
          {
            current_position_x0_axis = X_MIN_POS;
          }
          else if((TEST(axis_trusted, X_AXIS)) && (current_position_x0_axis > (current_position_x1_axis + X_MIN_POS)))
          {
            current_position_x0_axis = current_position_x1_axis + X_MIN_POS;
          }
          else if(current_position_x0_axis > (X2_MAX_POS + X_MIN_POS))
          {
            current_position_x0_axis = X2_MAX_POS + X_MIN_POS;
          }
          current_position[X_AXIS] = current_position_x0_axis;
        }
        RTS_line_to_current(X_AXIS, manual_feedrate_X);
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        RTS_waitway = 0;
      }
      break;

    case YaxismoveKey:
    if(!planner.has_blocks_queued())
      {
        float y_min, y_max;
        RTS_waitway = 4;
        y_min = Y_MIN_POS;
        y_max = Y_MAX_POS;
        current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;
        if (current_position[Y_AXIS] < y_min)
        {
          current_position[Y_AXIS] = y_min;
        }
        else if (current_position[Y_AXIS] > y_max)
        {
          current_position[Y_AXIS] = y_max;
        }
        RTS_line_to_current(Y_AXIS, manual_feedrate_Y);
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        RTS_waitway = 0;
      }
      break;

    case ZaxismoveKey:
      if(!planner.has_blocks_queued())
      {
        float z_min, z_max;
        RTS_waitway = 4;
        z_min = Z_MIN_POS;
        z_max = Z_MAX_POS;
        current_position[Z_AXIS] = ((float)recdat.data[0]) / 10;
        if (current_position[Z_AXIS] < z_min)
        {
          current_position[Z_AXIS] = z_min;
        }
        else if (current_position[Z_AXIS] > z_max)
        {
          current_position[Z_AXIS] = z_max;
        }
        RTS_line_to_current(Z_AXIS, manual_feedrate_Z);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        RTS_waitway = 0;
      }
      break;

    case SelectExtruderKey:
      if(recdat.data[0] == 1) //from screen #29,#30,#31 or #58
      {
        if(!planner.has_blocks_queued())
        {
          if(active_extruder == 0)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T1"));
            active_extruder = 1;
            active_extruder_flag = true;
            active_extruder_font = active_extruder;

            RTS_SndData(10 * X2_MAX_POS, AXIS_X_COORD_VP);
            RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
          }
          else if(active_extruder == 1)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T0"));
            active_extruder = 0;
            active_extruder_flag = false;
            active_extruder_font = active_extruder;

            RTS_SndData(10 * X_MIN_POS, AXIS_X_COORD_VP);
            RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
          }
        }
      }
      else if (recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())  //from Aux leveling, screen #28
        {
          RTS_waitway = 4;
          if(active_extruder == 0)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T1"));
            active_extruder_flag = true;
            active_extruder = 1;
            active_extruder_font = active_extruder;
            RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
            RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
          }
          else if(active_extruder == 1)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T0"));
            active_extruder_flag = false;
            active_extruder = 0;
            active_extruder_font = active_extruder;
            RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
            RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
          }
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
          RTS_waitway = 0;
        }
      }
      break;

    case Heater0LoadEnterKey:
      Filament0LOAD = ((float)recdat.data[0]) / 10;
      break;

    case FilamentLoadKey:
      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          if(READ(FIL_RUNOUT_PIN) == 0)
          {
            RTS_currentScreen = 20;
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          current_position[E_AXIS_N(0)] -= Filament0LOAD;
          active_extruder = 0;
          queue.enqueue_now_P(PSTR("T0"));

          if(thermalManager.temp_hotend[0].celsius < (FILAMENT_CHANGE_TEMPERATURE_0 - 5))
          {
            RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_0, CHANGE_FILAMENT0_TEMP_VP);
            RTS_currentScreen = 24;
        RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS_N(0), manual_feedrate_E);
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          if(READ(FIL_RUNOUT_PIN) == 0)
          {
            RTS_currentScreen = 20;
            RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          current_position[E_AXIS_N(0)] += Filament0LOAD;
          active_extruder = 0;
          queue.enqueue_now_P(PSTR("T0"));

          if(thermalManager.temp_hotend[0].celsius < (FILAMENT_CHANGE_TEMPERATURE_0 - 5))
          {
            RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_0, CHANGE_FILAMENT0_TEMP_VP);
            RTS_currentScreen = 24;
            RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS_N(0), manual_feedrate_E);
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(!planner.has_blocks_queued())
        {
          if(READ(FIL_RUNOUT2_PIN) == 0)
          {
            RTS_currentScreen = 20;
            RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          current_position[E_AXIS_N(1)] -= Filament1LOAD;
          active_extruder = 1;
          queue.enqueue_now_P(PSTR("T1"));

          if (thermalManager.temp_hotend[1].celsius < (FILAMENT_CHANGE_TEMPERATURE_1 - 5))
          {
            RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_1, CHANGE_FILAMENT1_TEMP_VP);

            RTS_currentScreen = 25;
        RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS_N(1), manual_feedrate_E);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued())
        {
          if(READ(FIL_RUNOUT2_PIN) == 0)
          {
            RTS_currentScreen = 20;
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          current_position[E_AXIS_N(1)] += Filament1LOAD;
          active_extruder = 1;
          queue.enqueue_now_P(PSTR("T1"));

          if(thermalManager.temp_hotend[1].celsius < (FILAMENT_CHANGE_TEMPERATURE_1 - 5))
          {
            RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_1, CHANGE_FILAMENT1_TEMP_VP);
            RTS_currentScreen = 25;
        RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS_N(1), manual_feedrate_E);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 5)
      {
        if(!planner.has_blocks_queued())
        {
          if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);}
          else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);}
          RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
          thermalManager.setTargetHotend(FILAMENT_CHANGE_TEMPERATURE_0, 0);
          if (FILAMENT_CHANGE_TEMPERATURE_0 < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
          else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
          RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_0, HEAD0_SET_TEMP_VP);
          RTS_currentScreen = 26;
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          heatway = 1;
        }
      }
      else if(recdat.data[0] == 6)
      {
        if(!planner.has_blocks_queued())
        {
          Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
          Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
          RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          RTS_currentScreen = 23;
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 7)
      {
        if(!planner.has_blocks_queued())
        {
          if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
          else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
          RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
          thermalManager.setTargetHotend(FILAMENT_CHANGE_TEMPERATURE_1, 1);
          if (FILAMENT_CHANGE_TEMPERATURE_1 < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
          else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
          RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_1, HEAD1_SET_TEMP_VP);
          RTS_currentScreen = 26;
        RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          heatway = 2;
        }
      }
      else if (recdat.data[0] == 8)
      {
        if(!planner.has_blocks_queued())
        {
          Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
          Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
          RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          RTS_currentScreen = 23;
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 0xF1)
      {
        if(!planner.has_blocks_queued())
        {
          thermalManager.temp_hotend[0].target = 0;
          thermalManager.temp_hotend[1].target = 0;
          RTS_SndData(0, HEAD0_SET_ICON_VP);
          RTS_SndData(0, HEAD1_SET_ICON_VP);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
          RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
          RTS_currentScreen = 23;
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
          Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
          RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          break;
        }
      }
      else if(recdat.data[0] == 0xF0)
      {
        break;
      }
      break;

    case Heater1LoadEnterKey:
      Filament1LOAD = ((float)recdat.data[0]) / 10;
      break;

    case FilamentCheckKey:
      if (recdat.data[0] == 1)
      {
        if(((READ(FIL_RUNOUT_PIN) == 0) && (active_extruder == 0)) || ((READ(FIL_RUNOUT2_PIN) == 0) && (active_extruder == 1)))
        {
          RTS_currentScreen = 20;
          RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        }
        else
        {
          RTS_currentScreen = 23;
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        }
      }
      else if (recdat.data[0] == 2)
      {
        if (stepper.is_awake() == true)
        {
          RTS_SndData(0, MOTOR_FREE_ICON_VP); //motors enabled
        }
        else
        {
          RTS_SndData(1, MOTOR_FREE_ICON_VP); //motors disabled
        }
        RTS_currentScreen = 21;
      RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        Filament0LOAD = (float)FILAMENT_LOAD_LENGTH_0;
        Filament1LOAD = (float)FILAMENT_LOAD_LENGTH_1;
      }
      break;

    case PowerContinuePrintKey:
      if (recdat.data[0] == 1) //screen #36 resume print "yes"
      {
        if (print_job_timer.isRunning) // added to resume from pause
        {
          rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
          queue.enqueue_one_P(PSTR("M117 Resuming..."));
          #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
            pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
          #endif
          wait_for_user = false;
          card.startOrResumeFilePrinting();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          pause_action_flag = false;
          RTS_currentScreen = 11;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          PrintFlag = 2;
          queue.enqueue_now_P(PSTR("M75"));
          TERN_(HOST_PAUSE_M76, host_action_resume());
        }
        else //resume after power loss
        {
          #if ENABLED(DUAL_X_CARRIAGE)
            save_dual_x_carriage_mode = dualXPrintingModeStatus;
            SERIAL_ECHOLNPGM("Debug Mode: PowerContinuePrintKey=1");
            switch(save_dual_x_carriage_mode)
            {
              case 1:
                queue.enqueue_now_P(PSTR("M605 S1"));
                break;
              case 2:
                queue.enqueue_now_P(PSTR("M605 S2"));
                break;
              case 3:
                queue.enqueue_now_P(PSTR("M605 S2 X68 R0"));
                queue.enqueue_now_P(PSTR("M605 S3"));
                break;
              default:
                queue.enqueue_now_P(PSTR("M605 S1"));
                break;
            }
          #endif
          if (recovery.info.recovery_flag)
          {
            power_off_type_yes = 1;
            Update_Time_Value = 0;
            RTS_currentScreen = 11;
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
            // recovery.resume();
            queue.enqueue_now_P(PSTR("M1000"));

            PoweroffContinue = true;
            sdcard_pause_check = true;
            zprobe_zoffset = probe.offset.z;
            RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
            RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
            PrintFlag = 2;
          }
        }
      }
      else if (recdat.data[0] == 2) //screen #36 resume print "no"
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        #if ENABLED(DUAL_X_CARRIAGE)
          extruder_duplication_enabled = false;
          dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
          active_extruder = 0;
        #endif
        RTS_currentScreen = 1;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);

        PoweroffContinue = false;
        sdcard_pause_check = false;
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        thermalManager.disable_all_heaters();
        print_job_timer.reset();

        if(CardReader::flag.mounted)
        {
          #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
            card.removeJobRecoveryFile();
            recovery.info.valid_head = 0;
            recovery.info.valid_foot = 0;
            recovery.close();
          #endif
        }
        wait_for_heatup = wait_for_user = false;
        sd_printing_autopause = false;
        PrintFlag = 0;
      }
      break;

    case PrintSelectModeKey:
      SetExtruderMode(recdat.data[0], false);
      break;

    case XhotendOffsetKey:
      if (recdat.data[0] >= 32768)
      {
        hotend_offset[1].x = (recdat.data[0] - 65536) / 100.0;
        hotend_offset[1].x = hotend_offset[1].x - 0.00001 + X2_MAX_POS;
      }
      else
      {
        hotend_offset[1].x = (recdat.data[0]) / 100.0;
        hotend_offset[1].x = hotend_offset[1].x + 0.00001 + X2_MAX_POS;
      }

      RTS_SndData((hotend_offset[1].x - X2_MAX_POS)* 100, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
      break;

    case YhotendOffsetKey:
      if (recdat.data[0] >= 32768)
      {
        hotend_offset[1].y = (recdat.data[0] - 65536) / 100.0;
        hotend_offset[1].y = hotend_offset[1].y - 0.00001;
      }
      else
      {
        hotend_offset[1].y = (recdat.data[0]) / 100.0;
        hotend_offset[1].y = hotend_offset[1].y + 0.00001;
      }

      RTS_SndData(hotend_offset[1].y * 100, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
      break;

    case StoreMemoryKey:
      if(recdat.data[0] == 0xF1)
      {
        settings.init_eeprom();
        bool zig = false;
        int8_t inStart, inStop, inInc, showcount;
        showcount = 0;
        for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
        {
          // away from origin
          if (zig)
          {
            inStart = 0;
            inStop = GRID_MAX_POINTS_X;
            inInc = 1;
          }
          else
          {
            // towards origin
            inStart = GRID_MAX_POINTS_X - 1;
            inStop = -1;
            inInc = -1;
          }
          zig ^= true;
          for (int x = inStart; x != inStop; x += inInc)
          {
            RTS_SndData(z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
            showcount++;
          }
        }
      queue.enqueue_now_P(PSTR("M420 S1"));
      zprobe_zoffset = 0;
      last_zoffset = 0;
      RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      RTS_SndData(0, MOTOR_FREE_ICON_VP); //motors enabled
      RTS_currentScreen = 21;
      RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
      RTS_SndData((hotend_offset[1].x - X2_MAX_POS) * 100, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
      RTS_SndData(hotend_offset[1].y * 100, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
    }
    else if (recdat.data[0] == 0xF0)
    {
      memset(commandbuf, 0, sizeof(commandbuf));
      sprintf_P(commandbuf, PSTR("M218 T1 X%4.1f"), hotend_offset[1].x);
      queue.enqueue_now_P(commandbuf);
      sprintf_P(commandbuf, PSTR("M218 T1 Y%4.1f"), hotend_offset[1].y);
      queue.enqueue_now_P(commandbuf);
      sprintf_P(commandbuf, PSTR("M218 T1 Z%4.1f"), hotend_offset[1].z);
      queue.enqueue_now_P(commandbuf);
      //settings.save();
      RTS_currentScreen = 35;
      RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
    }
    break;

    case Fan0SpeedEnterKey:
      thermalManager.set_fan_speed(0, recdat.data[0]);
      RTS_SndData(recdat.data[0], HEAD0_FAN_SPEED_VP);
      SERIAL_ECHOLNPGM("Fan0SpeedEnterKey: ", recdat.data[0]);
    break;

    case Fan1SpeedEnterKey:
      thermalManager.set_fan_speed(1, recdat.data[0]);
      RTS_SndData(recdat.data[0], HEAD1_FAN_SPEED_VP);
      SERIAL_ECHOLNPGM("Fan1SpeedEnterKey: ", recdat.data[0]);
    break;

    case Flow0EnterKey:
      planner.set_flow(0, recdat.data[0]);
      RTS_SndData(recdat.data[0], HEAD0_FLOW_RATE_VP);
      SERIAL_ECHOLNPGM("Flow0EnterKey: ", recdat.data[0]);
    break;

    case Flow1EnterKey:
      planner.set_flow(1, recdat.data[0]);
      RTS_SndData(recdat.data[0], HEAD1_FLOW_RATE_VP);
      SERIAL_ECHOLNPGM("Flow1EnterKey: ", recdat.data[0]);
    break;

    case TuneCyclesEnterKey:
      if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7))
      {
        //double beep error notification
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);
        break;
      }
      TuneCycles = recdat.data[0];
      RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP);
      SERIAL_ECHOLNPGM("TuneCyclesEnterKey: ", TuneCycles);
    break;

    case EStepsAdjustKey:
      if (recdat.data[0] == 1)
      {
        ESteps0 = ESteps0 - 1.0;
        ConvertLong = ESteps0 * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t)ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_E_STEPS_VP + 1);
        memset(cmdbuf, 0, sizeof(cmdbuf));
        sprintf_P(cmdbuf, PSTR("M92 E%4.2f T0"), ESteps0);
        queue.enqueue_now_P(commandbuf);
        SERIAL_ECHOLNPGM("EStepsAdjustKey: ", ESteps0);
      }
      else if (recdat.data[0] == 2)
      {
        ESteps0 = ESteps0 + 1.0;
        ConvertLong = ESteps0 * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t) ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD0_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_E_STEPS_VP + 1);
        memset(cmdbuf, 0, sizeof(cmdbuf));
        sprintf_P(cmdbuf, PSTR("M92 E%4.2f T0"), ESteps0);
        queue.enqueue_now_P(commandbuf);
        SERIAL_ECHOLNPGM("EStepsAdjustKey: ", ESteps0);
      }
      else if (recdat.data[0] == 3)
      {
        ESteps1 = ESteps1 - 1.0;
        ConvertLong = ESteps1 * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t) ConvertLong & 0xFFFF;
        RTS_SndData((uint16_t)ConvertH, HEAD1_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_E_STEPS_VP + 1);
        memset(cmdbuf, 0, sizeof(cmdbuf));
        sprintf_P(cmdbuf, PSTR("M92 E%4.2f T1"), ESteps1);
        queue.enqueue_now_P(commandbuf);
        SERIAL_ECHOLNPGM("EStepsAdjustKey: ", ESteps1);
      }
      else if (recdat.data[0] == 4)
      {
        ESteps1 = ESteps1 + 1.0;
        ConvertLong = ESteps1 * 100;
        ConvertH = (uint16_t)(ConvertLong >>16) & 0xFFFF;
        ConvertL = (uint16_t) ConvertLong & 0xFFFF;
        RTS_SndData(ConvertH, HEAD1_E_STEPS_VP);
        RTS_SndData(ConvertL, HEAD1_E_STEPS_VP + 1);
        memset(cmdbuf, 0, sizeof(cmdbuf));
        sprintf_P(cmdbuf, PSTR("M92 E%4.2f T1"), ESteps1);
        queue.enqueue_now_P(commandbuf);
        SERIAL_ECHOLNPGM("EStepsAdjustKey: ", ESteps1);
      }

    break;

    case ESteps0EnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        ESteps0 = (float)ConvertLong/100;
        memset(cmdbuf, 0, sizeof(cmdbuf));
        sprintf_P(cmdbuf, PSTR("M92 E%4.2f T0"), ESteps0);
        queue.enqueue_now_P(commandbuf);
        SERIAL_ECHOLNPGM("ESteps0EnterKey: ", ESteps0);
        RTS_SndData((uint16_t)ConvertH, HEAD0_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_E_STEPS_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD0_E_STEPS_VP, VarAddr_R);
      }
    break;

    case ESteps1EnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        ESteps1 = (float)ConvertLong / 100;
        memset(cmdbuf, 0, sizeof(cmdbuf));
        sprintf_P(cmdbuf, PSTR("M92 E%4.2f T1"), ESteps1);
        queue.enqueue_now_P(commandbuf);
        SERIAL_ECHOLNPGM("ESteps1EnterKey: ", ESteps1);
        RTS_SndData((uint16_t)ConvertH, HEAD1_E_STEPS_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_E_STEPS_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD1_E_STEPS_VP, VarAddr_R);
      }
    break;

    case FeedRateEnterKey:
      EstepFeedRate = (float)recdat.data[0]/10;
      RTS_SndData(EstepFeedRate*10, E_STEPS_FEED_RATE_VP);
      SERIAL_ECHOLNPGM("FeedRateEnterKey: ", EstepFeedRate);
    break;

    case FeedDistEnterKey:
      EstepFeedDistance = (float)recdat.data[0]/10;
      RTS_SndData(EstepFeedDistance*10, E_STEPS_FEED_DIST_VP);
      SERIAL_ECHOLNPGM("FeedDistEnterKey: ", EstepFeedDistance);
    break;

    case FeedKey:
      if (recdat.data[0] == 1) // from screen #91: feed left extruder
      {
        if(!planner.has_blocks_queued())
        {
          //ensure feedrate > 0
          if (EstepFeedRate <= 0.0)
          {
            EstepFeedRate = 6.0;
            SERIAL_ECHOLNPGM("Set minimum feed rate 6 mm/min");
            RTS_SndData(StartSoundSet, SoundAddr);
            wait_idle(250);
          }
          //ensure feed distance > 0
          if (EstepFeedDistance <= 0.0)
          {
            EstepFeedDistance = 1.0;
            SERIAL_ECHOLNPGM("Set minimum feed distance 1 mm");
            RTS_SndData(StartSoundSet, SoundAddr);
            wait_idle(250);
          }
          //ensure filament is present
          if(READ(FIL_RUNOUT_PIN) == 0) //check if filament is present
          {
            RTS_currentScreen = 20;
            RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          else
          {
            //ensure sufficient nozzle temperature
            if(thermalManager.temp_hotend[0].celsius < (FILAMENT_CHANGE_TEMPERATURE_0 - 5))
            {
              RTS_SndData((uint16_t)FILAMENT_CHANGE_TEMPERATURE_0, TUNE_TARGET_TEMP_VP);
              RTS_currentScreen = 98;
              RTS_SndData(ExchangePageBase + 98, ExchangepageAddr);
            }
            else
            {
              current_position[E_AXIS_N(0)] += EstepFeedDistance;
              active_extruder = 0;
              queue.enqueue_now_P(PSTR("T0"));
              // Assert in case of low nozzle temp => New screen!
              RTS_line_to_current(E_AXIS_N(0), MMM_TO_MMS(EstepFeedRate));
              RTS_SndData(EstepFeedRate*10, E_STEPS_FEED_RATE_VP);
              RTS_SndData(EstepFeedDistance*10, E_STEPS_FEED_DIST_VP);
              SERIAL_ECHOLNPGM("Feeding left extruder");
            }
          }
        }
      }
      else if (recdat.data[0] == 2) // from screen #98: ok to heat up
      {
        //heat up left nozzle
        thermalManager.temp_hotend[0].target = FILAMENT_CHANGE_TEMPERATURE_0;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        SERIAL_ECHOLNPGM("Heating up left nozzle to 200°C");
        RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_0, CHANGE_FILAMENT0_TEMP_VP);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
        RTS_currentScreen = 91;
        RTS_SndData(ExchangePageBase + 91, ExchangepageAddr);
        break;
      }
      else if (recdat.data[0] == 3) // from screen #92: feed right extruder
      {
        if(!planner.has_blocks_queued())
        {
          //ensure feedrate > 0
          if (EstepFeedRate <= 0.0)
          {
            EstepFeedRate = 6.0;
            SERIAL_ECHOLNPGM("Set minimum feed rate 6 mm/min");
            RTS_SndData(StartSoundSet, SoundAddr);
            wait_idle(250);
          }
          //ensure feed distance > 0
          if (EstepFeedDistance <= 0.0)
          {
            EstepFeedDistance = 1.0;
            SERIAL_ECHOLNPGM("Set minimum feed distance 1 mm");
            RTS_SndData(StartSoundSet, SoundAddr);
            wait_idle(250);
          }
          //ensure filament is present
          if(READ(FIL_RUNOUT2_PIN) == 0) //check if filament is present
          {
            RTS_currentScreen = 20;
            RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          else
          {
            //ensure sufficient nozzle temperature
            if(thermalManager.temp_hotend[1].celsius < (FILAMENT_CHANGE_TEMPERATURE_1 - 5))
            {
              RTS_SndData((uint16_t)FILAMENT_CHANGE_TEMPERATURE_1, TUNE_TARGET_TEMP_VP);
              RTS_currentScreen = 99;
              RTS_SndData(ExchangePageBase + 99, ExchangepageAddr);
            }
            else
            {
              current_position[E_AXIS_N(1)] += EstepFeedDistance;
              active_extruder = 1;
              queue.enqueue_now_P(PSTR("T1"));
              // Assert in case of low nozzle temp => New screen!
              RTS_line_to_current(E_AXIS_N(1), MMM_TO_MMS(EstepFeedRate));
              RTS_SndData(EstepFeedDistance*10, E_STEPS_FEED_DIST_VP);
              SERIAL_ECHOLNPGM("Feeding right extruder");
            }
          }
        }
      }
      else if (recdat.data[0] == 4) // from screen #99: ok to heat up
      {
        //heat up right nozzle
        thermalManager.temp_hotend[1].target = FILAMENT_CHANGE_TEMPERATURE_1;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        SERIAL_ECHOLNPGM("Heating up right nozzle to 200°C");
        RTS_SndData(FILAMENT_CHANGE_TEMPERATURE_1, CHANGE_FILAMENT1_TEMP_VP);
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
        RTS_currentScreen = 92;
        RTS_SndData(ExchangePageBase + 92, ExchangepageAddr);
        break;
      }
    break;

    case PrintFileKey:
    if (recdat.data[0] == 1)
      {
        if((dualXPrintingModeStatus != 0) && (dualXPrintingModeStatus != 4))
        {
          RTS_SndData(dualXPrintingModeStatus, SELECT_MODE_ICON_VP);
        }
        else if(dualXPrintingModeStatus == 4)
        {
          RTS_SndData(5, SELECT_MODE_ICON_VP);   //SINGLE MODE R icon
        }
        else
        {
          RTS_SndData(4, SELECT_MODE_ICON_VP);   //SINGLE MODE L icon
        }
        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
        RTS_currentScreen = 1;
        RTS_SndData(ExchangePageBase + 56, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2)
      {
        //dir back
        card.cdup();
        InitCardList();
      }
      else if(recdat.data[0] == 3)
      {
        //page left
        ShowFilesOnCardPage(fileInfo.currentPage-1);
      }
      else if(recdat.data[0] == 4)
      {
        //page right
        ShowFilesOnCardPage(fileInfo.currentPage+1);
      }
      else if (recdat.data[0] == 5)
      {
        //change text scrolling speed simultaniously
        FileScrollSpeed ++;
        if (FileScrollSpeed > 3) {FileScrollSpeed = 0;}
        for (int j = 0; j < 8; j++) //set scroll speed
        {
          RTS_SndData((unsigned int)FileScrollSpeed, PRINT_FILE_TEXT_SP + (j * 32));
        }
      }
      else if ((recdat.data[0] == 11) && RTS_SD_Detected())
      {
        if (!card.fileExists(fileInfo.currentFilePath)) {
          break;
        }

        char cmd[MAX_CMD_SIZE+16];
        char *c;
        sprintf_P(cmd, PSTR("M23 %s"), fileInfo.currentFilePath);
        for (c = &cmd[4]; *c; c++)
          *c = tolower(*c);

        memset(cmdbuf, 0, sizeof(cmdbuf));
        strcpy(cmdbuf, cmd);

        save_dual_x_carriage_mode = dualXPrintingModeStatus;
        SERIAL_ECHOLNPGM("Debug Mode: PrintFileKey=11; save_dual_x_carriage_mode=" , save_dual_x_carriage_mode);
        SERIAL_ECHOLNPGM("dualXPrintingModeStatus: ", dualXPrintingModeStatus);

        switch(save_dual_x_carriage_mode)
        {
          case 1:
            queue.enqueue_now_P(PSTR("M605 S1"));
            break;
          case 2:
            queue.enqueue_now_P(PSTR("M605 S2"));
            break;
          case 3:
            queue.enqueue_now_P(PSTR("M605 S2 X68 R0"));
            queue.enqueue_now_P(PSTR("M605 S3"));
            break;
          default:
            queue.enqueue_now_P(PSTR("M605 S1"));
            break;
        }
        queue.enqueue_one_now(cmd);
        delay(20);
        queue.enqueue_now_P(PSTR("M24"));
        // clean screen.
        for (int j = 0; j < FileNameLen; j ++)
        {
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        }

        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);

        delay(2);

        #if ENABLED(BABYSTEPPING)
          RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
        #endif
        feedrate_percentage = 100;
        RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
        zprobe_zoffset = last_zoffset;
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        PoweroffContinue = true;
        RTS_currentScreen = 11;
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        Update_Time_Value = 0;
        PrintFlag = 2;
        change_page_number = 11;
      }
      else if(recdat.data[0] == 12)
      {
        RTS_currentScreen = 1;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      }
      break;

    case SelectFileKey:
      if (RTS_SD_Detected())
      {
        fileInfo.pageFileIndex = recdat.data[0]-5; //5=> 1st entry, 9=> 5th entry
        int index = 5*(fileInfo.currentPage-1) + fileInfo.pageFileIndex;

        if (index >= card.countFilesInWorkDir())
        {
          break;
        }

        card.selectFileByIndex(index);
        strncpy(fileInfo.currentDisplayFilename, card.longest_filename(), FileNameLen);
        for (int j = 0; j < MAXPATHNAMELENGTH; j ++)
        {
          fileInfo.currentFilePath[j] = 0;
        }
        card.getAbsFilenameInCWD(fileInfo.currentFilePath);
        strcat(fileInfo.currentFilePath, card.filename);

        if (!EndsWith(card.filename, "GCO"))
        {
          card.cd(fileInfo.currentFilePath);
          InitCardList();
          break;
        }

        for (int j = 0; j < FileNameLen; j ++)
        {
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        }

        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
        delay(2);

        if (hasSelected)
        {
          RTS_SndData((unsigned int)0xA514, FilenameNature + previousSelectionIndex * 32);
        }

        previousSelectionIndex = recdat.data[0] - 3;
        hasSelected = true;

        RTS_SndData((unsigned int)0x073F, FilenameNature + (recdat.data[0] - 3) * 32);
      }
      break;

    case SaveEEPROM:
      if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7)) //check if tuning is in process
      {
        //only abort key will be processed during tuning
        //double beep error notification
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);
        break;
      }
      if (recdat.data[0] == 1)
      {
        settings.save(); //save all settings
        RTS_SndData(StartSoundSet, SoundAddr);
      }
    break;

    case AutoTuneEnterKey:
      if ((RTS_cyclesIcon > 0) && (RTS_cyclesIcon < 7)) //check if tuning is in process
      {
        if ( !(recdat.data[0] == 7) )
        {
          //only abort key will be processed during tuning
          //double beep error notification
          RTS_SndData(StartSoundSet, SoundAddr);
          wait_idle(150);
          RTS_SndData(StartSoundSet, SoundAddr);
          break;
        }
      }
      if (recdat.data[0] == 1) //tune left nozzle
      {
        //ensure sufficient target temperature
        if(thermalManager.temp_hotend[0].target < AutoTuneNozzleLowerLimit)
        {
          RTS_SndData((uint16_t)AutoTuneNozzleLowerLimit, TUNE_TARGET_TEMP_VP);
          RTS_currentScreen = 101;
          RTS_SndData(ExchangePageBase + 101, ExchangepageAddr);
        }
        else
        {
          BackupPID = 1;
          BackupKp = thermalManager.temp_hotend[0].pid.Kp;
          BackupKi = thermalManager.temp_hotend[0].pid.Ki;
          BackupKd = thermalManager.temp_hotend[0].pid.Kd;
          SERIAL_ECHOLNPGM("Started left extruder autotuning");
          thermalManager.PID_autotune(thermalManager.temp_hotend[0].target, (heater_id_t)H_E0, TuneCycles, true);
          SERIAL_ECHOLNPGM("Left extruder autotuning finished");
          BackupPID=0;
          RTS_currentScreen = 93;
          RTS_SndData(ExchangePageBase + 93, ExchangepageAddr);
        }
        break;
      }
      else if (recdat.data[0] == 2) //tune right nozzle
      {
        //ensure sufficient target temperature
        if(thermalManager.temp_hotend[1].target < AutoTuneNozzleLowerLimit)
        {
          RTS_SndData((uint16_t)AutoTuneNozzleLowerLimit, TUNE_TARGET_TEMP_VP);
          RTS_currentScreen = 102;
          RTS_SndData(ExchangePageBase + 102, ExchangepageAddr);
        }
        else
        {
          BackupPID=2;
          BackupKp = thermalManager.temp_hotend[1].pid.Kp;
          BackupKi = thermalManager.temp_hotend[1].pid.Ki;
          BackupKd = thermalManager.temp_hotend[1].pid.Kd;
          SERIAL_ECHOLNPGM("Started right extruder autotuning");
          thermalManager.PID_autotune(thermalManager.temp_hotend[1].target, (heater_id_t)H_E1, TuneCycles, true);
          SERIAL_ECHOLNPGM("Right extruder autotuning finished");
          BackupPID=0;
          RTS_currentScreen = 94;
          RTS_SndData(ExchangePageBase + 94, ExchangepageAddr);
        }
        break;
      }
      else if (recdat.data[0] == 3) //tune hot-bed
      {
        //ensure sufficient target temperature
        if(thermalManager.temp_bed.target < AutoTuneHotBedLowerLimit)
        {
          RTS_SndData((uint16_t)AutoTuneHotBedLowerLimit, TUNE_TARGET_TEMP_VP);
          RTS_currentScreen = 103;
          RTS_SndData(ExchangePageBase + 103, ExchangepageAddr);
        }
        else
        {
          BackupPID=3;
          BackupKp = thermalManager.temp_bed.pid.Kp;
          BackupKi = thermalManager.temp_bed.pid.Ki;
          BackupKd = thermalManager.temp_bed.pid.Kd;
          SERIAL_ECHOLNPGM("Started hot.bed autotuning");
          thermalManager.PID_autotune(thermalManager.temp_bed.target, (heater_id_t)H_BED, TuneCycles, true);
          SERIAL_ECHOLNPGM("Hot-bed autotuning finished");
          BackupPID=0;
          RTS_currentScreen = 95;
          RTS_SndData(ExchangePageBase + 95, ExchangepageAddr);
        }
        break;
      }
      else if (recdat.data[0] == 4) // from screen #101: ok to heat up
      {
        //heat up left nozzle
        thermalManager.temp_hotend[0].target = AutoTuneNozzleLowerLimit;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        SERIAL_ECHOLNPGM("Set left nozzle target temperature to ", AutoTuneNozzleLowerLimit , " °C");
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
        RTS_currentScreen = 93;
        RTS_SndData(ExchangePageBase + 93, ExchangepageAddr);
        break;
      }
      else if (recdat.data[0] == 5) // from screen #102: ok to heat up
      {
        //heat up right nozzle
        thermalManager.temp_hotend[1].target = AutoTuneNozzleLowerLimit;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        SERIAL_ECHOLNPGM("Set right nozzle target temperature to ", AutoTuneNozzleLowerLimit , " °C");
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD0_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD0_CURRENT_TEMP_VP);
        RTS_currentScreen = 94;
        RTS_SndData(ExchangePageBase + 94, ExchangepageAddr);
        break;
      }
      else if (recdat.data[0] == 6) // from screen #103: ok to heat up
      {
        //heat up hot-bed
        thermalManager.setTargetBed(AutoTuneHotBedLowerLimit);
        SERIAL_ECHOLNPGM("Set hot-bed  nozzle target temperature to ", AutoTuneHotBedLowerLimit , " °C");
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
        RTS_currentScreen = 95;
        RTS_SndData(ExchangePageBase + 95, ExchangepageAddr);
        break;
      }
      else if (recdat.data[0] == 7) //abort tuning
      {
        SERIAL_ECHOLNPGM("Autotuning aborted. Previous PID parameters restored.");
        //reset Autotuning indicator
        RTS_cyclesIcon = 0;
        //triple beep abort notification
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);
        wait_idle(150);
        RTS_SndData(StartSoundSet, SoundAddr);

        TERN_(HAS_RESUME_CONTINUE, wait_for_user = false);
        wait_for_heatup = false;
        //turn off all heaters
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD0_SET_ICON_VP);
        RTS_SndData(0, HEAD0_SET_TEMP_VP);
        thermalManager.setTargetHotend(0, 1);
        RTS_SndData(0, HEAD1_SET_ICON_VP);
        RTS_SndData(0, HEAD1_SET_TEMP_VP);
        if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD0_CURRENT_ICON_VP);} //black
        else {RTS_SndData(1, HEAD0_CURRENT_ICON_VP);} //red data background
        if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
        else {RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
        RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
        RTS_SndData(0, TUNE_PROGRESS_ICON_VP);
        //reset PID data
        switch(BackupPID)
        {
          case 1:
            thermalManager.temp_hotend[0].pid.Kp = BackupKp;
            thermalManager.temp_hotend[0].pid.Ki = BackupKi;
            thermalManager.temp_hotend[0].pid.Kd = BackupKd;
            break;
          case 2:
            thermalManager.temp_hotend[1].pid.Kp = BackupKp;
            thermalManager.temp_hotend[1].pid.Ki = BackupKi;
            thermalManager.temp_hotend[1].pid.Kd = BackupKd;
            break;
          case 3:
            thermalManager.temp_bed.pid.Kp = BackupKp;
            thermalManager.temp_bed.pid.Ki = BackupKi;
            thermalManager.temp_bed.pid.Kd = BackupKd;
            break;
          default:
            break;
        }
        BackupPID=0;
      }
      else if (recdat.data[0] == 10) // from screens #93,#94,#95: tuning cycles -
      {
        TuneCycles--;
        if (TuneCycles < TuneCyclesLowerLimit) {TuneCycles = TuneCyclesLowerLimit;}
        RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP);
        SERIAL_ECHOLNPGM("TuneCycles--: ", TuneCycles);
        break;
      }
      else if (recdat.data[0] == 11) // from screens #93,#94,#95: tuning cycles +
      {
        TuneCycles++;
        if (TuneCycles > TuneCyclesUpperLimit) {TuneCycles = TuneCyclesUpperLimit;}
        RTS_SndData(TuneCycles, SET_TUNE_CYCLES_VP);
        SERIAL_ECHOLNPGM("TuneCycles++: ", TuneCycles);
        break;
      }
      else if (recdat.data[0] == 12) // from screen #93: left nozzle temperature -
      {
        celsius_t t_temp = thermalManager.temp_hotend[0].target - 1;
        if (t_temp < Head0TempLowerLimit) {t_temp = Head0TempLowerLimit;}
        thermalManager.setTargetHotend(t_temp, 0);
        if (t_temp < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(t_temp, HEAD0_SET_TEMP_VP);
        SERIAL_ECHOLNPGM("Heater0Temp--: ", t_temp);
        break;
      }
      else if (recdat.data[0] == 13) // from screen #93: left nozzle temperature +
      {
        celsius_t t_temp = thermalManager.temp_hotend[0].target + 1;
        if (t_temp > Head0TempUpperLimit) {t_temp = Head0TempUpperLimit;}
        thermalManager.setTargetHotend(t_temp, 0);
        if (t_temp < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
        RTS_SndData(t_temp, HEAD0_SET_TEMP_VP);
        SERIAL_ECHOLNPGM("Heater0Temp++: ", t_temp);
        break;
      }
      else if (recdat.data[0] == 14) // from screen #94: right nozzle temperature -
      {
        celsius_t t_temp = thermalManager.temp_hotend[1].target - 1;
        if (t_temp < Head1TempLowerLimit) {t_temp = Head1TempLowerLimit;}
        thermalManager.setTargetHotend(t_temp, 1);
        if (t_temp < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(t_temp, HEAD1_SET_TEMP_VP);
        SERIAL_ECHOLNPGM("Heater1Temp--: ", t_temp);
        break;
      }
      else if (recdat.data[0] == 15) // from screen #94: right nozzle temperature +
      {
        celsius_t t_temp = thermalManager.temp_hotend[1].target + 1;
        if (t_temp > Head1TempUpperLimit) {t_temp = Head1TempUpperLimit;}
        thermalManager.setTargetHotend(t_temp, 0);
        if (t_temp < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
        RTS_SndData(t_temp, HEAD1_SET_TEMP_VP);
        SERIAL_ECHOLNPGM("Heater1Temp++: ", t_temp);
        break;
      }
      else if (recdat.data[0] == 16) // from screen #95: hot-bed temperature -
      {
        celsius_t t_temp = thermalManager.temp_bed.target - 1;
        if (t_temp < BedTempLowerLimit) {t_temp = BedTempLowerLimit;}
        thermalManager.setTargetBed(t_temp);
        RTS_SndData(t_temp, BED_SET_TEMP_VP);
        SERIAL_ECHOLNPGM("HotBedTemp--: ", t_temp);
        break;
      }
      else if (recdat.data[0] == 17) // from screen #95: hot-bed temperature +
      {
        celsius_t t_temp = thermalManager.temp_bed.target + 1;
        if (t_temp > BedTempUpperLimit) {t_temp = BedTempUpperLimit;}
        thermalManager.setTargetBed(t_temp);
        RTS_SndData(t_temp, BED_SET_TEMP_VP);
                SERIAL_ECHOLNPGM("HotBedTemp++: ", t_temp);
        break;
      }
    break;

    case RestartKey:  //'Restart' screen #86 receives key codes 0xF1 (Restart) / 0xF0 (Cancel)
      RTS_SndData(StartSoundSet, SoundAddr);
      if(recdat.data[0] == 0xF1) //Restart printer now
      {
        RTS_Restart(); //restart
      }
      else if(recdat.data[0] == 0xF0) //Cancel, switch back to settings screen
      {
        RTS_currentScreen = 21;
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr); //call 'settings' screen
      }
      break;

    case SuicideKey:  //'Power Off' screen #88 receives key codes 0xF1 (Yes) / 0xF0 (No)
      RTS_SndData(StartSoundSet, SoundAddr);
      if(recdat.data[0] == 0xF1) //Yes, power off printer now
      {
        autoPowerOffEnabled = true;
        queue.enqueue_now_P(PSTR("M81")); //power off
      }
      else if(recdat.data[0] == 0xF0) //No, switch back to settings screen
      {
        if (stepper.is_awake() == true)
        {
          RTS_SndData(0, MOTOR_FREE_ICON_VP); //motors enabled
        }
        else
        {
          RTS_SndData(1, MOTOR_FREE_ICON_VP); //motors disabled
        }
        RTS_currentScreen = 90;
        RTS_SndData(ExchangePageBase + 90, ExchangepageAddr); //call 'Extra settings' screen
      }
      break;

    case Head0PIDKpEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKp0 = (float)ConvertLong/100;
        PID_PARAM(Kp, 0) = PIDparamKp0;
        SERIAL_ECHOLNPGM("Head0PIDKpEnterKey: ", PIDparamKp0);
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KP_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD0_TUNE_KP_VP, VarAddr_R);
      }
    break;

    case Head0PIDKiEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKi0 = (float)ConvertLong/100;
        PID_PARAM(Ki, 0) = scalePID_i(PIDparamKi0);
        SERIAL_ECHOLNPGM("Head0PIDKiEnterKey (unscaled): ", PIDparamKi0);
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KI_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD0_TUNE_KI_VP, VarAddr_R);
      }
    break;

    case Head0PIDKdEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKd0 = (float)ConvertLong/100;
        PID_PARAM(Kd, 0) = scalePID_d(PIDparamKd0);
        SERIAL_ECHOLNPGM("Head0PIDKdEnterKey (unscaled): ", PIDparamKd0);
        RTS_SndData((uint16_t)ConvertH, HEAD0_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD0_TUNE_KD_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD0_TUNE_KD_VP, VarAddr_R);
      }
    break;

    case Head1PIDKpEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKp1 = (float)ConvertLong/100;
        PID_PARAM(Kp, 1) = PIDparamKp1;
        SERIAL_ECHOLNPGM("Head1PIDKpEnterKey: ", PIDparamKp1);
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KP_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD1_TUNE_KP_VP, VarAddr_R);
      }
    break;

    case Head1PIDKiEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKi1 = (float)ConvertLong/100;
        PID_PARAM(Ki, 1) = scalePID_i(PIDparamKi1);
        SERIAL_ECHOLNPGM("Head1PIDKiEnterKey (unscaled): ", PIDparamKi1);
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KI_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD1_TUNE_KI_VP, VarAddr_R);
      }
    break;

    case Head1PIDKdEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKd1 = (float)ConvertLong/100;
        PID_PARAM(Kd, 1) = scalePID_d(PIDparamKd1);
        SERIAL_ECHOLNPGM("Head1PIDKdEnterKey (unscaled): ", PIDparamKd1);
        RTS_SndData((uint16_t)ConvertH, HEAD1_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, HEAD1_TUNE_KD_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, HEAD1_TUNE_KD_VP, VarAddr_R);
      }
    break;

    case BedPIDKpEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKpB = (float)ConvertLong/100;
        thermalManager.temp_bed.pid.Kp = PIDparamKpB;
        SERIAL_ECHOLNPGM("BedPIDKpEnterKey: ", PIDparamKpB);
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KP_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KP_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, BED_TUNE_KP_VP, VarAddr_R);
      }
    break;

    case BedPIDKiEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKiB = (float)ConvertLong/100;
        thermalManager.temp_bed.pid.Ki = scalePID_i(PIDparamKiB);
        SERIAL_ECHOLNPGM("BedPIDKiEnterKey (unscaled): ", PIDparamKiB);
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KI_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KI_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, BED_TUNE_KI_VP, VarAddr_R);
      }
    break;

    case BedPIDKdEnterKey:
      if (recdat.bytelen == 4) //read from display memory
      {
        ConvertLong = 0;
        ConvertH = recdat.data[0];
        ConvertL = recdat.data[1];
        ConvertLong = ConvertH;
        ConvertLong = ConvertLong<<16;
        ConvertLong = ConvertLong | ConvertL;
        PIDparamKdB = (float)ConvertLong/100;
        thermalManager.temp_bed.pid.Kd = scalePID_d(PIDparamKdB);
        SERIAL_ECHOLNPGM("BedPIDKdEnterKey (unscaled): ", PIDparamKdB);
        RTS_SndData((uint16_t)ConvertH, BED_TUNE_KD_VP);
        RTS_SndData((uint16_t)ConvertL, BED_TUNE_KD_VP + 1);
      }
      else //trigger read callback
      {
        RTS_SndData(4, BED_TUNE_KD_VP, VarAddr_R);
      }
    break;

    case PresetPlaHotendKey:
      if (recdat.data[0] < 0)
      {
        RTS_presets.pla_hotend_t = 0; //Input < 0 will be set to 0
      }
      else
      {
        RTS_presets.pla_hotend_t = recdat.data[0];
      }
      RTS_SndData(RTS_presets.pla_hotend_t, PRESET_PLA_HOTEND_VP);
      SERIAL_ECHOLNPGM("PresetPlaHotendKey: ", RTS_presets.pla_hotend_t);
    break;

    case PresetPlaBedKey:
      if (recdat.data[0] < 0)
      {
        RTS_presets.pla_bed_t = 0; //Input < 0 will be set to 0
      }
      else
      {
        RTS_presets.pla_bed_t = recdat.data[0];
      }
      RTS_SndData(RTS_presets.pla_bed_t, PRESET_PLA_BED_VP);
      SERIAL_ECHOLNPGM("PresetPlaBedKey: ", RTS_presets.pla_bed_t);
    break;

    case PresetPetgHotendKey:
      if (recdat.data[0] < 0)
      {
        RTS_presets.petg_hotend_t = 0; //Input < 0 will be set to 0
      }
      else
      {
        RTS_presets.petg_hotend_t = recdat.data[0];
      }
      RTS_SndData(RTS_presets.petg_hotend_t, PRESET_PETG_HOTEND_VP);
      SERIAL_ECHOLNPGM("PresetPetgHotendKey: ", RTS_presets.petg_hotend_t);
    break;

    case PresetPetgBedKey:
      if (recdat.data[0] < 0)
      {
        RTS_presets.petg_bed_t = 0; //Input < 0 will be set to 0
      }
      else
      {
        RTS_presets.petg_bed_t = recdat.data[0];
      }
      RTS_SndData(RTS_presets.petg_bed_t, PRESET_PETG_BED_VP);
      SERIAL_ECHOLNPGM("PresetPetgBedKey: ", RTS_presets.petg_bed_t);
    break;

    case PresetMotorHoldKey:
      if (recdat.data[0] < 0)
      {
        RTS_presets.motor_hold_time = 0; //Input < 0 will be set to 0
      }
      else
      {
        RTS_presets.motor_hold_time = recdat.data[0];
      }
      RTS_SndData(RTS_presets.motor_hold_time, PRESET_MOTOR_HOLD_TIME_VP);
      SERIAL_ECHOLNPGM("PresetMotorHoldKey: ", RTS_presets.motor_hold_time);
    break;

    case PresetAutoOffKey: //Toggle Auto Power-Off
      if (RTS_presets.auto_power_off_enable)
      {
        RTS_SndData(1, PRESET_AUTO_POWER_OFF_VP);   //Send "Off"-Icon
        RTS_presets.auto_power_off_enable = false;  //Preset Auto Power-Off is disabled now
        RTS_SndData(1, AUTO_POWER_OFF_ICON_VP);     //Send "Off"-Icon
        autoPowerOffEnabled = false;                //Override current Auto Power-Off setting & disable
      }
      else
      {
        RTS_SndData(0, PRESET_AUTO_POWER_OFF_VP); //Send "On"-Icon
        RTS_presets.auto_power_off_enable = true; //Preset Auto Power-Off is enabled now
        RTS_SndData(0, AUTO_POWER_OFF_ICON_VP);    //Send "On"-Icon
        autoPowerOffEnabled = true;                //Override curent Auto Power-Off setting & enable
      }
    break;

    case ChangePageKey:
      SERIAL_ECHOLNPGM("Change Page: ", change_page_number);
      if ((change_page_number == 36) || (change_page_number == 76))
      {
        break;
      }
      else if (change_page_number == 11)
      {
        RTS_currentScreen = ExchangePageBase;
        RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
        if ((dualXPrintingModeStatus != 0) && (dualXPrintingModeStatus != 4))
        {
          RTS_SndData(dualXPrintingModeStatus, PRINT_MODE_ICON_VP);
        }
        else if (dualXPrintingModeStatus == 4)
        {
          RTS_SndData(5, PRINT_MODE_ICON_VP); //DXC_SINGLE_R_MODE => SINGLE MODE 2
        }
        else
        {
          RTS_SndData(4, PRINT_MODE_ICON_VP); //DXC_SINGLE_L_MODE => SINGLE MODE 1
        }
      }
      else if (change_page_number == 12)
      {
        RTS_currentScreen = ExchangePageBase;
        RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
        if ((dualXPrintingModeStatus != 0) && (dualXPrintingModeStatus != 4))
        {
          RTS_SndData(dualXPrintingModeStatus, PRINT_MODE_ICON_VP);
        }
        else if (dualXPrintingModeStatus == 4)
        {
          RTS_SndData(5, PRINT_MODE_ICON_VP); //DXC_SINGLE_R_MODE => SINGLE MODE 2
        }
        else
        {
          RTS_SndData(4, PRINT_MODE_ICON_VP); //DXC_SINGLE_L_MODE => SINGLE MODE 1
        }
      }
      else
      {
        RTS_currentScreen = ExchangePageBase;
        RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
        change_page_number = 1;
      }
      if ((dualXPrintingModeStatus != 0) && (dualXPrintingModeStatus != 4))
      {
        RTS_SndData(dualXPrintingModeStatus, SELECT_MODE_ICON_VP);
      }
      else if (dualXPrintingModeStatus == 4)
      {
        RTS_SndData(5, SELECT_MODE_ICON_VP); //DXC_SINGLE_R_MODE => SINGLE MODE 2
      }
      else
      {
        RTS_SndData(4, SELECT_MODE_ICON_VP); //DXC_SINGLE_L_MODE => SINGLE MODE 1
      }

      for (int i = 0; i < card.get_num_Files(); i ++)
      {
        for (int j = 0; j < FileNameLen; j ++)
        {
          //RTS_SndData(0, FILE1_TEXT_VP + i * FileNameLen + j);
          RTS_SndData(0, FILE1_TEXT_VP + i * 256 + j);
        }
      }

      for (int i = 1; i < 5; i++)
      {
        RTS_SndData((unsigned int)0xA514, FilenameNature + (i + 1) * 32);
      }

      for (int j = 0; j < FileNameLen; j ++)
      {
        // clean screen.
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        // clean dirname
        RTS_SndData(0, SELECT_DIR_TEXT_VP + j);
      }

      RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);

      // represents to update file list
      if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
      {
        card.cdroot();
        InitCardList();
      }

      char sizeBuf[20];
      sprintf(sizeBuf, "%d X %d X %d", X_MAX_POS - 2, Y_MAX_POS - 2, Z_MAX_POS);
      RTS_SndData(MACHINE_NAME, PRINTER_MACHINE_TEXT_VP);
      RTS_SndData(sizeBuf, PRINTER_PRINTSIZE_TEXT_VP);
      RTS_SndData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
      RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
      RTS_SndData(DISPLAY_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);

      RTS_SndData(thermalManager.fan_speed[0], HEAD0_FAN_SPEED_VP);
      RTS_SndData(thermalManager.fan_speed[1], HEAD1_FAN_SPEED_VP);
      Percentrecord = card.percentDone() + 1;
      if (Percentrecord <= 100)
      {
        rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
      }
      rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);

      RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {RTS_SndData(0, HEAD0_SET_ICON_VP);}
      else {RTS_SndData(1, HEAD0_SET_ICON_VP);}
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
      if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {RTS_SndData(0, HEAD1_SET_ICON_VP);}
      else {RTS_SndData(1, HEAD1_SET_ICON_VP);}
      RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      if (autoPowerOffEnabled == true)
      {
        RTS_SndData(0, AUTO_POWER_OFF_ICON_VP);
      }
      else
      {
        RTS_SndData(1, AUTO_POWER_OFF_ICON_VP);
      }
      break;

    default:
      break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

void RTSSHOW::RTS_AutoBedLevelPage()
{
  if(RTS_waitway == 3)
  {
    RTS_currentScreen = 22;
    rtscheck.RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
    RTS_waitway = 0;
  }
}

void RTSSHOW::RTS_ViewMesh()
{
    //Mesh visualization
    int8_t x, y, color;
    float z_val, max_bed_delta, min_bed_delta, delta;
    int8_t inStart, inStop, inInc, showcount;
    // Probe in reverse order for every other row/column
    bool zig = GRID_MAX_POINTS_Y & 1;// Always end at RIGHT and BACK_PROBE_BED_POSITION
    //find max + min bed delta
    max_bed_delta = 0.0;
    min_bed_delta = 0.0;
    showcount = 0;
    for (y = 0; y < GRID_MAX_POINTS_Y; y++)
    {
      for (x = 0; x < GRID_MAX_POINTS_X; x++)
      {
        z_val = z_values[x][y];
        if (z_val > max_bed_delta)
        {
          max_bed_delta = z_val;
        }
        if (z_val < min_bed_delta)
        {
          min_bed_delta = z_val;
        }
      }
    }
    //calculate 25 color shadings for positive & negative deviation
    delta = max (max_bed_delta,(-1 * min_bed_delta)) / 25;
    //prepare mesh viewer page
    for (y = 0; y < GRID_MAX_POINTS_Y; y++)
    {
      // zag towards origin
      if (zig)
      {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else
      {
        // zag towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      zig ^= true;
      for (x = inStart; x != inStop; x += inInc)
      {
        color = LROUND(z_values[x][y]/delta);
        if (color < 0)
        {
          color = 25 - color;
        }
        RTS_SndData(color , MESH_VISUAL_ICON_VP + showcount * 2);
        RTS_SndData(z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
        showcount++;
      }
    }
    RTS_currentScreen = 81;
    RTS_SndData(ExchangePageBase + 81, ExchangepageAddr); //call 'Mesh view' page
}

int EndsWith(const char *str, const char *suffix)
{
    if (!str || !suffix)
        return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix >  lenstr)
        return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

void EachMomentUpdate()
{
  millis_t ms = millis();
  if(ms > next_rts_update_ms)
  {
    // print the file before the power is off.
    if((power_off_type_yes == 0) && lcd_sd_status && (recovery.info.recovery_flag == true))
    {
      RTS_currentScreen = 0;
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START1_PROCESS_ICON_VP);
      }
      delay(15);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        #if ENABLED(POWER_LOSS_RECOVERY)
          if (card.jobRecoverFileExists()) {
            rtscheck.RTS_SndData(recovery.filename, PRINT_FILE_TEXT_VP);
            RTS_currentScreen = 36;
            rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
          }
        #endif
      }
      return;
    }
    else if((power_off_type_yes == 0) && (recovery.info.recovery_flag == false))
    {
      RTS_currentScreen = 0;
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START1_PROCESS_ICON_VP);
      }
      delay(15);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        Update_Time_Value = RTS_UPDATE_VALUE;
        RTS_currentScreen = 1;
        rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_number = 1;
      }
      return;
    }
    else
    {
      // need to optimize
      if(recovery.info.print_job_elapsed != 0)
      {
        duration_t elapsed = print_job_timer.duration();
        static unsigned char last_cardpercentValue = 100;
        rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
        rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);

        if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
        {
          if((unsigned char) card.percentDone() > 0)
          {
            Percentrecord = card.percentDone();
            if(Percentrecord <= 100)
            {
              rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
            }
            // Estimate remaining time every 20 seconds
            static millis_t next_remain_time_update = 0;
            if(ELAPSED(ms, next_remain_time_update))
            {
              if((save_dual_x_carriage_mode == 0) && (thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target - 5)))
              {
                remain_time = elapsed.value / (Percentrecord * 0.01f) - elapsed.value;
                next_remain_time_update += 20 * 1000UL;
                rtscheck.RTS_SndData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
                rtscheck.RTS_SndData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
              }
              else if((save_dual_x_carriage_mode != 0) && (thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_hotend[1].celsius >= (thermalManager.temp_hotend[1].target - 5)))
              {
                remain_time = elapsed.value / (Percentrecord * 0.01f) - elapsed.value;
                next_remain_time_update += 20 * 1000UL;
                rtscheck.RTS_SndData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
                rtscheck.RTS_SndData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
              }
            }
          }
          else
          {
            rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
            rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
            rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
          }
          rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
          last_cardpercentValue = card.percentDone();
          rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        }
      }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        if((active_extruder == 1) && (save_dual_x_carriage_mode == 1))
        {
          queue.enqueue_now_P(PSTR("G0 F3000 X362"));
        }
        else
        {
          queue.enqueue_now_P(PSTR("G0 F3000 X-62"));
        }
      }
      if (thermalManager.temp_hotend[0].celsius < NozzleWarningLimit) {rtscheck.RTS_SndData(0, HEAD0_CURRENT_ICON_VP);}
      else {rtscheck.RTS_SndData(1, HEAD0_CURRENT_ICON_VP);}
      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
      if (thermalManager.temp_hotend[1].celsius < NozzleWarningLimit) {rtscheck.RTS_SndData(0, HEAD1_CURRENT_ICON_VP);}
      else {rtscheck.RTS_SndData(1, HEAD1_CURRENT_ICON_VP);}
      rtscheck.RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature[1] != thermalManager.temp_hotend[1].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        if (thermalManager.temp_hotend[0].target < NozzleWarningLimit) {rtscheck.RTS_SndData(0, HEAD0_SET_ICON_VP);}
        else {rtscheck.RTS_SndData(1, HEAD0_SET_ICON_VP);}
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        if (thermalManager.temp_hotend[1].target < NozzleWarningLimit) {rtscheck.RTS_SndData(0, HEAD1_SET_ICON_VP);}
        else {rtscheck.RTS_SndData(1, HEAD1_SET_ICON_VP);}
        rtscheck.RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature[1] = thermalManager.temp_hotend[1].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        RTS_currentScreen = 23;
        rtscheck.RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        heatway = 0;
        rtscheck.RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
      }
      else if((thermalManager.temp_hotend[1].celsius >= thermalManager.temp_hotend[1].target) && (heatway == 2))
      {
        RTS_currentScreen = 23;
        rtscheck.RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        heatway = 0;
        rtscheck.RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
      }

      rtscheck.RTS_SndData(AutoHomeIconNum ++, AUTO_HOME_DISPLAY_ICON_VP);
      if (AutoHomeIconNum > 8)
      {
        AutoHomeIconNum = 0;
      }
    }
    // moved z height output to make sure it is always up to date
    rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);

    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void SetExtruderMode(unsigned int mode, bool isDirect) {
  SERIAL_ECHOLNPGM("Selected extruder mode: ", mode);
  if (isDirect && mode == 4)
  {
    mode = 5; //DXC_SINGLE_R_MODE => SINGLE MODE 2
  }
  else if (isDirect && mode == 0)
  {
    mode = 4; //DXC_SINGLE_L_MODE => SINGLE MODE 1
  }
  SERIAL_ECHOLNPGM("Select new extruder mode: ", mode);
  if (mode == 1)
  {
    //dual mode
    rtscheck.RTS_SndData(1, TWO_COLOR_MODE_ICON_VP); //DUAL MODE icon blue
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);      //COPY MODE icon grey
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);    //MIRROR MODE icon grey
    rtscheck.RTS_SndData(0, SINGLE_MODE_ICON_VP);    //SINGLE MODE icon L grey + R grey
    dualXPrintingModeStatus = 1;
    rtscheck.RTS_SndData(1, PRINT_MODE_ICON_VP);     //DUAL MODE icon
    rtscheck.RTS_SndData(1, SELECT_MODE_ICON_VP);    //DUAL MODE icon
  }
  if (mode == 2)
  {
    //duplicate mode
    rtscheck.RTS_SndData(1, COPY_MODE_ICON_VP);      //COPY MODE icon blue
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP); //DUAL MODE icon grey
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);    //MIRROR MODE icon grey
    rtscheck.RTS_SndData(0, SINGLE_MODE_ICON_VP);    //SINGLE MODE icon L grey + R grey
    dualXPrintingModeStatus = 2;
    rtscheck.RTS_SndData(2, PRINT_MODE_ICON_VP);     //COPY MODE icon
    rtscheck.RTS_SndData(2, SELECT_MODE_ICON_VP);    //COPY MODE icon
  }
  if (mode == 3)
  {
    //mirror mode
    rtscheck.RTS_SndData(1, MIRROR_MODE_ICON_VP);    //MIRROR MODE icon blue
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP); //DUAL MODE icon grey
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);      //COPY MODE icon grey
    rtscheck.RTS_SndData(0, SINGLE_MODE_ICON_VP);    //SINGLE MODE icon L grey + R grey
    dualXPrintingModeStatus = 3;
    rtscheck.RTS_SndData(3, PRINT_MODE_ICON_VP);     //MIRROR MODE icon
    rtscheck.RTS_SndData(3, SELECT_MODE_ICON_VP);    //MIRROR MODE icon
  }
  if (mode == 4)
  {
    //single L mode
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);    //MIRROR MODE icon grey
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP); //DUAL MODE icon grey
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);      //COPY MODE icon grey
    rtscheck.RTS_SndData(1, SINGLE_MODE_ICON_VP);    //SINGLE MODE icon L blue + R grey
    dualXPrintingModeStatus = 0;
    rtscheck.RTS_SndData(4, PRINT_MODE_ICON_VP);     //SINGLE MODE L icon
    rtscheck.RTS_SndData(4, SELECT_MODE_ICON_VP);    //SINGLE MODE L icon
    queue.enqueue_now_P(PSTR("T0")); //select left extruder
  }
  if (mode == 5)
  {
    //single R mode
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);    //MIRROR MODE icon grey
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP); //DUAL MODE icon grey
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);      //COPY MODE icon grey
    rtscheck.RTS_SndData(2, SINGLE_MODE_ICON_VP);    //SINGLE MODE icon L grey + R blue
    dualXPrintingModeStatus = 4;
    rtscheck.RTS_SndData(5, PRINT_MODE_ICON_VP);     //SINGLE MODE R icon
    rtscheck.RTS_SndData(5, SELECT_MODE_ICON_VP);    //SINGLE MODE R icon
    queue.enqueue_now_P(PSTR("T1")); //select right extruder
  }
  if (mode == 6)
  {
    //return key; keep previous print mode
    save_dual_x_carriage_mode = dualXPrintingModeStatus;
    SERIAL_ECHOLNPGM("save_dual_x_carriage_mode: ", save_dual_x_carriage_mode);
    settings.save();
    RTS_currentScreen = 1;
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  }
  if ((mode < 1) || (mode > 6))
  {
    //single 1 mode
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);    //MIRROR MODE icon grey
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP); //DUAL MODE icon grey
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);      //COPY MODE icon grey
    rtscheck.RTS_SndData(1, SINGLE_MODE_ICON_VP);    //SINGLE MODE icon blue L + grey R
    dualXPrintingModeStatus = 0;
    rtscheck.RTS_SndData(4, PRINT_MODE_ICON_VP);     //SINGLE MODE L icon
    rtscheck.RTS_SndData(4, SELECT_MODE_ICON_VP);    //SINGLE MODE L icon
    queue.enqueue_now_P(PSTR("T0")); //select left extruder
  }
}

// looping at the loop function
void RTSUpdate()
{
  // Check the status of card
  rtscheck.RTS_SDCardUpate();

  sd_printing = IS_SD_PRINTING();
  card_insert_st = IS_SD_INSERTED();

  if((card_insert_st == false) && (sd_printing == true))
  {
    RTS_currentScreen = 1;
    rtscheck.RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
    rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
    /* Pause printing so that the nozzle can return to zero */
    card.pauseSDPrint();
    print_job_timer.pause();
    pause_action_flag = true;
    sdcard_pause_check = false;

  }

  /* Update the card removal and card prompt icons */
  if(last_card_insert_st != card_insert_st)
  {
    /* The current screen is displayed as a card removal prompt screen, but the card has already been inserted, update the card icon */
    rtscheck.RTS_SndData((int)card_insert_st, CHANGE_SDCARD_ICON_VP);
    last_card_insert_st = card_insert_st;
  }

  EachMomentUpdate();
  // wait to receive massage and response
  while(rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
}

void RTS_PauseMoveAxisPage()
{
  if (RTS_waitway == 1) //enable display pause processing
  {
    RTS_currentScreen = 12;
    rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
    RTS_waitway = 0;
  }
}

void RTS_MoveAxisHoming()
{
  if (RTS_waitway == 4) //moving
  {
    if (all_axes_trusted())
    {
      //Hide hazard warning
      rtscheck.RTS_SndData(0, COLLISION_HAZARD_ICON_VP);
    }
    else
    {
      //Show hazard warning
      rtscheck.RTS_SndData(1, COLLISION_HAZARD_ICON_VP);
    }
    if(AxisUnitMode == 4)
    {
      RTS_currentScreen = 58;
      rtscheck.RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
    }
    else if(AxisUnitMode == 3)
    {
      RTS_currentScreen = 29;
      rtscheck.RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
    }
    else if(AxisUnitMode == 2)
    {
      RTS_currentScreen = 30;
      rtscheck.RTS_SndData(ExchangePageBase + 30, ExchangepageAddr);
    }
    else if(AxisUnitMode == 1)
    {
      RTS_currentScreen = 31;
      rtscheck.RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
    }
    RTS_waitway = 0;
  }
  if (RTS_waitway == 6)
  {
    //leveling
    RTS_currentScreen = 22;
    rtscheck.RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
    RTS_waitway = 0;
  }
  if (RTS_waitway == 7)
  {
    // Click Print finish
    RTS_currentScreen = 1;
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    RTS_waitway = 0;
  }
  if(active_extruder == 0)
  {
    rtscheck.RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
  }
  else
  {
    rtscheck.RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
  }
  rtscheck.RTS_SndData(10*current_position[X_AXIS], AXIS_X_COORD_VP);
  rtscheck.RTS_SndData(10*current_position[Y_AXIS], AXIS_Y_COORD_VP);
  rtscheck.RTS_SndData(10*current_position[Z_AXIS], AXIS_Z_COORD_VP);
}

void RTS_Buzz(const uint16_t duration, const uint16_t frequency) //plays single beep tone on DWIN DGUS TL5 display onboard buzzer
{
  uint16_t foo = duration;
  foo =  foo + frequency; //unneeded operation just to supress compiler warning
  rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
}

void wait_idle(millis_t time) {
  time += millis();
  while (PENDING(millis(), time)) idle();
}

#endif
