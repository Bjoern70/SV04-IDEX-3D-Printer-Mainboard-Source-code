#ifndef LCD_RTS_H
#define LCD_RTS_H

#include "../../../sd/cardreader.h"
#include "string.h"
#include <Arduino.h>

#include "../../../inc/MarlinConfig.h"

extern int power_off_type_yes;

/*********************************/
#define FHONE   (0x5A)
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     40
#define MaxFileNumber   20

#define FileNum             MaxFileNumber
#define FileNameLen         40
#define RTS_UPDATE_INTERVAL 2000
#define RTS_UPDATE_VALUE    (2 * RTS_UPDATE_INTERVAL)

#define SizeofDatabuf       46

/*************Register and Variable addr*****************/
#define RegAddr_W   0x80
#define RegAddr_R   0x81
#define VarAddr_W   0x82
#define VarAddr_R   0x83
#define ExchangePageBase    ((unsigned long)0x5A010000)
#define StartSoundSet       ((unsigned long)0x060480A0)
#define FONT_EEPROM         0

/*variable addr*/
#define ExchangepageAddr      0x0084
#define SoundAddr             0x00A0

#define START1_PROCESS_ICON_VP             0x1000 //#0
#define PRINT_SPEED_RATE_VP                0x1006 //#14
#define PRINT_PROCESS_ICON_VP              0x100E //#9,#10,#11,#12
#define PRINT_TIME_HOUR_VP                 0x1010 //#9,#10,#11,#12
#define PRINT_TIME_MIN_VP                  0x1012 //#9,#10,#11,#12
#define PRINT_PROCESS_VP                   0x1016 //#9,#10,#11,#12
#define HEAD0_FAN_ICON_VP                  0x101E //#14.#16
#define HEAD1_FAN_ICON_VP                  0x101F //#14.#17
#define CHANGE_FILAMENT0_TEMP_VP           0x1020 //#24
#define CHANGE_FILAMENT1_TEMP_VP           0x1022 //#25
#define AUTO_BED_LEVEL_ZOFFSET_VP          0x1026 //#14,#22

#define HEAD0_SET_TEMP_VP                  0x1034 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define HEAD0_CURRENT_TEMP_VP              0x1036 //#1,#7,#8,#9,#10,#11,#12,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define HEAD1_SET_TEMP_VP                  0x1038 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define HEAD1_CURRENT_TEMP_VP              0x1052 //#1,#7,#8,#9,#10,#11,#12,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define BED_SET_TEMP_VP                    0x103A //#1,#9,#10,#11,#12,#14,#15,#18,#52,#53,#54
#define BED_CURRENT_TEMP_VP                0x103C //#1,#9,#10,#11,#12,#15,#18,#52,#53,#54
#define AUTO_HOME_DISPLAY_ICON_VP          0x1042 //#32
#define AXIS_X_COORD_VP                    0x1048 //#29,#30,#31,#58
#define AXIS_Y_COORD_VP                    0x104A //#29,#30,#31,#58
#define AXIS_Z_COORD_VP                    0x104C //#9,#10,#11,#12,#29,#30,#31,#58
#define HEAD0_FILAMENT_LOAD_DATA_VP        0x1054 //#23,#24,#26,#27
#define HEAD1_FILAMENT_LOAD_DATA_VP        0x1058 //#23,#25,#26
#define PRINTER_MACHINE_TEXT_VP            0x1060 //20_#33
#define PRINTER_PRINTSIZE_TEXT_VP          0x1074 //20_#33

#define AUTO_BED_LEVEL_ICON_VP             0x108D //#38
#define CHANGE_FILAMENT_ICON_VP            0x108E //#7,#8
#define TWO_EXTRUDER_HOTEND_XOFFSET_VP     0x1092 //#35
#define TWO_EXTRUDER_HOTEND_YOFFSET_VP     0x1094 //#35

#define AUTO_TRAM_1TEXT_VP                 0x1096 //24_#57
//#define AUTO_TRAM_2TEXT_VP                 0x10AE //24_#57
//#define AUTO_TRAM_3TEXT_VP                 0x10C6 //24_#57
//#define AUTO_TRAM_4TEXT_VP                 0x10DE //24_#57

#define TWO_COLOR_MODE_ICON_VP             0x10F8 //#34
#define COPY_MODE_ICON_VP                  0x10F9 //#34
#define MIRROR_MODE_ICON_VP                0x10FA //#34
#define SINGLE_MODE_ICON_VP                0x10FB //#34
#define EXCHANGE_NOZZLE_ICON_VP            0x10FC //#28,#29,#30,#31,#58
#define PRINT_MODE_ICON_VP                 0x10FD //#9,#10,#11,#12

#define PRINTER_VERSION_TEXT_VP            0x10FE //20_#33
#define PRINTER_WEBSITE_TEXT_VP            0x1112 //20_#33
#define COLLISION_HAZARD_ICON_VP           0x1126 //#29,#30,#31,#58
//next free 0x1128 - 0x1161

#define PRINT_SURPLUS_TIME_HOUR_VP         0x1162 //#9,#10,#11,#12
#define PRINT_SURPLUS_TIME_MIN_VP          0x1164 //#9,#10,#11,#12
#define SELECT_MODE_ICON_VP                0x1166 //#1,#56
#define CHANGE_SDCARD_ICON_VP              0x1168 //#46,#57

//suicide enable/disable function
#define AUTO_POWER_OFF_ICON_VP             0x116A //#14
#define MOTOR_FREE_ICON_VP                 0x116C //#21
#define PAGE_STATUS_TEXT_VP                0x116E //7_#3
#define PRINTER_DISPLAY_VERSION_TEXT_VP    0x1176 //20_#0,#33
//next free 0x118A - 0x118F

//TEXT_LENGTH size has to match RTS_FileNameLength
#define PRINT_FILE_TEXT_VP                 0x1190 //40 #9,#10,#11,#12,#36,#56
#define SELECT_FILE_TEXT_VP                0x11B8 //40 #3
#define FILE1_TEXT_VP                      0x11E0 //40 #3
//#define FILE2_TEXT_VP                      0x1208 //40 #3
//#define FILE3_TEXT_VP                      0x1230 //40 #3
//#define FILE4_TEXT_VP                      0x1258 //40 #3
//#define FILE5_TEXT_VP                      0x1280 //40 #3
//next free 0x12A8 - 0x3FFF

#define AUTO_BED_LEVEL_1POINT_VP           0x4000 //#81
//[128] next free 0x4080

//Mesh visualization 0x4100 - 0x417F
#define MESH_VISUAL_ICON_VP                0x4100 //#81
//[128] next free 0x4180

#define HEAD0_SET_ICON_VP                  0x4200 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define HEAD0_CURRENT_ICON_VP              0x4202 //#1,#7,#8,#9,#10,#11,#12,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define HEAD1_SET_ICON_VP                  0x4204 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define HEAD1_CURRENT_ICON_VP              0x4206 //#1,#7,#8,#9,#10,#11,#12,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54
#define FilenameNature                     0x6003 //#3,#9,#10,#11,#12
#define	Beep                               ((unsigned long)0x02AF0100)
#define	Beep1                              ((unsigned long)0xFFFF0101)
/************struct**************/
typedef struct DataBuf
{
  unsigned char len;
  unsigned char head[2];
  unsigned char command;
  unsigned long addr;
  unsigned long bytelen;
  unsigned short data[32];
  unsigned char reserv[4];
} DB;

typedef struct
{
  char currentDir[MAXPATHNAMELENGTH];
  char currentFilePath[MAXPATHNAMELENGTH];
  uint16_t fileCount;
  uint16_t pages;
  int pageFileIndex;
  int currentPage;
  char currentDisplayFilename[40];
} FileInfo;

class RTSUI
{
  public:
  #if ENABLED(LCD_BED_LEVELING) && EITHER(PROBE_MANUALLY, MESH_BED_LEVELING)
    static bool wait_for_bl_move;
  #else
    static constexpr bool wait_for_bl_move = false;
  #endif
};

extern RTSUI rtsui;

class RTSSHOW
{
  public:
    RTSSHOW();
    int RTS_RecData();
    void RTS_SDCardInit(void);
    bool RTS_SD_Detected(void);
    void RTS_SDCardUpate(void);
    void RTS_SndData(void);
    void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SDcardStop();
    void RTS_SDcardFinish();
    void RTS_HandleData();
    void RTS_Init();
    void InitCardList();
    void ShowFilesOnCardPage(int);
    void RTS_ProcessPause();
    void RTS_ProcessResume();
    FileInfo fileInfo;
    DB recdat;
    DB snddat;
  private:
    unsigned char databuf[SizeofDatabuf];
};

extern RTSSHOW rtscheck;

enum PROC_COM
{
  MainPageKey = 0,
  AdjustmentKey,
  PrintSpeedKey,
  StopPrintKey,
  PausePrintKey,
  ResumePrintKey,
  ZOffsetKey,
  TempScreenKey,
  CoolScreenKey,
  Heater0TempEnterKey,
  Heater1TempEnterKey,
  HotBedTempEnterKey,
  SettingScreenKey,
  SettingBackKey,
  BedLevelFunKey,
  AxisPageSelectKey,
  XaxismoveKey,
  YaxismoveKey,
  ZaxismoveKey,
  SelectExtruderKey,
  Heater0LoadEnterKey,
  FilamentLoadKey,
  Heater1LoadEnterKey,
  SelectLanguageKey,
  FilamentCheckKey,
  PowerContinuePrintKey,
  PrintSelectModeKey,
  XhotendOffsetKey,
  YhotendOffsetKey,
  ZhotendOffsetKey,
  StoreMemoryKey,
  PrintFileKey,
  SelectFileKey,
  SaveEEPROM,
  SuicideKey,
  ChangePageKey
};

const unsigned long Addrbuf[] =
{
  0x1002,
  0x1004,
  0x1006,
  0x1008,
  0x100A,
  0x100C,
  0x1026,
  0x1030,
  0x1032,
  0x1034,
  0x1038,
  0x103A,
  0x103E,
  0x1040,
  0x1044,
  0x1046,
  0x1048,
  0x104A,
  0x104C,
  0x104E,
  0x1054,
  0x1056,
  0x1058,
  0x105C,
  0x105E,
  0x105F,
  0x1090,
  0x1092,
  0x1094,
  0x1096,
  0x1098,
  0x2198,
  0x2199,
  0x2202,
  0x2220,
  0x2300,
  0
};

/*
0x1002  MainPageKey,            #1,#9,#47
0x1004  AdjustmentKey,          #10,#11,#12,#14,#16,#17
0x1006  PrintSpeedKey,          #14
0x1008  StopPrintKey,           #8,10,#11,#12,#14,#39,#46
0x100A  PausePrintKey,          #11
0x100C  ResumePrintKey,         #8,#12,#39,#46
0x1026  ZOffsetKey,             #14,#22
0x1030  TempScreenKey,          #15
0x1032  CoolScreenKey,          #16,#17,#18
0x1034  Heater0TempEnterKey,    #14,#16
0x1038  Heater1TempEnterKey,    #14,#17
0x103A  HotBedTempEnterKey,     #14,#18
0x103E  SettingScreenKey,       #21
0x1040  SettingBackKey,         #22,#23,#28,#29,#30,#31,#33,#35,#57,#58,#81
0x1044  BedLevelFunKey,         #14,#22,#28,#57
0x1046  AxisPageSelectKey,      #29,#30,#31,#58
0x1048  XaxismoveKey,           #29,#30,#31,#58
0x104A  YaxismoveKey,           #29,#30,#31,#58
0x104C  ZaxismoveKey,           #29,#30,#31,#58
0x104E  SelectExtruderKey,      #28,#29,#30,#31,#58
0x1054  Heater0LoadEnterKey,    #23
0x1056  FilamentLoadKey,        #23,#24,#25,#26
0x1058  Heater1LoadEnterKey,    #23
0x105C  SelectLanguageKey,      UNUSED
0x105E  FilamentCheckKey,       #20
0x105F  PowerContinuePrintKey,  #36
0x1090  PrintSelectModeKey,     #34
0x1092  XhotendOffsetKey,       #35
0x1094  YhotendOffsetKey,       #35
0x1096  ZhotendOffsetKey,       UNUSED
0x1098  StoreMemoryKey,         #35
0x2198  PrintFileKey,           #3,#47,#56
0x2199  SelectFileKey,          #3
0x2202  SaveEEPROM,             #14,#22,#35
0x2220  SuicideKey,             #22
0x2300  ChangePageKey           #21
*/

extern int EndsWith(const char*, const char*);
extern void SetExtruderMode(unsigned int, bool);
extern void RTSUpdate();
extern void RTSInit();

extern uint8_t active_extruder_font;
extern uint8_t dualXPrintingModeStatus;
extern int Update_Time_Value;
extern bool PoweroffContinue;
extern bool sdcard_pause_check;

extern char save_dual_x_carriage_mode;
extern float current_position_x0_axis;
extern float current_position_x1_axis;

void RTS_PauseMoveAxisPage();
void RTS_AutoBedLevelPage();
void RTS_MoveAxisHoming();
extern void RTS_Buzz(const uint16_t, const uint16_t);
void wait_idle(millis_t);
#endif
