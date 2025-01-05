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

#define FileNameLen         132 //Allow longer filenames
//LONG_FILENAME_LENGTH = 65 + 1

#define NozzleWarningLimit  260.0 //260°C nozzle temperature limit for "sizzling" red background
#define AutoTuneNozzleLowerLimit  150.0 //150°C lower temperature limit for nozzle autotuning
#define AutoTuneHotBedLowerLimit  40.0  //40°C lower temperature limit for hot-bed autotuning
#define TuneCyclesLowerLimit  3  //Minumum number of autotuning cycles
#define TuneCyclesUpperLimit  16 //Maximum number of autotuning cycles
#define Head0TempLowerLimit  0.0   //Minumum temperature setting in left nozzle autotuning screen
#define Head0TempUpperLimit  300.0 //Maximum temperature setting in left nozzle autotuning screen
#define Head1TempLowerLimit  0.0   //Minumum temperature setting in right nozzle autotuning screen
#define Head1TempUpperLimit  300.0 //Maximum temperature setting in right nozzle autotuning screen
#define BedTempLowerLimit  0.0   //Minumum temperature setting in hot-bed autotuning screen
#define BedTempUpperLimit  110.0 //Maximum temperature setting in hot-bed autotuning screen

#define RTS_UPDATE_INTERVAL 2000
#define RTS_UPDATE_VALUE    (2 * RTS_UPDATE_INTERVAL)

#define SizeofDatabuf       138 //Allow longer filenames
//#define SizeofDatabuf        73 //LONG_FILENAME_LENGTH = 65 + 6

/*************Register and Variable addr*****************/
#define RegAddr_W   0x80
#define RegAddr_R   0x81
#define VarAddr_W   0x82
#define VarAddr_R   0x83
#define ExchangePageBase    ((unsigned long)0x5A010000)
#define StartSoundSet       ((unsigned long)0x060480A0)
#define	Beep                ((unsigned long)0x02AF0100)
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
#define CHANGE_FILAMENT0_TEMP_VP           0x1020 //#24
#define CHANGE_FILAMENT1_TEMP_VP           0x1022 //#25
#define AUTO_BED_LEVEL_ZOFFSET_VP          0x1026 //#14,#22
#define BED_SET_TEMP_VP                    0x102A //#1,#9,#10,#11,#12,#14,#15,#18,#52,#53,#54,#60,#61,#63,#64,#95,#103
#define HEAD0_SET_TEMP_VP                  0x1034 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54,#91,#93,#98,#101
#define HEAD0_CURRENT_TEMP_VP              0x1036 //#1,#7,#8,#9,#10,#11,#12,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54,#91,#93,#98,#101
#define HEAD1_SET_TEMP_VP                  0x1038 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54,#92,#94,#99,#102
#define BED_CURRENT_TEMP_VP                0x103C //#1,#9,#10,#11,#12,#15,#18,#52,#53,#54,#95,#103
#define AUTO_HOME_DISPLAY_ICON_VP          0x1042 //#32
#define AXIS_X_COORD_VP                    0x1048 //#29,#30,#31,#58
#define AXIS_Y_COORD_VP                    0x104A //#29,#30,#31,#58
#define AXIS_Z_COORD_VP                    0x104C //#9,#10,#11,#12,#29,#30,#31,#58
#define HEAD1_CURRENT_TEMP_VP              0x1052 //#1,#7,#8,#9,#10,#11,#12,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54,#92,#94,#99,#102
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
#define PRINT_MODE_ICON_VP                 0x10FD //#9,#10,#11,#12,#60,#62
#define PRINTER_VERSION_TEXT_VP            0x10FE //20_#33
#define PRINTER_WEBSITE_TEXT_VP            0x1112 //20_#33
#define COLLISION_HAZARD_ICON_VP           0x1126 //#29,#30,#31,#58

//evolved adjustment screen
#define HEAD0_FAN_SPEED_VP                 0x1128 //#14,#16
#define HEAD1_FAN_SPEED_VP                 0x112A //#14,#17
#define HEAD0_FLOW_RATE_VP                 0x112C //#14,#16
#define HEAD1_FLOW_RATE_VP                 0x112E //#14,#17

//PID tuning
#define TUNE_CURRENT_CYCLE_VP              0x1130 //#93,#94,#95
#define SET_TUNE_CYCLES_VP                 0x1132 //#93,#94,#95
#define TUNE_PROGRESS_ICON_VP              0x1134 //#93,#94,#95
#define HEAD0_TUNE_KP_VP                   0x1400 //Long! #93,#96,#101
#define HEAD0_TUNE_KI_VP                   0x1404 //Long! #93,#96,#101
#define HEAD0_TUNE_KD_VP                   0x1408 //Long! #93,#96,#101
#define HEAD1_TUNE_KP_VP                   0x140C //Long! #94,#96,#102
#define HEAD1_TUNE_KI_VP                   0x1410 //Long! #94,#96,#102
#define HEAD1_TUNE_KD_VP                   0x1414 //Long! #94,#96,#102
#define BED_TUNE_KP_VP                     0x1418 //Long! #95,#96.#103
#define BED_TUNE_KI_VP                     0x141C //Long! #95,#96.#103
#define BED_TUNE_KD_VP                     0x1420 //Long! #95,#96.#103
//next free 0x1424

//E-Steps
#define HEAD0_E_STEPS_VP                   0x1138 //Long! #91,#98
#define HEAD1_E_STEPS_VP                   0x113C //Long! #92,#99
#define E_STEPS_FEED_RATE_VP               0x1140 //#91,#92
#define E_STEPS_FEED_DIST_VP               0x1142 //#91,#92
#define BACKUP_MODE_ICON_VP                0x1144 //#34

#define HEAD0_SET_ICON_VP                  0x1150 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54,#91,#93
#define HEAD0_CURRENT_ICON_VP              0x1152 //#1,#7,#8,#9,#10,#11,#12,#15,#16,#23,#24,#25,#26,#27,#46,#52,#53,#54,#91,#93
#define HEAD1_SET_ICON_VP                  0x1154 //#1,#7,#8,#9,#10,#11,#12,#14,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54,#92,#94
#define HEAD1_CURRENT_ICON_VP              0x1156 //#1,#7,#8,#9,#10,#11,#12,#15,#17,#23,#24,#25,#26,#27,#46,#52,#53,#54,#92,#94

#define TUNE_TARGET_TEMP_VP                0x115A //#101, #102,#103

#define PRINT_SURPLUS_TIME_HOUR_VP         0x1162 //#9,#10,#11,#12
#define PRINT_SURPLUS_TIME_MIN_VP          0x1164 //#9,#10,#11,#12
#define SELECT_MODE_ICON_VP                0x1166 //#1,#56
#define CHANGE_SDCARD_ICON_VP              0x1168 //#46,#57

//suicide enable/disable function
#define AUTO_POWER_OFF_ICON_VP             0x116A //#14
#define MOTOR_FREE_ICON_VP                 0x116C //#21
#define PAGE_STATUS_TEXT_VP                0x116E //7_#3
#define PRINTER_DISPLAY_VERSION_TEXT_VP    0x1176 //20_#0,#33
//next free 0x118E - 0x118F

//Scrolling text pointer = XX_TEXT_VP + 0x0003
#define PRINT_FILE_TEXT_VP                 0x1503 //#9,#10,#11,#12,#36,#37,#56,#60,#62
#define PRESET_PLA_HOTEND_VP               0x1606 //#97
#define PRESET_PLA_BED_VP                  0x160A //#97
#define PRESET_PETG_HOTEND_VP              0x1608 //#97
#define PRESET_PETG_BED_VP                 0x160C //#97
#define PRESET_MOTOR_HOLD_TIME_VP          0x160E //#97
#define PRESET_AUTO_POWER_OFF_VP           0x1612 //#97

#define AUTO_BED_LEVEL_1POINT_VP           0x4000 //#81
//[128] next free 0x4080

//Mesh visualization 0x4100 - 0x417F
#define MESH_VISUAL_ICON_VP                0x4100 //#81
//[128] next free 0x4180

//scroll text SP begin                        0x4200
#define PRINT_FILE_TEXT_SP                 0x4201 //#9,#10,#11,#12,#36,#56
#define SELECT_DIR_TEXT_SP                 0x4221 //#2
#define FILE1_TEXT_SP                      0x4241 //#2
#define FILE2_TEXT_SP                      0x4261 //#2
#define FILE3_TEXT_SP                      0x4281 //#2
#define FILE4_TEXT_SP                      0x42A1 //#2
#define FILE5_TEXT_SP                      0x42C1 //#2
//Scroll text speed attribute
#define SCROLL_DIS_SP                      0x42E1 //#14
//next free 0x42F0-0x42FF


//scroll text begin                        0x4200
//Scroll text color attribute
#define FilenameNature                     0x4203 //#3,#9,#10,#11,#12
#define SELECT_DIR_TEXT_VP                 0x4303 //#2
#define FILE1_TEXT_VP                      0x4403 //#2
#define FILE2_TEXT_VP                      0x4503 //#2
#define FILE3_TEXT_VP                      0x4603 //#2
#define FILE4_TEXT_VP                      0x4703 //#2
#define FILE5_TEXT_VP                      0x4803 //#2
#define SCROLL_DIS_VP                      0x4903 //20_#14
//next free 0x4917



/************struct**************/
//
// RTS preset data
//
typedef struct
{
  celsius_t pla_hotend_t;     //nozzle temperature, type int16_t
  celsius_t pla_bed_t;        //hot-bed temperature, type int16_t
  celsius_t petg_hotend_t;    //nozzle temperature, type int16_t
  celsius_t petg_bed_t;       //hot-bed temperature, type int16_t
  int16_t motor_hold_time;    //DEFAULT_STEPPER_DEACTIVE_TIME in sec
  bool auto_power_off_enable; //auto power-off enabled, type boolean
} RTS_preset_data;

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
  char currentDisplayFilename[FileNameLen];
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
    void RTS_Restart();
    void RTS_HandleData();
    void RTS_Init();
    void InitCardList();
    void ShowFilesOnCardPage(int);
    void RTS_ProcessPause();
    void RTS_ProcessResume();
    void RTS_ProcessM24();
    void RTS_ProcessM25();
    void RTS_AutoBedLevelPage();
    void RTS_ViewMesh();
    FileInfo fileInfo;
    DB recdat;
    DB snddat;
    RTS_preset_data RTS_presets;
  private:
    unsigned char databuf[SizeofDatabuf];
};

extern RTSSHOW rtscheck;

/*
0x1002  MainPageKey,            #1,#9,#47,#60,#62
0x1004  AdjustmentKey,          #10,#11,#12,#14
0x1006  PrintSpeedKey,          #14
0x1008  StopPrintKey,           #8,10,#11,#12,#39,#46
0x100A  PausePrintKey,          #11
0x100C  ResumePrintKey,         #8,#12,#39,#46
0x102A  HotBedTempEnterKey,     #14,#18,#95
0x1030  TempScreenKey,          #15
0x1032  CoolScreenKey,          #16,#17,#18
0x1034  Heater0TempEnterKey,    #14,#16,#91,#93
0x1038  Heater1TempEnterKey,    #14,#17,#92,#94
0x103E  SettingScreenKey,       #21,#90
0x1040  SettingBackKey,         #22,#23,#28,#29,#30,#31,#33,#35,#57,#58,#81,#90,#91,#92,#93,#94,#95
0x1044  BedLevelFunKey,         #14,#22,#28,#57
0x1046  AxisPageSelectKey,      #29,#30,#31,#58
0x1048  XaxismoveKey,           #29,#30,#31,#58
0x104A  YaxismoveKey,           #29,#30,#31,#58
0x104C  ZaxismoveKey,           #29,#30,#31,#58
0x104E  SelectExtruderKey,      #28,#29,#30,#31,#58
0x1054  Heater0LoadEnterKey,    #23
0x1056  FilamentLoadKey,        #23,#24,#25,#26
0x1058  Heater1LoadEnterKey,    #23
0x105E  FilamentCheckKey,       #20
0x105F  PowerContinuePrintKey,  #36
0x1090  PrintSelectModeKey,     #34
0x1092  XhotendOffsetKey,       #35
0x1094  YhotendOffsetKey,       #35
0x1098  StoreMemoryKey,         #35
0x1128  Fan0SpeedEnterKey,      #14,#16
0x112A  Fan1SpeedEnterKey,      #14,#17
0x112C  Flow0EnterKey,          #14,#16
0x112E  Flow1EnterKey,          #14,#17
0x1132  TuneCyclesEnterKey,     #93,#94,#95
0x1136  EStepsAdjustKey,        #91,#92
0x1138  ESteps0EnterKey,        #91
0x113C  ESteps1EnterKey,        #92
0x1140  FeedRateEnterKey,       #91,#92
0x1142  FeedDistEnterKey,       #91,#92
0x1144  FeedKey,                #91,#92
0x114A  PrintFileKey,           #2,#3,#47,#56
0x114C  SelectFileKey,          #3
0x114E  SaveEEPROM,             #14,#22,#35,#91,#92,#93,#94,#95,#96,#97,#98
0x1158  AutoTuneEnterKey,       #93,#94,#95
0x115C  RestartKey,             #21
0x1160  SuicideKey,             #90
0x1400  Head0PIDKpEnterKey,     #96
0x1404  Head0PIDKiEnterKey,     #96
0x1408  Head0PIDKdEnterKey,     #96
0x140C  Head1PIDKpEnterKey,     #96
0x1410  Head1PIDKiEnterKey,     #96
0x1414  Head1PIDKdEnterKey,     #96
0x1418  BedPIDKpEnterKey,       #96
0x141C  BedPIDKiEnterKey,       #96
0x1420  BedPIDKdEnterKey,       #96
0x1606  PresetPlaHotendKey,     #97
0x1608  PresetPetgHotendKey,    #97
0x160A  PresetPlaBedKey,        #97
0x160C  PresetPetgBedKey,       #97
0x160E  PresetMotorHoldKey,     #97
0x1610  PresetAutoOffKey,       #97
0x2000  ChangePageKey           UNUSED
*/

enum PROC_COM
{
  MainPageKey = 0,
  AdjustmentKey,
  PrintSpeedKey,
  StopPrintKey,
  PausePrintKey,
  ResumePrintKey,
  HotBedTempEnterKey,
  TempScreenKey,
  CoolScreenKey,
  Heater0TempEnterKey,
  Heater1TempEnterKey,
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
  FilamentCheckKey,
  PowerContinuePrintKey,
  PrintSelectModeKey,
  XhotendOffsetKey,
  YhotendOffsetKey,
  StoreMemoryKey,
  Fan0SpeedEnterKey,
  Fan1SpeedEnterKey,
  Flow0EnterKey,
  Flow1EnterKey,
  TuneCyclesEnterKey,
  EStepsAdjustKey,
  ESteps0EnterKey,
  ESteps1EnterKey,
  FeedRateEnterKey,
  FeedDistEnterKey,
  FeedKey,
  PrintFileKey,
  SelectFileKey,
  SaveEEPROM,
  AutoTuneEnterKey,
  RestartKey,
  SuicideKey,
  Head0PIDKpEnterKey,
  Head0PIDKiEnterKey,
  Head0PIDKdEnterKey,
  Head1PIDKpEnterKey,
  Head1PIDKiEnterKey,
  Head1PIDKdEnterKey,
  BedPIDKpEnterKey,
  BedPIDKiEnterKey,
  BedPIDKdEnterKey,
  PresetPlaHotendKey,
  PresetPetgHotendKey,
  PresetPlaBedKey,
  PresetPetgBedKey,
  PresetMotorHoldKey,
  PresetAutoOffKey,
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
  0x102A,
  0x1030,
  0x1032,
  0x1034,
  0x1038,
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
  0x105E,
  0x105F,
  0x1090,
  0x1092,
  0x1094,
  0x1098,
  0x1128,
  0x112A,
  0x112C,
  0x112E,
  0x1132,
  0x1136,
  0x1138,
  0x113C,
  0x1140,
  0x1142,
  0x1144,
  0x114A,
  0x114C,
  0x114E,
  0x1158,
  0x115C,
  0x1160,
  0x1400,
  0x1404,
  0x1408,
  0x140C,
  0x1410,
  0x1414,
  0x1418,
  0x141C,
  0x1420,
  0x1606,
  0x1608,
  0x160A,
  0x160C,
  0x160E,
  0x1610,
  0x2000,
  0
};

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
extern char RTS_cyclesIcon;
extern uint8_t RTS_currentScreen;
extern uint8_t RTS_lastScreen;
extern char RTS_waitway;
extern char RTS_heatway;

void RTS_MoveAxisHoming();
extern void RTS_Buzz(const uint16_t, const uint16_t);
void wait_idle(millis_t);
#endif
