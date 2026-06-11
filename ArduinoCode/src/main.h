#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <PressButton.h>
#include <RotaryEncoderAccel.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include "PID_Controller.h"
#include <AS5600.h>
// FreeRTOS - must be first before any other FreeRTOS headers
#include <STM32FreeRTOS.h>

// FreeRTOS primitives you'll actually use
#include <task.h>       // xTaskCreate, vTaskDelay, vTaskDelayUntil
#include <semphr.h>     // SemaphoreCreateMutex, xSemaphoreTake/Give
#include <queue.h>      // xQueueCreate, xQueueSend, xQueueReceive

//--------------------------------------------------------------------------------------------------
// Stepper pins
//--------------------------------------------------------------------------------------------------
#define EN_PIN     PA6
#define DIR_PIN_1  PA0
#define STEP_PIN_1 PA1
#define DIR_PIN_2  PA2
#define STEP_PIN_2 PA3
#define MS1_PIN    PA15
#define MS2_PIN    PB3
#define MS3_PIN    PB4

// I2C pins - AS5600
#define SDA_PIN_1  PB11
#define SCL_PIN_1  PB10

// I2C pins - OLED
#define SDA_PIN_0  PB9
#define SCL_PIN_0  PB8

// Rotary encoder / button pins
#define confirmBtnPin PB7
#define outputA       PB6
#define outputB       PB5

//--------------------------------------------------------------------------------------------------
// Display
//--------------------------------------------------------------------------------------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

//--------------------------------------------------------------------------------------------------
// Settings
//--------------------------------------------------------------------------------------------------
#define FLASH_RST_CNT    3
#define SETTINGS_CHKVAL  3647

//--------------------------------------------------------------------------------------------------
// Magnetic encoder
//--------------------------------------------------------------------------------------------------
#define AS5600_ADDR      0x36
#define AS5600_RAW_ANGLE 0x0C

//--------------------------------------------------------------------------------------------------
// System mode
//--------------------------------------------------------------------------------------------------
enum SystemMode {
    MODE_OSCILATION,
    MODE_SETPOINT,
    MODE_RPM
};

//--------------------------------------------------------------------------------------------------
// Menu page type
//--------------------------------------------------------------------------------------------------
enum pageType {
    MENU_ROOT,
    MENU_SYSTEM_MODE
};

//--------------------------------------------------------------------------------------------------
// Settings struct
//--------------------------------------------------------------------------------------------------
struct Mysettings {
    float setPoint          = 10.0f;
    float Kp                = 50.0f;
    float Ki                = 0.0f;
    float Kd                = 2.0f;
    float amplitude         = 20.0f;
    float period            = 2.0f;
    float timeBeforeDisable = 2.0f;
    SystemMode systemMode   = MODE_OSCILATION;
    uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

//--------------------------------------------------------------------------------------------------
// External objects (defined in main.cpp)
//--------------------------------------------------------------------------------------------------
extern TwoWire Wire1;
extern RotaryEncoderAccel encoder;
extern PressButton btnOk;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C display1;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C display2;
extern PID_Controller pid;
extern HardwareTimer *stepTimer;

//--------------------------------------------------------------------------------------------------
// External globals
//--------------------------------------------------------------------------------------------------
extern Mysettings settings;
extern Mysettings oldSettings;

extern volatile uint32_t stepFrequency;

extern float angle;
extern float stepVelocity;
extern float absoluteAngle;
extern float rpm_measured;
extern float passedTime;
extern float timeLastTouched;
extern bool  displaySleeping;

extern uint8_t microstepping;
extern float   steps_per_rev;

// Display layout constants
extern uint8_t DISP_ITEM_ROWS;
extern uint8_t DISP_CHAR_WIDTH;
extern uint8_t CHAR_X;
extern uint8_t CHAR_Y;
extern uint8_t DISP2_ITEM_ROWS;
extern uint8_t DISP2_CHAR_WIDTH;
extern uint8_t DISP2_CHAR_X;
extern uint8_t DISP2_CHAR_Y;

// Menu state
extern enum pageType currPage;
extern bool    updateAllItems;
extern bool    updateItemValue;
extern uint8_t itemCnt;
extern int8_t  cursorPos;
extern uint8_t saveCursorPos;
extern uint8_t dispOffset;
extern uint8_t saveDispOffset;
extern bool    edditing;
extern uint8_t root_pntrPos;
extern uint8_t root_dispOffset;
extern uint8_t flashCntr;
extern bool    flashIsOn;
extern bool    initPage;
extern bool    changeValues[10];

// Timing
extern float    previousPassedTime1;
extern float    previousPassedTime2;
extern uint32_t now;
extern uint32_t loopTime;
extern uint32_t avgLoopTime;

// Sensor
extern unsigned long lastReadTime;

// Last-known settings (for change detection)
extern float lastSetPoint;
extern float lastKp;
extern float lastKi;
extern float lastKd;
extern float lastAmplitude;
extern float lastPeriod;

//--------------------------------------------------------------------------------------------------
// Function declarations
//--------------------------------------------------------------------------------------------------

// Arduino entry points
void setup();
void loop();

// Menu pages
void page_MenuRoot();
void page_MenuMode();
void page_MENU_SYSTEM_MODE();

// Menu internals
void initMenuPage(String title, uint8_t itemCount);
void doPointerNavigation();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);
void menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos);
bool isFlashChanged();

void incrementDecrementInt(int16_t *v, int16_t amount, int16_t min, int16_t max);
void incrementDecrementFloat(float *v, float amount, float min, float max);
void incrementDecrementDouble(double *v, double amount, double min, double max);

// Print / display tools
void printPointer();
void FlashPointer();
void printOnOff(bool val);
void printChars(uint8_t cnt, char c);
void printInt32_tAtWidth(int32_t value, uint8_t width, const char* c);
void printDoubleAtWidth(double value, uint8_t width, const char* c, uint8_t decimals = 1);
void printStringAtWidth(const char* str, uint8_t width);
uint8_t getInt32_tCharCnt(int32_t value);
uint8_t getDoubleCharCnt(double value);

// Display 2 tools
void updateDisp2();
void printCharsDisplay2(uint8_t cnt, char c);
void printInt32_tAtWidthDisplay2(int32_t value, uint8_t width, char c);
void printDoubleAtWidthDisplay2(double value, uint8_t width, char c);

// Settings
void set_Default();
void sets_Load();
void sets_Save();
void updateSettings();

// Sensors
void updateSensorValues();
void initAS5600();
inline uint16_t readAS5600RawFast();

// Stepper / motor
void stepISR();
void initStepTimer();
void setStepFrequency(uint32_t steps_per_sec);
void setDirection(int velocity);
void setAngle();
void setMicrostepTMC2209(uint16_t microstep);
void setMicrostepA4988(uint8_t microstep);
void updateMicrostepCycle();
void microstepTest();

// Mode
void updateMode();
const char* systemModeToString(SystemMode mode);

// Math helpers
float Filter(float New, float Current, float alpha, float maxValue);
float roundTo(float value, int decimals);

// Interrupt handler
void handleInterrupt();
