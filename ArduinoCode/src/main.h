#pragma once

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <PressButton.h>
#include <RotaryEncoderAccel.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include "PID_Controller.h"
// FreeRTOS - must be first before any other FreeRTOS headers
#include <STM32FreeRTOS.h>

// FreeRTOS primitives you'll actually use
#include <task.h>       // xTaskCreate, vTaskDelay, vTaskDelayUntil
#include <semphr.h>     // SemaphoreCreateMutex, xSemaphoreTake/Give
#include <queue.h>      // xQueueCreate, xQueueSend, xQueueReceive

//--------------------------------------------------------------------------------------------------
// Stepper pins
//--------------------------------------------------------------------------------------------------
#define DIR_PIN_1  PA_4
#define STEP_PIN_1 PA_5
#define DIR_PIN_2  PA_6
#define STEP_PIN_2 PA_7
#define EN_PIN     PB_0

//--------------------------------------------------------------------------------------------------
// UART for TMC2209 (Full duplex)
//--------------------------------------------------------------------------------------------------
#define UART1_TX   PA_9
#define UART1_RX   PA_10
#define UART2_TX   PA_2
#define UART2_RX   PA_3

// I2C_1 pins - OLED
#define SDA_PIN_1  PB_9
#define SCL_PIN_1  PB_8

// SPI pins - OLED 2 (hardware SPI2)
#define OLED2_SDA_PIN_1   PB_9   // SPI2 MOSI
#define OLED2_SCL_PIN_1   PB_8   // SPI2 SCK

// I2C_2 pins - AS5600 1
#define SDA_PIN_2  PB_3
#define SCL_PIN_2  PB_10

// I2C_3 pins - AS5600 2
#define SDA_PIN_3  PB_4
#define SCL_PIN_3  PA_8

// Rotary encoder / button pins
#define confirmBtnPin PB_7
#define outputA       PB_6
#define outputB       PB_5

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
// Unit conversion
//--------------------------------------------------------------------------------------------------
constexpr float RPM_TO_DEG_S = 6.0f;   // 360° / 60s

//--------------------------------------------------------------------------------------------------
// Menu page type
//--------------------------------------------------------------------------------------------------
enum pageType {
    MENU_ROOT,
    MENU_AXIS_MODE,
    MENU_SELECTED_AXIS
};

//--------------------------------------------------------------------------------------------------
// Settings struct
//--------------------------------------------------------------------------------------------------
struct GlobalSettings {

    float timeBeforeDisable = 5.0f;
    uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

//--------------------------------------------------------------------------------------------------
// External objects (defined in main.cpp)
//--------------------------------------------------------------------------------------------------
extern TwoWire Wire1;
extern TwoWire Wire2;
extern RotaryEncoderAccel encoder;
extern PressButton btnOk;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C display1;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C display2;

//--------------------------------------------------------------------------------------------------
// External globals
//--------------------------------------------------------------------------------------------------
extern GlobalSettings settings;
extern GlobalSettings oldSettings;

extern float timeLastTouched;
extern bool  displaySleeping;
extern bool updateDisp2Flag;

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

//Debug loop Time
extern uint32_t loopCount;
extern uint32_t sumLoopTime;
extern uint32_t last;

//--------------------------------------------------------------------------------------------------
// Function declarations
//--------------------------------------------------------------------------------------------------

// Arduino entry points
void setup();
void loop();

// Menu pages
void page_MenuRoot();
void page_MenuMode();
void page_MENU_AXIS_MODE();
void page_MENU_SELECTED_AXIS();

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
void printFloatAtWidth(float value, uint8_t width, const char* c, uint8_t decimals);
void printDoubleAtWidth(double value, uint8_t width, const char* c, uint8_t decimals = 1);
void printStringAtWidth(const char* str, uint8_t width);
uint8_t getInt32_tCharCnt(int32_t value);
uint8_t getFloatCharCnt(float value);
uint8_t getDoubleCharCnt(double value);

// Display 2 tools
void updateDisp2();
void printCharsDisplay2(uint8_t cnt, char c);
void printInt32_tAtWidthDisplay2(int32_t value, uint8_t width, char c);
void printFloatAtWidthDisplay2(float value, uint8_t width, char c, uint8_t decimals);
void printDoubleAtWidthDisplay2(double value, uint8_t width, char c);

// Settings
void set_Default();
void sets_Load();
void sets_Save();
void updateSettings();

// Math helpers
float Filter(float New, float Current, float alpha, float maxValue);
float roundTo(float value, int decimals);

// Interrupt handler
void handleInterrupt();
