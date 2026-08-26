#include "main.h"
#include "Axis/axis.h"
#include "taskMain.h"
#include "taskControl.h"
#include "taskDisplay.h"
#include "TMC2209/tmc2209.h"
//--------------------------------------------------------------------------------------------------
// Object definitions
//--------------------------------------------------------------------------------------------------
TwoWire Wire1(1);  // I2C2
TwoWire Wire2(2);  // I2C3

RotaryEncoderAccel encoder(outputA, outputB);  // GPIO32 och GPIO33 på ESP32
PressButton btnOk(confirmBtnPin, 10); //debounce (ISR-attached inside library)

U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//--------------------------------------------------------------------------------------------------
// Global variable definitions
//--------------------------------------------------------------------------------------------------
GlobalSettings settings;
GlobalSettings oldSettings;

// Display layout
uint8_t DISP_ITEM_ROWS  = 0;
uint8_t DISP_CHAR_WIDTH = 0;
uint8_t CHAR_X          = 0;
uint8_t CHAR_Y          = 0;
uint8_t DISP2_ITEM_ROWS  = 0;
uint8_t DISP2_CHAR_WIDTH = 0;
uint8_t DISP2_CHAR_X     = 0;
uint8_t DISP2_CHAR_Y     = 0;

// Menu state
enum pageType currPage   = MENU_ROOT;
bool    updateAllItems   = true;
bool    updateItemValue  = false;
bool    updateDisp2Flag = false;
uint8_t itemCnt          = 0;
int8_t  cursorPos        = 0;
uint8_t saveCursorPos    = 0;
uint8_t dispOffset       = 0;
uint8_t saveDispOffset   = 0;
bool    edditing         = false;
uint8_t root_pntrPos     = 0;
uint8_t root_dispOffset  = 0;
uint8_t flashCntr        = 0;
bool    flashIsOn        = false;
bool    initPage         = true;
bool    changeValues[10] = {};

// Timing
float previousPassedTime1 = 0.0f;
float previousPassedTime2 = 0.0f;
float timeLastTouched     = 0.0f;
bool  displaySleeping     = false;

// Loop time measurement variables
uint32_t last = 0;     //staic removed
uint32_t now         = 0;
uint32_t loopTime    = 0;
uint32_t sumLoopTime = 0;
uint32_t loopCount   = 0;
uint32_t avgLoopTime = 0;

//sensors
unsigned long lastReadTime  = 0;

//==================================================================================================
// ISR för båda pins, som anropas vid ändring (rising/falling)
//==================================================================================================
void handleInterrupt()
{
    encoder.tick();
}

//==================================================================================================
//Setup
//==================================================================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);
    // while(!Serial);
    Serial.println("Boot");

    

    // 1. Pin modes first
    pinMode(EN_PIN,     OUTPUT);
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_1,  OUTPUT);
    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(DIR_PIN_2,  OUTPUT);

    // pinMode(SDA_PIN_0, INPUT_PULLUP);
    // pinMode(SCL_PIN_0, INPUT_PULLUP);

    pinMode(confirmBtnPin, INPUT_PULLUP);
    pinMode(outputA,       INPUT_PULLUP);
    pinMode(outputB,       INPUT_PULLUP);

    // setMicrostepA4988(1);
    // microstepTest();
    initStepTimer(axis1, TIM2, stepISR1);
    initStepTimer(axis2, TIM3, stepISR2);
    
    // 2. Enable all stepper drivers
    digitalWrite(EN_PIN, LOW); 
    delay(100); // Allow time for drivers to enable
    
    // 3. TMC2209
    // setMicrostepTMC2209(microstepping);
    initTMC2209();

    initAxisSettings(axis1, MODE_velControl, -10.0f, 5.0f, 0.0f, 0.0f, 90.0f, 4.0f);
    initAxisSettings(axis2, MODE_posControl_OSC, 10.0f, 25.0f, 0.0f, 0.0f, 360.0f, 7.0f);

    axis1.posProfile.type = TrapProfile::Type::POSITION;
    axis1.velProfile.type = TrapProfile::Type::VELOCITY;

    axis2.posProfile.type = TrapProfile::Type::POSITION;
    axis2.velProfile.type = TrapProfile::Type::VELOCITY;

    updatePIDTunings(axis1, posPid1);
    updatePIDTunings(axis1, velPid1);
    updatePIDTunings(axis2, posPid2);
    updatePIDTunings(axis2, velPid2);

    // 4. Init PID controllers
    initPID(posPid1, axis1);
    initPID(velPid1, axis1);
    initPID(posPid2, axis2);
    initPID(velPid2, axis2);       

    // Initialize encoder interrupts (confirm button interrupt disabled)
    attachInterrupt(digitalPinToInterrupt(outputA), handleInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(outputB), handleInterrupt, CHANGE);

    // Initialize button ISR
    btnOk.init();

    // -------- I2C - Oled display1 and display2--------
    Wire.setSDA(SDA_PIN_1);
    Wire.setSCL(SCL_PIN_1);
    Wire.begin();
    Wire.setClock(400000);

    // -------- I2C - AS5600--------
    Wire1.setSDA(SDA_PIN_2);
    Wire1.setSCL(SCL_PIN_2);
    Wire1.begin();
    Wire1.setClock(400000);
    sensor1.begin();
    initSensor(axis1);
    
    // -------- I2C - AS5600--------
    Wire2.setSDA(SDA_PIN_3);
    Wire2.setSCL(SCL_PIN_3);
    Wire2.begin();
    Wire2.setClock(400000);
    sensor2.begin();
    initSensor(axis2);
    sensor2.setDirection(AS5600_COUNTERCLOCK_WISE);

    Serial.println("Scanning Wire...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);
        }
    }
    Serial.println("Wire done");

    Serial.println("Scanning Wire1...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        uint8_t error = Wire1.endTransmission();
        if (error == 0) {
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);
        }
    }
    Serial.println("Wire1 done");

    Serial.println("Scanning Wire2...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire2.beginTransmission(addr);
        uint8_t error = Wire2.endTransmission();
        if (error == 0) {
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);
        }
    }
    Serial.println("Wire2 done");

    sets_Load();

    // -------- I2C - Display1--------
    display1.setI2CAddress(0x3C << 1);
    display1.begin();
    display1.setBusClock(400000);
    display1.setFont(u8g2_font_5x8_mr);
    display1.clearBuffer();
    display1.setCursor(0, 0);

    DISP_CHAR_WIDTH = SCREEN_WIDTH  / (display1.getMaxCharWidth()  + 0);          // hur många tecken per rad: display1.getMaxCharWidth();
    DISP_ITEM_ROWS  = SCREEN_HEIGHT / (display1.getMaxCharHeight() + 3);          // max rader som får plats
    CHAR_X          = display1.getMaxCharWidth()  + 2; //margin
    CHAR_Y          = display1.getMaxCharHeight() + 2; //margin

    // -------- I2C - Display2--------
    display2.setI2CAddress(0x3D << 1);
    display2.begin();
    display2.setBusClock(400000);
    display2.setFont(u8g2_font_5x8_mr);
    display2.clearBuffer();
    display2.setCursor(0, 0);

    DISP2_CHAR_WIDTH = SCREEN_WIDTH / (display2.getMaxCharWidth() + 2);          // hur många tecken per rad: display2.getMaxCharWidth();
    DISP2_ITEM_ROWS  = 5;
    DISP2_CHAR_X     = display2.getMaxCharWidth() + 0; //margin
    DISP2_CHAR_Y     = (SCREEN_HEIGHT / DISP2_ITEM_ROWS) + 0; //margin

    last = micros();

    selectedAxis->lastSetPoint  = selectedAxis->settings.targetSetPoint;
    selectedAxis->lastKp        = selectedAxis->settings.Kp;
    selectedAxis->lastKi        = selectedAxis->settings.Ki;
    selectedAxis->lastKd        = selectedAxis->settings.Kd;
    selectedAxis->lastAmplitude = selectedAxis->settings.amplitude;
    selectedAxis->lastPeriod    = selectedAxis->settings.period;

    // Create main task
    xTaskCreate(taskMain,    "Main",    512,  nullptr, 2, nullptr);
    // Create control task
    xTaskCreate(taskControl, "Control", 512,  nullptr, 3, &taskControlHandle);
    // Create display task
    xTaskCreate(taskDisplay, "Display", 1024, nullptr, 1, &taskDisplayHandle);

    // Start the scheduler
    vTaskStartScheduler();
}

void loop(){} // not used

void page_MenuRoot() //=================================================ROOT_MENU============================================
{
    if (initPage) {
        cursorPos  = root_pntrPos;
        dispOffset = root_dispOffset;
        memset(changeValues, 0, sizeof(changeValues));
        initMenuPage(F("MAIN MENU"), 8);
        initPage = false;
    }

    if (btnOk.Pressed()) {
        FlashPointer();
        changeValues[cursorPos] = !changeValues[cursorPos];
        edditing = !edditing;
    }
         if (changeValues[0]) { currPage = MENU_SELECTED_AXIS; initPage = true; updateAllItems = true; edditing = false; return; }
    else if (changeValues[1]) { incrementDecrementFloat(&selectedAxis->settings.targetSetPoint, 10.0, -20000.0, 20000.0); }
    else if (changeValues[2])   incrementDecrementFloat(&selectedAxis->settings.Kp,       0.25,  0.0, 50.0);
    else if (changeValues[3])   incrementDecrementFloat(&selectedAxis->settings.Ki,       0.05,  0.0,  20.0);
    else if (changeValues[4])   incrementDecrementFloat(&selectedAxis->settings.Kd,       0.05,  0.0, 10.0);
    else if (changeValues[5]) { currPage = MENU_AXIS_MODE; initPage = true; updateAllItems = true; edditing = false; return; }
    else if (changeValues[6])   incrementDecrementFloat(&selectedAxis->settings.amplitude, 5.0, 0.0, 1440.0);
    else if (changeValues[7])   incrementDecrementFloat(&selectedAxis->settings.period,    0.1, 0.0,    60.0);
    else
        doPointerNavigation();

    if (!(updateAllItems | updateItemValue)) return;

    for (uint8_t i = 1; i <= 8; i++) {
        if (menuItemPrintable(1, i)) {
            switch (i) {
                case 1: display1.print(F("Selected:             ")); break;
                case 2: display1.print(F("Setpoint:             ")); break;
                case 3: display1.print(F("Kp:                   ")); break;
                case 4: display1.print(F("Ki:                   ")); break;
                case 5: display1.print(F("Kd:                   ")); break;
                case 6: display1.print(F("Mode:                 ")); break;
                case 7: display1.print(F("Amplitude:            ")); break;
                case 8: display1.print(F("Period:               ")); break;
            }
        }
        if (menuItemPrintable(10, i)) {
            switch (i) {
                case 1: printStringAtWidth(axisIdToString(selectedAxis->axisId), 4);       break;
                case 2: printDoubleAtWidth(selectedAxis->settings.targetSetPoint,  3, " ");      break;
                case 3: printDoubleAtWidth(selectedAxis->settings.Kp,        4, " ");      break;
                case 4: printDoubleAtWidth(selectedAxis->settings.Ki,        4, " ", 2);   break;
                case 5: printDoubleAtWidth(selectedAxis->settings.Kd,        4, " ", 2);   break;
                case 6: printStringAtWidth(axisModeToString(selectedAxis->settings.axisMode), 4); break;
                case 7: printDoubleAtWidth(selectedAxis->settings.amplitude, 4, " ", 2);   break;
                case 8: printDoubleAtWidth(selectedAxis->settings.period,    4, " ", 2);   break;
            }
        }
    }
}

void page_MENU_AXIS_MODE() //=================================================AXIS_MODE============================================
{
    if (initPage) {
        cursorPos  = 0;
        dispOffset = 0;
        memset(changeValues, 0, sizeof(changeValues));
        initMenuPage(F("AXIS CONTROL MODE"), 5);
        initPage = false;
    }

    if (btnOk.Pressed()) {
        FlashPointer();
        changeValues[cursorPos] = true;
    }

         if (changeValues[0]) { selectedAxis->settings.axisMode = MODE_posControl;     currPage = MENU_ROOT; initPage = true; return; }
    else if (changeValues[1]) { selectedAxis->settings.axisMode = MODE_posControl_OSC; currPage = MENU_ROOT; initPage = true; return; }
    else if (changeValues[2]) { selectedAxis->settings.axisMode = MODE_velControl;     currPage = MENU_ROOT; initPage = true; return; }
    else if (changeValues[3]) { selectedAxis->settings.axisMode = MODE_velControl_OSC; currPage = MENU_ROOT; initPage = true; return; }
    else if (changeValues[4]) {                                                        currPage = MENU_ROOT; initPage = true; return; }
    else
        doPointerNavigation();

    if (!(updateAllItems | updateItemValue)) return;

    for (uint8_t i = 1; i <= 5; i++) {
        if (menuItemPrintable(1, i)) {
            switch (i) {
                case 1: display1.print(F("Pos Control     ")); break;
                case 2: display1.print(F("Pos Control_OSC ")); break;
                case 3: display1.print(F("Vel Control     ")); break;
                case 4: display1.print(F("Vel Control_OSC ")); break;
                case 5: display1.print(F("Back           ")); break;
            }
        }
    }
}

void page_MENU_SELECTED_AXIS() //=================================================SELECTED_AXIS============================================
{
    if (initPage) {
        cursorPos  = 0;
        dispOffset = 0;
        memset(changeValues, 0, sizeof(changeValues));
        initMenuPage(F("SELECTED AXIS"), 3);
        initPage = false;
    }

    if (btnOk.Pressed()) {
        FlashPointer();
        changeValues[cursorPos] = true;
    }

         if (changeValues[0]) { selectedAxis = &axis1; currPage = MENU_ROOT; initPage = true; return; }
    else if (changeValues[1]) { selectedAxis = &axis2; currPage = MENU_ROOT; initPage = true; return; }
    else if (changeValues[2]) {                        currPage = MENU_ROOT; initPage = true; return; }
    else
        doPointerNavigation();

    if (!(updateAllItems | updateItemValue)) return;

    for (uint8_t i = 1; i <= 3; i++) {
        if (menuItemPrintable(1, i)) {
            switch (i) {
                case 1: display1.print(F("AXIS 1    ")); break;
                case 2: display1.print(F("AXIS 2    ")); break;
                case 3: display1.print(F("Back      ")); break;
            }
        }
    }
}

//======================================================TOOLS - menu Internals==================================================
void initMenuPage(String title, uint8_t itemCount)
{
    display1.clearBuffer();
    printPointer();
    uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;

    itemCnt   = itemCount;
    flashCntr = 0;
    flashIsOn = false;
    updateAllItems = true;
}

void doPointerNavigation()
{
    int direction = round(encoder.getRPM());

    if (direction != 0) {
        int newCursorPos = cursorPos + direction;

        // Clamp
        if (newCursorPos < 0)           newCursorPos = 0;
        if (newCursorPos > itemCnt - 1) newCursorPos = itemCnt - 1;

        if (newCursorPos != cursorPos) {
            cursorPos = newCursorPos;

            // Scroll logic
            if (cursorPos < dispOffset) {
                dispOffset = cursorPos;
                // scroll caused, already forcing update
                updateAllItems = true;
            } else if (cursorPos >= dispOffset + DISP_ITEM_ROWS) {
                dispOffset = cursorPos - DISP_ITEM_ROWS + 1;
                // scroll caused, already forcing update
                updateAllItems = true;
            }

            printPointer();  // Only redraw when view actually changes
        }

        timeLastTouched = millis() / 60000.0f; // Update last touched time (encoder rotation)

        // Serial.print("Direction: ");
        // Serial.print(direction);
        // Serial.print(",\t\tNew Cursor Position: ");
        // Serial.print(newCursorPos);
        // Serial.print(",\t\tDisplay Offset: ");
        // Serial.print(dispOffset);
        // Serial.print(", updateAllItems: ");
        // Serial.println(updateAllItems);
    }
}

void incrementDecrementInt(int16_t *v, int16_t amount, int16_t min, int16_t max)
{
    int16_t direction = encoder.getRPM();

    if (direction != 0) {
        int16_t target   = direction * amount;
        int16_t newValue = *v + target;

        if (newValue >= min && newValue <= max) {
            *v = newValue;
        } else if (newValue < min) {
            *v = min;
        } else {
            *v = max;
        }

        updateItemValue = true;
        timeLastTouched = millis() / 60000.0f;
    }

    delayMicroseconds(5);
}

void incrementDecrementFloat(float *v, float amount, float min, float max)
{
    float direction = encoder.getRPM();

    if (direction != 0) {
        float target   = 0;
        float newValue = 0;
        int   threshold = 4;

        if (abs(direction) <= threshold) //slow rotation
        {
            target   = direction * amount;
            newValue = *v + target;
        }
        else if (abs(direction) > threshold) //fast rotation
        {
            target   = direction * amount;
            newValue = *v + target;
            newValue = roundTo(newValue, -log10(amount));
        }

        if      (newValue >= min && newValue <= max) *v = newValue;
        else if (newValue < min)                     *v = min;
        else                                         *v = max;

        // Serial.print(", Direction: ");
        // Serial.print(direction);
        // Serial.print(",\tNew Value: ");
        // Serial.println(newValue);

        // updateItemValue = true;
        timeLastTouched = millis() / 60000.0f;
    }

    delayMicroseconds(5);
}

void incrementDecrementDouble(double *v, double amount, double min, double max)
{
    double direction = encoder.getRPM();

    if (direction != 0) {
        double target   = 0;
        double newValue = 0;
        int    threshold = 4;

        if (abs(direction) <= threshold) //slow rotation
        {
            target   = direction * amount;
            newValue = *v + target;
        }
        else if (abs(direction) > threshold) //fast rotation
        {
            target   = direction * amount;
            newValue = *v + target;
            newValue = roundTo(newValue, -log10(amount));
        }

        if      (newValue >= min && newValue <= max) *v = newValue;
        else if (newValue < min)                     *v = min;
        else                                         *v = max;

        timeLastTouched = millis() / 60000.0f;
    }

    delayMicroseconds(5);
}

bool isFlashChanged()
{
    if (flashCntr == 0) {
        flashIsOn = !flashIsOn;
        flashCntr = FLASH_RST_CNT;
        return true;
    }
    else { flashCntr--; return false; }
}

bool menuItemPrintable(uint8_t xPos, uint8_t yPos)
{
    if (!(updateAllItems || (updateItemValue && cursorPos == yPos))) { return false; }
    uint8_t yMaxOffset = 0;
    if (yPos > DISP_ITEM_ROWS) { yMaxOffset = yPos - DISP_ITEM_ROWS; }
    if (dispOffset <= (yPos) && dispOffset >= yMaxOffset) { display1.setCursor(CHAR_X * xPos, CHAR_Y * (yPos - dispOffset)); return true; }
    return false;
}

void menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos)
{
    display2.setCursor(DISP2_CHAR_X * xPos, DISP2_CHAR_Y * (yPos - 0));
}

//======================================================TOOLS_display========================================================
void printPointer(){
  //Serial.println("printPointer");
  for(uint8_t i=1; i<=DISP_ITEM_ROWS; i++) display1.drawStr(0, i*CHAR_Y, " ");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  updateAllItems = true;
  // display1.sendBuffer();
}
void FlashPointer(){
  timeLastTouched = millis()/1000.0/60.0;
  for(uint8_t i=1; i<=DISP_ITEM_ROWS; i++) display1.drawStr(0, i*CHAR_Y, " ");
  updateAllItems = true;

  delay(100);
  //Serial.println("FlashPointer");
  display1.drawStr(0, (cursorPos - dispOffset + 1)*CHAR_Y, "*");
  updateAllItems = true;
}

void printOnOff(bool val)
{
    if (val) { display1.print(F("ON    ")); }
    else     { display1.print(F("OFF   ")); }
}

void printChars(uint8_t cnt, char c)
{
    if (cnt > 0) {
        char cc[] = " "; cc[0] = c;
        for (uint8_t i = 1; i < cnt; i++) { display1.print(cc); }
    }
}

uint8_t getInt32_tCharCnt(int32_t value)
{
    if (value == 0) { return 1; }
    int32_t tensCalc = 10; int8_t cnt = 1;
    while (tensCalc <= value && cnt < 20) { tensCalc *= 10; cnt += 1; }
    return cnt;
}

uint8_t getFloatCharCnt(float value)
{
    if (value == 0) { return 1; }
    uint32_t tensCalc = 10; uint8_t cnt = 1;
    while (tensCalc <= value && cnt < 20) { tensCalc *= 10; cnt += 1; }
    return cnt;
}

uint8_t getDoubleCharCnt(double value)
{
    if (value == 0) { return 1; }
    uint32_t tensCalc = 10; uint8_t cnt = 1;
    while (tensCalc <= value && cnt < 20) { tensCalc *= 10; cnt += 1; }
    return cnt;
}

void printInt32_tAtWidth(int32_t value, uint8_t width, const char* c)
{
    display1.print(value);
    display1.print(c);
    printChars(width - getInt32_tCharCnt(value), ' ');
}

void printFloatAtWidth(float value, uint8_t width, const char* c, uint8_t decimals)
{
    char buf[10];
    dtostrf(value, width - getFloatCharCnt(value), decimals, buf); // 1 decimal
    display1.print(buf);
    display1.print(c);
}

void printDoubleAtWidth(double value, uint8_t width, const char* c, uint8_t decimals)
{
    char buf[10];
    dtostrf(value, width - getDoubleCharCnt(value), decimals, buf); // 1 decimal
    display1.print(buf);
    display1.print(c);
}

void printStringAtWidth(const char* str, uint8_t width)
{
    uint8_t len = strlen(str);
    display1.print(str);
    if (len < width) {
        for (uint8_t i = 0; i < (width - len); i++)
            display1.print(' ');
    }
}

const char* axisIdToString(AxisId axisId)
{
    switch (axisId) {
        case AXIS_1: return "AXIS 1";
        case AXIS_2: return "AXIS 2";
        default:     return "UNKNOWN";
    }
}

const char* axisModeToString(AxisMode mode)
{
    switch (mode) {
        case MODE_posControl_OSC: return "posControl_OSC";
        case MODE_posControl:     return "posControl";
        case MODE_velControl:     return "velControl";
        case MODE_velControl_OSC: return "velControl_OSC";
        default:                  return "UNKNOWN";
    }
}

//======================================================DISPLAY_2======================================================
void updateDisp2()
{
    // display2.clearBuffer();
    // display2.setCursor(0, 10);
    // display2.print("AbsoluteAngle: ");
    // display2.print(axis1.absoluteAngle);
    // display2.setCursor(0, 22);
    // display2.print("RPM: ");
    // display2.print((int32_t)axis1.rpm);
    // display2.sendBuffer();
    // display2.clearBuffer();

    // Serial.println("updateDisp2");
    display2.clearBuffer();

    menuItemPrintableDisp2(1,  1); display2.print(F("Angle_Axis1: "));
    menuItemPrintableDisp2(17, 1); printFloatAtWidthDisplay2(axis1.absPos, 3, ' ', 1);

    menuItemPrintableDisp2(1,  2); display2.print(F("Deg/s_Axis1: "));
    menuItemPrintableDisp2(17, 2); printFloatAtWidthDisplay2(axis1.vel, 3, ' ', 1);

    menuItemPrintableDisp2(1,3); display2.print(F("Angle_Axis2: "));
    menuItemPrintableDisp2(17,3); printFloatAtWidthDisplay2(axis2.absPos, 3, ' ', 1);

    menuItemPrintableDisp2(1,4); display2.print(F("Deg/s_Axis2:             "));
    menuItemPrintableDisp2(17,4); printFloatAtWidthDisplay2(axis2.vel, 3, ' ', 1);

    display2.sendBuffer();
    display2.clearBuffer();
}

void printCharsDisplay2(uint8_t cnt, char c)
{
    if (cnt > 0) {
        char cc[] = " "; cc[0] = c;
        for (uint8_t i = 1; i < cnt; i++) { display2.print(cc); }
    }
}

void printInt32_tAtWidthDisplay2(int32_t value, uint8_t width, char c)
{
    char buf[16];
    ltoa(value, buf, 10);
    display2.print(buf);
    display2.print(c);
    uint8_t len = strlen(buf);
    for (uint8_t i = len; i < width; i++)
        display2.print(' ');
}

void printFloatAtWidthDisplay2(float value, uint8_t width, char c, uint8_t decimals)
{
    char buf[10];
    dtostrf(value, width - getFloatCharCnt(value), decimals, buf); // 1 decimal
    display2.print(buf);
    display2.print(c);
}

void printDoubleAtWidthDisplay2(double value, uint8_t width, char c)
{
    char buf[10];
    dtostrf(value, width - getDoubleCharCnt(value), 1, buf); // 1 decimal
    display2.print(buf);
    display2.print(c);
}

//======================================================TOOLS_settings======================================================
void set_Default()
{
    GlobalSettings tempSets;
    memcpy(&settings, &tempSets, sizeof settings);
}

void sets_Load()
{
    EEPROM.get(0, settings);
    if (settings.settingsCheckValue != SETTINGS_CHKVAL) { set_Default(); }
}

void sets_Save()
{
    if (memcmp(&settings, &oldSettings, sizeof(settings)) != 0) {
        EEPROM.put(0, settings);
        oldSettings = settings;
    }
}

void updateSettings()
{
    // --- Retune controller ONLY if changed ---
    if (selectedAxis->settings.targetSetPoint  != selectedAxis->lastSetPoint  ||
        selectedAxis->settings.Kp        != selectedAxis->lastKp        ||
        selectedAxis->settings.Ki        != selectedAxis->lastKi        ||
        selectedAxis->settings.Kd        != selectedAxis->lastKd        ||
        selectedAxis->settings.amplitude != selectedAxis->lastAmplitude ||
        selectedAxis->settings.period    != selectedAxis->lastPeriod)
        {
            selectedAxis->lastSetPoint  = selectedAxis->settings.targetSetPoint;
            selectedAxis->lastKp        = selectedAxis->settings.Kp;
            selectedAxis->lastKi        = selectedAxis->settings.Ki;
            selectedAxis->lastKd        = selectedAxis->settings.Kd;
            selectedAxis->lastAmplitude = selectedAxis->settings.amplitude;
            selectedAxis->lastPeriod    = selectedAxis->settings.period;

            updatePIDTunings(axis1, posPid1);
            updatePIDTunings(axis1, velPid1);
            updatePIDTunings(axis2, posPid2);
            updatePIDTunings(axis2, velPid2);

            updateAllItems = true;
        }

    //updateAllItems = true;
}

float Filter(float New, float Current, float alpha, float maxValue)
{
    float diff         = fabs(New - Current);
    // Normalize difference by max value for relative comparison
    float relativeDiff = diff / maxValue;
    // Smaller alpha (more responsive) when difference is large
    float adjustedAlpha = alpha / (1.0 + relativeDiff * 0.15);
    // Serial.printf(", adjAlpha: %.4f |\n", adjustedAlpha);
    return (1.0 - adjustedAlpha) * New + adjustedAlpha * Current;
}

float roundTo(float value, int decimals)
{
    float multiplier = pow(10.0, decimals);
    return round(value * multiplier) / multiplier;
}