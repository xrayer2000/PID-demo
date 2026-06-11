#include "main.h"
#include "TaskDisplay.h"

//--------------------------------------------------------------------------------------------------
// Object definitions
//--------------------------------------------------------------------------------------------------
TwoWire Wire1(1);  // I2C2

RotaryEncoderAccel encoder(outputA, outputB);  // GPIO32 och GPIO33 på ESP32
PressButton btnOk(confirmBtnPin, 10); //debounce (ISR-attached inside library)

U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//--------------------------------------------------------------------------------------------------
// Global variable definitions
//--------------------------------------------------------------------------------------------------
Mysettings settings;
Mysettings oldSettings;

HardwareTimer *stepTimer = nullptr;
volatile uint32_t stepFrequency = 0;

float angle        = 0.0f;
float stepVelocity = 0.0f;

PID_Controller pid(&angle, &stepVelocity, &settings.setPoint);

// Magnetic encoder - AS5600
float absoluteAngle = 0.0f;
float rpm_measured  = 0.0f;

//stepper motor parameters
uint8_t microstepping = 16;  // 16, 32, 64, 128 // there is no 8 microstepping mode on the TMC2209
float   steps_per_rev = 200.0f * 16;

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
float passedTime          = 0.0f;
float previousPassedTime1 = 0.0f;
float previousPassedTime2 = 0.0f;
float timeLastTouched     = 0.0f;
bool  displaySleeping     = false;

// Loop time measurement variables
static uint32_t last = 0;
uint32_t now         = 0;
uint32_t loopTime    = 0;
uint32_t sumLoopTime = 0;
uint32_t loopCount   = 0;
uint32_t avgLoopTime = 0;

//sensors
unsigned long lastReadTime  = 0;
const float   readInterval  = 500;  // microseconds

// Last-known settings (change detection)
float lastSetPoint  = 0.0f;
float lastKp        = 0.0f;
float lastKi        = 0.0f;
float lastKd        = 0.0f;
float lastAmplitude = 0.0f;
float lastPeriod    = 0.0f;

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
    // Create display task
    xTaskCreate(TaskDisplay, "Display", 256, nullptr, 1, &taskDisplayHandle);
    // Start the scheduler
    vTaskStartScheduler();

    Serial.begin(115200);
    // while(!Serial);
    Serial.println("Boot");

    pinMode(EN_PIN,     OUTPUT);
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_1,  OUTPUT);
    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(DIR_PIN_2,  OUTPUT);
    pinMode(MS1_PIN,    OUTPUT);
    pinMode(MS2_PIN,    OUTPUT);
    pinMode(MS3_PIN,    OUTPUT);

    // pinMode(SDA_PIN_0, INPUT_PULLUP);
    // pinMode(SCL_PIN_0, INPUT_PULLUP);

    pinMode(confirmBtnPin, INPUT_PULLUP);
    pinMode(outputA,       INPUT_PULLUP);
    pinMode(outputB,       INPUT_PULLUP);

    setMicrostepTMC2209(microstepping);

    // Initialize encoder interrupts (confirm button interrupt disabled)
    attachInterrupt(digitalPinToInterrupt(outputA), handleInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(outputB), handleInterrupt, CHANGE);

    // Initialize button ISR
    btnOk.init();

    // -------- I2C - Oled--------
    Wire.setSDA(SDA_PIN_0);
    Wire.setSCL(SCL_PIN_0);
    Wire.begin();
    Wire.setClock(400000);

    // -------- I2C - AS5600--------
    Wire1.setSDA(SDA_PIN_1);
    Wire1.setSCL(SCL_PIN_1);
    Wire1.begin();
    Wire1.setClock(400000);

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

    sets_Load();

    last = micros();

    lastSetPoint  = settings.setPoint;
    lastKp        = settings.Kp;
    lastKi        = settings.Ki;
    lastKd        = settings.Kd;
    lastAmplitude = settings.amplitude;
    lastPeriod    = settings.period;

    pid.SetOutputLimits(-15000, 15000);
    pid.SetIntegralLimit(10000);

    initAS5600();
    // setMicrostepA4988(1);
    // microstepTest();
    initStepTimer();

    digitalWrite(EN_PIN, LOW); // Enable all stepper drivers
}

//==================================================================================================
// Loop
//==================================================================================================
void loop()
{
    passedTime = millis() * 0.001f;
    float minutesSinceLastAction = (passedTime * 0.0166667f) - timeLastTouched;
    bool  shouldSleep            = (minutesSinceLastAction > settings.timeBeforeDisable);

    if (shouldSleep != displaySleeping) {
        displaySleeping = shouldSleep;
        display1.setPowerSave(displaySleeping);
        display2.setPowerSave(displaySleeping);
    }

    const float UPDATE_INTERVAL1 = 0.05f;
    const float UPDATE_INTERVAL2 = 0.05f;
    bool shouldUpdate1 = (passedTime - previousPassedTime1 >= UPDATE_INTERVAL1);
    bool shouldUpdate2 = (passedTime - previousPassedTime2 >= UPDATE_INTERVAL2);

    updateSettings();
    updateSensorValues();

    if (shouldUpdate1) {
        static uint32_t lastPrint = 0;
        uint32_t now      = millis();
        uint32_t ms       = millis();
        uint32_t sec      = ms / 1000;
        uint32_t centisec = (ms % 1000) / 10;
        //updateDisp2();
        lastPrint = now;

        Serial.printf(
            "Time:%4lu.%02lu\t | SetPoint:%6ld | Angle:%6ld | RPM:%6ld | Vel:%6ld | Kp:%4ld | Ki:%4ld | Kd:%4ld | Mode:%s | Micro:1/%lu | Loop:%4lu | LoopFreq:%6lu\n",
            sec, centisec,
            (int32_t)settings.setPoint,
            (int32_t)absoluteAngle,
            (int32_t)rpm_measured,
            (int32_t)stepVelocity,
            (int32_t)settings.Kp,
            (int32_t)settings.Ki,
            (int32_t)settings.Kd,
            systemModeToString(settings.systemMode),
            (uint32_t)microstepping,
            avgLoopTime,
            1000000UL / avgLoopTime
        );

        // Serial.print("Loop: ");
        // Serial.println(avgLoopTime);

        previousPassedTime1 = passedTime;
    }

    switch (currPage) {
        case MENU_ROOT:        page_MenuRoot();         break; //page_MenuRoot() calls doPointerNavigation(), and it sets updateAllItems if scrolled
        case MENU_SYSTEM_MODE: page_MENU_SYSTEM_MODE(); break;
    }

    if ((updateAllItems || updateItemValue) && shouldUpdate2) {
        display1.sendBuffer(); //gör denna sist
        previousPassedTime2 = passedTime;
        updateAllItems  = false;
        updateItemValue = false;
    }

    // Consume confirm button timestamp from library ISR and update touch timers
    uint32_t t = btnOk.consumeTouchMs();
    if (t != 0) {
        timeLastTouched = passedTime / 60.0f;
    }

    //loop time measurement
    //----------------------------------------------------------------------------------
    now = micros();

    loopCount++;

    if (loopCount == 1)
        last = now;

    if (loopCount >= 1000) {
        sumLoopTime = now - last;
        avgLoopTime = sumLoopTime / (loopCount - 1);
        loopCount   = 0;
        last        = now; // ← you reset count but never reset `last`!
    }

    //----------------------------------------------------------------------------------
    //Stepper motor control
    //updateMicrostepCycle();
    updateMode();

    if (settings.systemMode != MODE_RPM) {
        if (pid.Compute()) {
            setAngle();
        }
    }
}

//==================================================================================================
// Menu pages
//==================================================================================================
void page_MenuRoot() //=================================================ROOT_MENU============================================
{
    if (initPage) {
        cursorPos  = root_pntrPos;
        dispOffset = root_dispOffset;
        initMenuPage(F("MAIN MENU"), 7);
        initPage = false;
    }

    if (btnOk.Pressed()) {
        FlashPointer();
        changeValues[cursorPos] = !changeValues[cursorPos];
        edditing = !edditing;
    }

         if (changeValues[0]) { incrementDecrementFloat(&settings.setPoint, 10.0, -20000.0, 20000.0); }
    else if (changeValues[1])   incrementDecrementFloat(&settings.Kp,       1.0,  0.0, 100.0);
    else if (changeValues[2])   incrementDecrementFloat(&settings.Ki,       0.5,  0.0,  50.0);
    else if (changeValues[3])   incrementDecrementFloat(&settings.Kd,       1.0,  0.0, 100.0);
    else if (changeValues[4]) { currPage = MENU_SYSTEM_MODE; initPage = true; updateAllItems = true; changeValues[4] = false; edditing = false; return; }
    else if (changeValues[5])   incrementDecrementFloat(&settings.amplitude, 5.0, 0.0, 20000.0);
    else if (changeValues[6])   incrementDecrementFloat(&settings.period,    0.1, 0.0,    20.0);
    else
        doPointerNavigation();

    if (!(updateAllItems | updateItemValue)) return;

    for (uint8_t i = 1; i <= 7; i++) {
        if (menuItemPrintable(1, i)) {
            switch (i) {
                case 1: display1.print(F("Setpoint:             ")); break;
                case 2: display1.print(F("Kp:                   ")); break;
                case 3: display1.print(F("Ki:                   ")); break;
                case 4: display1.print(F("Kd:                   ")); break;
                case 5: display1.print(F("Mode:                 ")); break;
                case 6: display1.print(F("Amplitude:            ")); break;
                case 7: display1.print(F("Period:               ")); break;
            }
        }
        if (menuItemPrintable(10, i)) {
            switch (i) {
                case 1: printDoubleAtWidth(settings.setPoint,  3, " ");      break;
                case 2: printDoubleAtWidth(settings.Kp,        4, " ");      break;
                case 3: printDoubleAtWidth(settings.Ki,        4, " ", 2);   break;
                case 4: printDoubleAtWidth(settings.Kd,        4, " ", 2);   break;
                case 5: printStringAtWidth(systemModeToString(settings.systemMode), 4); break;
                case 6: printDoubleAtWidth(settings.amplitude, 4, " ", 2);   break;
                case 7: printDoubleAtWidth(settings.period,    4, " ", 2);   break;
            }
        }
    }
}

void page_MENU_SYSTEM_MODE() //=================================================SYSTEM_MODE============================================
{
    if (initPage) {
        cursorPos  = 0;
        dispOffset = 0;
        initMenuPage(F("SYSTEM MODE"), 4);
        initPage = false;
    }

    if (btnOk.Pressed()) {
        FlashPointer();
        changeValues[cursorPos] = true;
    }

         if (changeValues[0]) { settings.systemMode = MODE_OSCILATION; currPage = MENU_ROOT; initPage = true; /*clear the flag so we don't re-enter*/ changeValues[0] = false; return; }
    else if (changeValues[1]) { settings.systemMode = MODE_SETPOINT;   currPage = MENU_ROOT; initPage = true; changeValues[1] = false; return; }
    else if (changeValues[2]) { settings.systemMode = MODE_RPM;        currPage = MENU_ROOT; initPage = true; changeValues[2] = false; return; }
    else if (changeValues[3]) {                                          currPage = MENU_ROOT; initPage = true; changeValues[3] = false; return; }
    else
        doPointerNavigation();

    if (!(updateAllItems | updateItemValue)) return;

    for (uint8_t i = 1; i <= 4; i++) {
        if (menuItemPrintable(1, i)) {
            switch (i) {
                case 1: display1.print(F("OSCILATION")); break;
                case 2: display1.print(F("SETPOINT  ")); break;
                case 3: display1.print(F("RPM       ")); break;
                case 4: display1.print(F("Back      ")); break;
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

        timeLastTouched = passedTime / 60.0; // Update last touched time (encoder rotation)

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
        timeLastTouched = passedTime / 60.0;
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

        //updateItemValue = true;
        timeLastTouched = passedTime / 60.0;
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

        timeLastTouched = passedTime / 60.0;
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
void printPointer()
{
    //Serial.println("printPointer");
    display1.drawStr(0, 1 * CHAR_Y, " ");
    display1.drawStr(0, 2 * CHAR_Y, " ");
    display1.drawStr(0, 3 * CHAR_Y, " ");
    display1.drawStr(0, 4 * CHAR_Y, " ");
    display1.drawStr(0, 5 * CHAR_Y, " ");
    display1.drawStr(0, (cursorPos - dispOffset + 1) * CHAR_Y, "*");
    display1.sendBuffer();
}

void FlashPointer()
{
    display1.drawStr(0, 1 * CHAR_Y, " ");
    display1.drawStr(0, 2 * CHAR_Y, " ");
    display1.drawStr(0, 3 * CHAR_Y, " ");
    display1.drawStr(0, 4 * CHAR_Y, " ");
    display1.drawStr(0, 5 * CHAR_Y, " ");
    display1.sendBuffer();                  //nytt

    delay(100);
    //Serial.println("FlashPointer");
    display1.drawStr(0, (cursorPos - dispOffset + 1) * CHAR_Y, "*");
    display1.sendBuffer();                  //nytt
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

const char* systemModeToString(SystemMode mode)
{
    switch (mode) {
        case MODE_OSCILATION: return "OSCILATION";
        case MODE_SETPOINT:   return "SETPOINT";
        case MODE_RPM:        return "RPM";
        default:              return "UNKNOWN";
    }
}

//======================================================DISPLAY_2======================================================
void updateDisp2()
{
    display2.clearBuffer();
    const __FlashStringHelper* labels[4] = {
        F("AbsoluteAngle:"),
        F("RPM:          "),
        F(":             "),
        F(":             ")
    };

    menuItemPrintableDisp2(1,  1); display2.print(F("AbsoluteAngle:"));
    menuItemPrintableDisp2(15, 1); printInt32_tAtWidthDisplay2(absoluteAngle, 3, ' ');

    menuItemPrintableDisp2(1,  2); display2.print(F("RPM:          "));
    menuItemPrintableDisp2(15, 2); printInt32_tAtWidthDisplay2((int32_t)rpm_measured, 3, ' ');

    // menuItemPrintableDisp2(1,3); display2.print(F(":             "));
    // menuItemPrintableDisp2(15,3); printInt32_tAtWidthDisplay2(2,3,' ');

    // menuItemPrintableDisp2(1,4); display2.print(F(":             "));
    // menuItemPrintableDisp2(15,4); printInt32_tAtWidthDisplay2(3,3,' ');

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
    Mysettings tempSets;
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
    if (settings.setPoint != lastSetPoint ||
        settings.Kp       != lastKp       ||
        settings.Ki       != lastKi       ||
        settings.Kd       != lastKd       ||
        settings.amplitude != lastAmplitude ||
        settings.period    != lastPeriod)
    {
        lastSetPoint  = settings.setPoint;
        lastKp        = settings.Kp;
        lastKi        = settings.Ki;
        lastKd        = settings.Kd;
        lastAmplitude = settings.amplitude;
        lastPeriod    = settings.period;

        pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
        updateAllItems = true;
    }

    //updateAllItems = true;
}

void updateSensorValues()
{
    uint32_t now = micros();
    uint32_t dt  = now - lastReadTime;

    if (dt < readInterval) return;
    lastReadTime = now;

    static uint16_t prevRaw     = 0;
    static bool     initialized = false;

    uint16_t rawAngle = readAS5600RawFast();

    if (!initialized) {
        prevRaw     = rawAngle;
        initialized = true;
        return;
    }

    int16_t diff = rawAngle - prevRaw;
    prevRaw = rawAngle;

    if      (diff >  2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;

    absoluteAngle += diff * 0.087890625f;
    angle          = absoluteAngle;

    float dt_sec = dt * 1e-6f;

    if (dt_sec > 0.0f) {
        float rpm_instant = (diff / 4096.0f) / dt_sec * 60.0f;
        rpm_measured = Filter(rpm_instant, rpm_measured, 0.99f, 100.0f);
    }
}

void initAS5600()
{
    Wire1.beginTransmission(AS5600_ADDR);
    Wire1.write(AS5600_RAW_ANGLE);
    Wire1.endTransmission();
}

inline uint16_t readAS5600RawFast()
{
    Wire1.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    uint16_t high = Wire1.read();
    uint16_t low  = Wire1.read();
    return ((high & 0x0F) << 8) | low;
}

void stepISR()
{
    digitalWrite(STEP_PIN_1, HIGH);
    digitalWrite(STEP_PIN_1, LOW);
}

void initStepTimer()
{
    pinMode(STEP_PIN_1, OUTPUT);
    stepTimer = new HardwareTimer(TIM2);
    stepTimer->setOverflow(1000, HERTZ_FORMAT);
    stepTimer->attachInterrupt(stepISR);
}

void setStepFrequency(uint32_t steps_per_sec)
{
    stepFrequency = steps_per_sec;
    if (steps_per_sec == 0) {
        stepTimer->pause();
        digitalWrite(STEP_PIN_1, LOW);
        return;
    }
    stepTimer->setOverflow(steps_per_sec, HERTZ_FORMAT);
    stepTimer->refresh();  // force immediate reload //// !!!!!VIKTIG
    stepTimer->resume();
}

void setDirection(int velocity)
{
    digitalWrite(DIR_PIN_1, velocity >= 0);
}

void setAngle()
{
    static bool enabled = false;

    const float ON_THRESHOLD  = 8.0f;  // start moving
    const float OFF_THRESHOLD = 1.0f;  // stop moving

    float v = fabs(stepVelocity);

    if (!enabled && v > ON_THRESHOLD) {
        enabled = true;
        digitalWrite(EN_PIN, LOW); // enable driver
    } else if (enabled && v < OFF_THRESHOLD) {
        enabled = false;
        setStepFrequency(0);
        stepVelocity = 0;
        digitalWrite(EN_PIN, HIGH); // disable driver
    }

    if (enabled) {
        setDirection(stepVelocity);
        setStepFrequency(v);
    }
}

void setMicrostepTMC2209(uint16_t microstep) //this version dont have 1/8 microstep
{
    switch (microstep) {
        case 8:  digitalWrite(MS1_PIN, LOW);  digitalWrite(MS2_PIN, LOW);  break;
        case 16: digitalWrite(MS1_PIN, HIGH); digitalWrite(MS2_PIN, LOW);  break;
        case 32: digitalWrite(MS1_PIN, LOW);  digitalWrite(MS2_PIN, HIGH); break;
        case 64: digitalWrite(MS1_PIN, HIGH); digitalWrite(MS2_PIN, HIGH); break;
    }
}

void setMicrostepA4988(uint8_t microstep)
{
    switch (microstep) {
        case 1:  // Full step
            digitalWrite(MS1_PIN, LOW);  digitalWrite(MS2_PIN, LOW);  digitalWrite(MS3_PIN, LOW);  break;
        case 2:  // Half step
            digitalWrite(MS1_PIN, HIGH); digitalWrite(MS2_PIN, LOW);  digitalWrite(MS3_PIN, LOW);  break;
        case 4:  // Quarter step
            digitalWrite(MS1_PIN, LOW);  digitalWrite(MS2_PIN, HIGH); digitalWrite(MS3_PIN, LOW);  break;
        case 8:  // Eighth step
            digitalWrite(MS1_PIN, HIGH); digitalWrite(MS2_PIN, HIGH); digitalWrite(MS3_PIN, LOW);  break;
        case 16: // Sixteenth step
            digitalWrite(MS1_PIN, HIGH); digitalWrite(MS2_PIN, HIGH); digitalWrite(MS3_PIN, HIGH); break;
        default: // fallback to full step
            digitalWrite(MS1_PIN, LOW);  digitalWrite(MS2_PIN, LOW);  digitalWrite(MS3_PIN, LOW);  break;
    }
}

const uint16_t microModes[] = {1, 2, 4, 8, 16};
const uint8_t  numModes     = sizeof(microModes) / sizeof(microModes[0]);
uint8_t  currentMode = 0;
uint32_t lastChange  = 0;

void updateMicrostepCycle()
{
    uint32_t now = millis();

    if (now - lastChange >= 5000)   // change every 5 seconds
    {
        microstepping = microModes[currentMode];
        steps_per_rev = 200.0f * microstepping;
        setMicrostepA4988(microModes[currentMode]);

        //Serial.printf("Microstepping set to 1/%u\n", microModes[currentMode]);

        currentMode++;
        if (currentMode >= numModes) currentMode = 0;
        lastChange = now;
    }
}

void microstepTest()
{
    const uint16_t testSteps = 200;

    updateSensorValues();
    float startAngle = absoluteAngle;

    for (uint16_t i = 0; i < testSteps; i++) {
        digitalWrite(STEP_PIN_1, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP_PIN_1, LOW);
        delayMicroseconds(2000);
        updateSensorValues();   // keep encoder updated
    }

    updateSensorValues();
    float delta = absoluteAngle - startAngle;

    Serial.print("Microstep test: ");
    Serial.print(testSteps);
    Serial.print(" pulses -> ");
    Serial.print(delta, 2);   // 2 decimal places
    Serial.println(" deg");
}

void updateMode()
{
    static int lastMode = -1;

    if (settings.systemMode != lastMode) {
        absoluteAngle = 0;      // reset once when mode changes
        lastMode      = settings.systemMode;
    }

    switch (settings.systemMode) {
        case MODE_OSCILATION: {
            uint32_t period_ms  = settings.period * 1000.0f;
            uint32_t phase      = millis() % period_ms;
            float    newSetPoint = (phase < period_ms / 2.0f)
                                   ? settings.amplitude
                                   : -settings.amplitude;

            // Reset integral whenever setpoint flips
            if ((newSetPoint > 0) != (settings.setPoint > 0))
                pid.ResetIntegral();

            settings.setPoint = newSetPoint;
        } break;

        case MODE_SETPOINT:
            // nothing to compute
        break;

        case MODE_RPM: {
            float steps_per_sec = abs(settings.setPoint) / 60.0f * steps_per_rev;

            static float lastSteps = -1;
            if (steps_per_sec != lastSteps) {
                setDirection(settings.setPoint >= 0 ? 1 : -1);
                setStepFrequency((uint32_t)steps_per_sec);
                lastSteps = steps_per_sec;
            }
            stepVelocity = settings.setPoint;
        } break;
    }
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