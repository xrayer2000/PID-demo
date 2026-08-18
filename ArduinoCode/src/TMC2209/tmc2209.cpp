#include "main.h"
#include "tmc2209.h"
#include "Axis/axis.h"

// UART1 -> Driver 1
HardwareSerial SerialTMC1(UART1_RX, UART1_TX);

// UART2 -> Driver 2
HardwareSerial SerialTMC2(UART2_RX, UART2_TX);

TMC2209Stepper driver1(&SerialTMC1, R_SENSE, 0);
TMC2209Stepper driver2(&SerialTMC2, R_SENSE, 0);

void initTMC2209()
{
    Serial.println("initTMC2209()");

    SerialTMC1.begin(115200);
    SerialTMC2.begin(115200);

    driver1.begin();
    driver2.begin();

    Serial.println("before IOIN1");
    uint32_t ioin1 = driver1.IOIN();
    Serial.println("after IOIN1");

    Serial.print("Driver1 IOIN = 0x");
    Serial.println(ioin1, HEX);

    Serial.println("before IOIN2");
    uint32_t ioin2 = driver2.IOIN();
    Serial.println("after IOIN2");

    Serial.print("Driver2 IOIN = 0x");
    Serial.println(ioin2, HEX);

    driver1.toff(4);
    driver1.hstrt(4);
    driver1.hend(1);
    driver2.toff(4);
    driver2.hstrt(4);
    driver2.hend(1);

    driver1.rms_current(1500);
    driver2.rms_current(1500);

    driver1.mstep_reg_select(true);
    driver2.mstep_reg_select(true);

    axis1.settings.microsteps = 8;
    axis2.settings.microsteps = 8;
    driver1.microsteps(axis1.settings.microsteps);
    driver2.microsteps(axis2.settings.microsteps);

    // driver1.intpol(true); // interpolerar internt till 256 mikrosteg ändå → fortsatt mjuk rörelse
    // driver2.intpol(true); // interpolerar internt till 256 mikrosteg ändå → fortsatt mjuk rörelse

    driver1.en_spreadCycle(true);  
    driver2.en_spreadCycle(true);

    driver1.pdn_disable(true);
    driver2.pdn_disable(true);

    Serial.print("CS_ACTUAL = ");
    Serial.println(driver1.cs_actual());

    Serial.print("IRUN: ");
    Serial.println(driver1.irun());

    Serial.print("IHOLD: ");
    Serial.println(driver1.ihold());

    Serial.print("VSENSE: ");
    Serial.println(driver1.vsense());

    Serial.print("DRV_STATUS: 0x");
    Serial.println(driver1.DRV_STATUS(), HEX);

    Serial.print("CHOPCONF: 0x");
    Serial.println(driver1.CHOPCONF(), HEX);

    Serial.print("Driver1 current: ");
    Serial.println(csToMilliamps(driver1.cs_actual(), R_SENSE, driver1.vsense()));

    Serial.print("Driver2 current: ");
    Serial.println(csToMilliamps(driver2.cs_actual(), R_SENSE, driver2.vsense()));

    Serial.print("Driver1 microsteps: ");
    Serial.println(driver1.microsteps());

    Serial.print("Driver2 microsteps: ");
    Serial.println(driver2.microsteps());
}

float csToMilliamps(uint8_t cs, float rSense, bool vsenseHigh)
{
    float vfs = vsenseHigh ? 0.180f : 0.325f;
    float i_rms = (cs + 1) / 32.0f * (vfs / rSense) / sqrtf(2.0f);
    return i_rms * 1000.0f; // convert A to mA
}