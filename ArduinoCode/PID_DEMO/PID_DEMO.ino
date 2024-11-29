#include <PID_v1.h>
int maxRpm = 2400;
double Setpoint = 0;
double measuredRpm = 0;
double dutyCycleMotor = 0;
double Kp = 0;
double Ki = 0;
double Kd = 0;
PID myPID(&measuredRpm, &dutyCycleMotor, &Setpoint, Kp, Ki, Kd, DIRECT);

int motor1pin1 = 18;
int motor1pin2 = 19;
int enablePWMpin = 21;
int encoderApin = 33;
int encoderBpin = 32;
int KpPin = 25;
int KiPin = 26;
int KdPin = 27;
int SetPointPin = 14;

//rotary encoder
int numberOfTeeth = 100;
int encoderA = 0;
int encoderB = 0;
int lastEncoderA = 0;
long countA = 0;
long countB = 0;
int Dir = 0;
unsigned long lastTime = 0;
double deltaT = 0;
unsigned long previousTime = 0;

const int PWM_CHANNEL = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 500;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits

void setup() {
  Serial.begin(115200);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(enablePWMpin,  OUTPUT);
  pinMode(encoderApin, INPUT);
  pinMode(encoderBpin, INPUT);
  pinMode(KpPin, INPUT);
  pinMode(KiPin, INPUT);
  pinMode(KdPin, INPUT);
  pinMode(SetPointPin, INPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  lastTime = millis();
  analogReadResolution(9);   

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enablePWMpin, PWM_CHANNEL);
  attachInterrupt(digitalPinToInterrupt(encoderApin), pulseA ,RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBpin), pulseB ,RISING);
}
void pulseA()
{
  checkDirection();
  countA += Dir;
}
void pulseB()
{
  countB += Dir;
}
void checkDirection()
{
  if(digitalRead(encoderB) == HIGH)
    Dir = 1;
  else
    Dir = -1;
}
void loop() {
  unsigned long currentTime = millis();

  measuredRpm = ((countA) * 10 * 60.0) / (numberOfTeeth);
  Kp = analogRead(KpPin)/512.0/maxRpm*8;
  Ki = analogRead(KiPin)/512.0/maxRpm*4;
  Kd = analogRead(KdPin)/512.0/maxRpm;
  
  Setpoint = analogRead(SetPointPin)/ 512.0 * maxRpm; // ;
  
  if (currentTime - previousTime >= 100)
  {
    previousTime = currentTime;
    myPID.SetTunings(Kp, Ki, Kd, true);
    myPID.Compute();
    dutyCycleMotor = constrain(dutyCycleMotor, 0.0, 1.0);
    
       Serial.print("Kp: ");
       
       Serial.print(Kp, 6);
       
       Serial.print(", Ki: ");
       Serial.print(Ki, 6);
       
       Serial.print(", Kd: ");
       Serial.print(Kd, 6);
       
       Serial.print(", (Setpoint): ");
       Serial.print(Setpoint,0);
       
       Serial.print(", Rpm (INPUT): ");
       
       Serial.print(measuredRpm, 0);
       
       Serial.print(", DuCy (OUTPUT): ");
       Serial.println(dutyCycleMotor,4);

    countA = 0;
    countB = 0;
  }

  ledcWrite(PWM_CHANNEL, dutyCycleMotor * 255.0);
  //analogWrite(enablePWMpin, Setpoint); //ENA  pin //dutyCycleMotor * 255
  digitalWrite(motor1pin1,  LOW);
  digitalWrite(motor1pin2, HIGH);

  deltaT = millis() - currentTime;

}
