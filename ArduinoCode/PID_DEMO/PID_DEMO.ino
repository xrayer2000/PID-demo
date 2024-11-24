#include <PID_v1.h>
double Setpoint;
double measuredRpm = 0;
double dutyCycleMotor = 0;
double Kp = 0.2;
double Ki = 0;
double Kd = 0;
PID myPID(&measuredRpm, &dutyCycleMotor, &Setpoint, Kp, Ki, Kd, DIRECT);

int motor1pin1 = 6;
int motor1pin2 = 7;
int enablePWMpin = 9;
int encoderApin = 2;
int encoderBpin = 3;
int KpPin = 0;
int SetPointPin = 1;

//rotary encoder
int encoderA = 0;
int encoderB = 0;
int lastEncoderA = 0;
long counter = 0;
unsigned long lastTime = 0;
double deltaT = 0;
unsigned long previousTime = 0;
  
void setup() {
  Serial.begin(9600);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(enablePWMpin,  OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderA, INPUT);
  pinMode(KpPin, INPUT);
  pinMode(SetPointPin, INPUT);

  Setpoint = 400;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,255);
  lastTime = millis();

}

void loop() {
  unsigned long currentTime = millis();
  encoderA = digitalRead(encoderApin);
  encoderB = digitalRead(encoderBpin);
  if (encoderA == 0 && lastEncoderA == 1)
  {
    if (encoderB == 1)
      counter ++;
    else
      counter --;
  }
  lastEncoderA = encoderA;


  double numberOfTeeth = 100.0;
  measuredRpm = (counter * 60.0) / (numberOfTeeth);
  Kp = analogRead(KpPin)/1024.0;
  Setpoint = analogRead(SetPointPin);
  myPID.SetTunings(Kp, Ki, Kd, true);
  myPID.Compute();
  if(currentTime - previousTime >= 1000)
  {
    previousTime = currentTime;  
    
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print(", (Setpoint): ");
    Serial.print(Setpoint);
    Serial.print(", Rpm (INPUT): ");
    Serial.print(measuredRpm);
    Serial.print(", DuCy (OUTPUT): ");
    Serial.println(dutyCycleMotor);
    
    counter = 0;
  }
   
  

  


  
  analogWrite(enablePWMpin, dutyCycleMotor); //ENA  pin
  digitalWrite(motor1pin1,  LOW);
  digitalWrite(motor1pin2, HIGH);

  deltaT = millis() - currentTime;
  
}
