// Spice Rack Controller
#include "settings.h"
#include "PIDcontroller.h"
#include <Encoder.h>

// Import required libraries for display
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

/*
HardWare I2C pins
A4   SDA
A5   SCL
*/

// Pin definitions
#define OLED_RESET  16  // Pin 15 -RESET digital signal

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

ArducamSSD1306 display(OLED_RESET); // FOR I2C

// Initialize the PIDloops for the heaters
PIDloop heater1(Kp, Ki, Kd);
PIDloop heater2(Kp, Ki, Kd);

// Initialize Encoder (for the Knob)
Encoder encoder(encoderApin, encoderBpin);

// VARIABLES
bool go = false; // the state of the drive system (go or stop)
//bool homed = false;  // has the stepper been properly homed
//int limitSwitch = LOW;  // the state of the limit switch
//int8_t dir = 1; //the current state direction of the drive system (up is HIGH)
//int8_t lastDir = 1; //the last direction of the system, to check if it switched
int buttonState = HIGH; // the current reading from the input pin
int currentPosition; //the current position [pulses]
int tempChange; // the amount to change the target temp by [K]
//float currentSpeed; //the current speed (average) [pulses / ms]
//int profilePositions[4]; //{x0, x1, x2, x3} x0 is the start position, and x3 is the end position [pulses]
//unsigned int profileTimes[4]; //{t0, t1, t2, t3} t0 is the start time, and t3 is the end time [ms]
//bool integrateStart = true; // initializes the start of an integration profile
float temp1; // temperature reading of the thermistor [degK]
float temp2;
const int numReadings = 5; // number of readings for moving average
int readIndex; // index to update the readings
float temp1readings[numReadings]; // for moving average
float temp2readings[numReadings];
float temp1total; // for moving average
float temp2total;

void setup() {
  
  // SSD1306 Init
  display.begin();  // Switch OLED
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Hello, Hot Stuff!");
  display.display();

  delay(2000);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Mystic Melter v1.0.0");
  display.display();

  delay(2000);

//  Serial.begin(9600);
  
  // put your setup code here, to run once:
//  setGains();
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(heaterPin1, OUTPUT);
  pinMode(heaterPin2, OUTPUT);
//  pinMode(limitSwitchPin, INPUT);
//  pinMode(PWMpin, OUTPUT);
//  pinMode(dirPin, OUTPUT);

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
   
//  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  // setup the temp readings
  for (int i = 0; i < numReadings; i++) {
    temp1readings[i] = 0;
    temp2readings[i] = 0;
  }
  
  updateSensors();
//  Serial.print("Stroke [mm]: ");
//  Serial.println(strokeMM,4);
//  Serial.print("Stroke [pulses]: ");
//  Serial.println(stroke,4);
//  Serial.print("Max Speed [mm/s]: ");
//  Serial.println(maxSpeedMM,4);
//  Serial.print("Max Speed [pulses/ms]: ");
//  Serial.println(maxSpeed,4);
//  Serial.print("Acceleration [mm/s^2]: ");
//  Serial.println(accelMM,4);
//  Serial.print("Acceleration [pulses/ms^2]: ");
//  Serial.println(accel,4);
//  Serial.print("Acceleration Time [ms]: ");
//  Serial.println(accelTime,4);
//  Serial.print("Acceleration Distance [pulses]: ");
//  Serial.println(accelDistance,4);
//  Serial.print("Kp: ");
//  Serial.println(Kp,4);
//  Serial.print("pulseKp: ");
//  Serial.println(pulseKp,4);
//  Serial.print("Ki: ");
//  Serial.println(Ki,4);
//  Serial.print("pulseKi: ");
//  Serial.println(pulseKi,6);
//  Serial.print("Kd: ");
//  Serial.println(Kd,4);
//  Serial.print("pulseKd: ");
//  Serial.println(pulseKd,4);

//  Serial.println("READY");
//  Serial.println("Time [ms], Setpoint [F], Temp1 [F], Volts [V]");
}

void voltageDriver(float volts, int PWMpin) {
  volts = constrain(volts, 0, voltRange);
  if (volts <= 0){
//    digitalWrite(dirPin, HIGH);
    analogWrite(PWMpin, 0);
  }
//  else if (milliVolts < 0) {
//    digitalWrite(dirPin, HIGH);
//    analogWrite(PWMpin, map(abs(milliVolts),0,24000,15,255));
//  }
  else {
//    digitalWrite(dirPin, LOW);
    analogWrite(PWMpin, map(abs(volts),0,voltRange,0,255));
  }
}

static inline int8_t sgn(float val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void updateSensors()
{
  // update button and limit switches
  int reading = digitalRead(buttonPin);  // read the state of the switch into a local variable
//  limitSwitch = digitalRead(limitSwitchPin);

  // read encoder and calculate the speed
  int newPosition = encoder.read();
//  static int lastPosition = newPosition;
  // only calculate the speed if time elapsed has been more than set sample time
//  if (now - lastTime >= sampleTime){
//    currentSpeed = (newPosition - lastPosition)/(float)(now - lastTime);
//    // save static variables for next round
//    lastTime = now;
//    lastPosition = newPosition;
//  } 
  tempChange = newPosition - currentPosition;
  currentPosition = newPosition;

  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;

  static int lastButtonState = HIGH; // the previous reading from the input pin
  static unsigned int lastDebounceTime = 0;  // the last time the output pin was toggled
  // reset the debouncing timer if reading has changed
  if (reading != lastButtonState) lastDebounceTime = now;

  if ((now - lastDebounceTime) > debounceDelay && reading != buttonState)
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, and the button state has changed
    buttonState = reading;
    if (buttonState == LOW)
    {
      go = !go;       // change system state to go
//      Serial.print("GO: ");
//      Serial.println(go);
//      integrateStart = true; // reset integration on PID controller
//      if (!go && homed) dir = -dir; // reverse directions if motion stopped with button, and homed
    }
  }
  lastButtonState = reading; // save the reading. Next time through the loop, it'll be the lastButtonState

  // Update temperature readings
  if (now - lastTime >= sampleTime)
  {
    temp1total -= temp1readings[readIndex];
    temp2total -= temp2readings[readIndex];
  
    int analogRead1 = analogRead(thermistorPin1); //read the analog pin (raw 0 to 1023)
    int analogRead2 = analogRead(thermistorPin2);
    float V1 = analogRead1 * (Vref / 1024.0); // convert the analog reading to voltage [V]
    float V2 = analogRead2 * (Vref / 1024.0);
    float R1 = V1*Rs / (Vref - V1); // convert the voltage to the thermistor resistance [Ohm]
    float R2 = V2*Rs / (Vref - V2);
    temp1readings[readIndex] = B / log(R1/ry); // covert the resistance to a temperature reading [K]
    temp2readings[readIndex] = B / log(R2/ry);
  
    temp1total += temp1readings[readIndex];
    temp2total += temp2readings[readIndex];
  
    readIndex += 1;
    if (readIndex >= numReadings) readIndex = 0;
  
    temp1 = temp1total / numReadings;
    temp2 = temp2total / numReadings;

    lastTime = now;
  }
}

//void setPosition(int newPosition){
//  voltageDriver(0);
//  encoder.write(newPosition);
//  while (currentSpeed != 0){
//    updateSensors();
//    delay(1);
//  }
//  encoder.write(newPosition);
//  updateSensors();
//}

//void stopNow(){
//  Serial.println("STOPPING");
//  int8_t stopDir = sgn(currentSpeed);
//  unsigned int startTime = millis();
//  int startPosition = currentPosition;
//  if (stopDir == 1) startPosition += 1.4*currentSpeed*sampleTime; // add fudge factor to make stopping smooth
//  else startPosition += 1.0*currentSpeed*sampleTime;
//  float startSpeed = abs(currentSpeed);
//  int distance = accel * (startSpeed/accel) * (startSpeed/accel) / 2;
//  int endPosition = startPosition + stopDir*distance;
//  int posSetpoint = startPosition; // initialize setpoint [pulses]
//  float milliVolts;
//  unsigned int now = startTime;
//  unsigned int lastTime = now;
//  integrateStart = true;
//  while (abs(posSetpoint - endPosition) > error){
//    if (now - lastTime >= sampleTime){
//      int deltaT = now - startTime; // calculate the time change from begginning of stop
//      posSetpoint = startPosition + stopDir*(startSpeed*deltaT - accel*deltaT*deltaT/2); // [pulses]
//      milliVolts = computePID(posSetpoint, currentPosition);
//      voltageDriver(milliVolts);
//      // save static variables for next round
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  while (currentSpeed != 0) {
//    if (now - lastTime >= sampleTime){
//      milliVolts = computePID(endPosition, currentPosition);
//      voltageDriver(milliVolts);
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  voltageDriver(0);
//  Serial.println("DONE STOPPING");
//}

//void homeNow(){
//  Serial.println("HOMING");
//  if (currentSpeed != 0) stopNow();
//  setPosition(0);
//  buildProfile();
//  printProfile();
//  int posSetpoint;
//  float milliVolts;
//  dir = -1;
//  integrateStart = true;
//  unsigned int now = millis();
//  unsigned int lastTime = now - sampleTime;
//  while (!limitSwitch && go){
//    if (now - lastTime >= sampleTime){
//      posSetpoint = integrateProfile(); // [pulses]
//      milliVolts = computePID(posSetpoint, currentPosition);
//      voltageDriver(milliVolts);
//      // save static variables for next round
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  if (currentSpeed != 0) stopNow();
//  now = millis();
//  lastTime = now - sampleTime;  
//  posSetpoint = currentPosition;
//  integrateStart = true;
//  while (limitSwitch && go){
//    if (now - lastTime >= sampleTime){
//      posSetpoint += homeStep;
//      milliVolts = computePID(posSetpoint, currentPosition);
//      voltageDriver(milliVolts);
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  unsigned int startTime = now;
//  while ((now - startTime) < limitTime && go){
//    if (now - lastTime >= sampleTime){
//      posSetpoint += homeStep;
//      milliVolts = computePID(posSetpoint, currentPosition);
//      voltageDriver(milliVolts);
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  voltageDriver(0);
//  lastTime = now - sampleTime;
//  integrateStart = true;
//  while (!limitSwitch && go){
//    if (now - lastTime >= sampleTime){
//      posSetpoint -= homeStep;
//      milliVolts = computePID(posSetpoint, currentPosition);
//      voltageDriver(milliVolts);
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  startTime = now;
//  while ((now - startTime) < limitTime && go){
//    if (now - lastTime >= sampleTime){
//      posSetpoint -= homeStep;
//      milliVolts = computePID(posSetpoint, currentPosition);
//      voltageDriver(milliVolts);
//      lastTime = now;
//    }
//    updateSensors();
//    now = millis();
//  }
//  voltageDriver(0);
//  if (limitSwitch && go) {
//    setPosition(0);
//    homed = true;
//  }
//  go = false;
//  dir = 1;
//  lastDir = -1;
//  Serial.println("DONE HOMING");
//}

void serialPrint(float setpoint, float temp1, float volts1, float temp2, float volts2, long timer)
{
//  unsigned long printTime = millis();
//  Serial.print(printTime); //Time [ms], Setpoint [F], Temp1 [F], Volts [V]
//  Serial.print(", ");
//  Serial.print(setpoint * (9.0/5.0) - 459.67);
//  Serial.print(", ");
//  Serial.print(temp1 * (9.0/5.0) - 459.67);
//  Serial.print(", ");
//  Serial.print(volts1);
//  Serial.print(", ");
//  Serial.print(temp2 * (9.0/5.0) - 459.67);
//  Serial.print(", ");
//  Serial.println(volts2);
//  Serial.print("Current Position: ");
//  Serial.println(currentPosition);
//  Serial.print("Temp Change: ");
//  Serial.println(tempChange);

  
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0,16);
  display.print("TRGT [F]: ");
  display.setTextSize(2);
  display.println(setpoint * (9.0/5.0) - 459.67, 1);
  display.setTextSize(1);
  display.setCursor(0,36);
  display.print("T1: ");
  display.print(temp1 * (9.0/5.0) - 459.67, 1);
  display.print(", T2: ");
  display.println(temp2 * (9.0/5.0) - 459.67, 1);

  display.setCursor(0,56);
  display.print("V1: ");
  display.print(min(max(volts1, 0),voltRange), 1);
  display.print(", V2: ");
  display.println(min(max(volts2, 0),voltRange), 1);

  if (go)
  {
    timer = timer / 1000; // convert timer to seconds
    display.setCursor(0,0);
    display.print("Mode: MELTING ");
    display.print(timer / 60); // convert to minutes and print timer
    display.print(":");
    display.print(timer % 60); // calculate seconds left in last minute
  }
  else
  {
    display.setCursor(0,0);
    display.println("Mode: DIPPING");
  }
  
  display.display();

}

void loop()
{

  unsigned long meltTimerNow = millis();
  static unsigned long meltTimerLastTime = meltTimerNow - meltTimer;
  if (meltTimerNow - meltTimerLastTime >= meltTimer || go == false){
    go = false;
    meltTimerLastTime = meltTimerNow;
  }

//  //check if system thinks it is at 0 but is not at home
//  if (abs(currentPosition) < error && !limitSwitch && homed) {
//    homed = false;
//    Serial.println("HOME FAILED (OUTSIDE LIMIT)");
//  }
//
//  //check if system is at home and doesn't know it, and travelling downwards
//  if (limitSwitch && abs(currentPosition) > error && dir == -1 && homed) {
//    homed = false;
//    Serial.println("HOME FAILED (INSIDE LIMIT)");
//  }
//
//  //check if target has been reached
//  if (abs(currentPosition - profilePositions[3]) < error && currentSpeed == 0){
//    dir = -dir;
//    go = false;
//    Serial.println("TARGET REACHED");
//  }
//
//  //if the direction has changed, update the target and build the profile
//  if (dir != lastDir){
//    if (dir == 1) profilePositions[3] = stroke;
//    else profilePositions[3] = 0;
//    buildProfile();
//    lastDir = dir;
//    integrateStart = true;
//    Serial.println("PROFILE BUILT");
//    printProfile();
//  }
  
  updateSensors();

  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;
  if (go) {
    //Serial.println("MELTING");
    highTempSetpoint += tempChange / 4.0 / (9.0/5.0); // convert to F and add to setpoint
    if (now - lastTime >= sampleTime){
      float volts1 = heater1.computePID(highTempSetpoint, temp1);
      float volts2 = heater2.computePID(highTempSetpoint, temp2);
      voltageDriver(volts1, heaterPin1);
      voltageDriver(volts2, heaterPin2);
      // save static variables for next round
      lastTime = now;
      
      serialPrint(highTempSetpoint, temp1, volts1, temp2, volts2, (meltTimer - (meltTimerNow - meltTimerLastTime)));
      
    }
  }
  else {
    lowTempSetpoint += tempChange / 4.0 / (9.0/5.0); // convert to F and add to setpoint
    if (now - lastTime >= sampleTime){
      float volts1 = heater1.computePID(lowTempSetpoint, temp1);
      float volts2 = heater2.computePID(lowTempSetpoint, temp2);
      voltageDriver(volts1, heaterPin1);
      voltageDriver(volts2, heaterPin2);
      // save static variables for next round
      lastTime = now;

      serialPrint(lowTempSetpoint, temp1, volts1, temp2, volts2, (meltTimer - (meltTimerNow - meltTimerLastTime)));

//      Serial.println(volts2);
//      for (int i = 0; i < numReadings; i++) {
//        Serial.println(temp1readings[i] * (9.0/5.0) - 459.67);
//        Serial.println(temp2readings[i] * (9.0/5.0) - 459.67);
//      }
    }
  }
}
