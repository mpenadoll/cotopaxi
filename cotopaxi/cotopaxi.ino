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
int buttonState = HIGH; // the current reading from the input pin
int currentPosition; //the current position [pulses]
int tempChange; // the amount to change the target temp by [K]
float temp1; // temperature reading of the thermistor [degK]
float temp2;
const int numReadings = 4; // number of readings for moving average
int readIndex; // index to update the readings
float temp1readings[numReadings]; // for moving average
float temp2readings[numReadings];
float temp1total; // for moving average
float temp2total;

void setup()
{
//  Serial.begin(9600);
  
  // SSD1306 Init
  display.begin();  // Switch OLED
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(F("Hey, Hot Stuff!"));
  display.display();

  delay(2000);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Mystic Melter v0.3.0"));
  display.display();

  delay(2000);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(heaterPin1, OUTPUT);
  pinMode(heaterPin2, OUTPUT);

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

//  Serial.println("READY");
//  Serial.println("Time [ms], Setpoint [F], Temp1 [F], Volts [V]");
}

void voltageDriver(float volts, int PWMpin) {
  volts = constrain(volts, 0, voltRange);
  if (volts <= 0){
    analogWrite(PWMpin, 0);
  }
  else {
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
  // update button, encoder knob, and thermistors
  
  int reading = digitalRead(buttonPin);  // read the state of the switch into a local variable

  // read encoder and calculate the speed
  int newPosition = encoder.read();
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
//    float R1 = V1*Rs / (Vref - V1); // convert the voltage to the thermistor resistance [Ohm]
//    float R2 = V2*Rs / (Vref - V2);
//    temp1readings[readIndex] = B / log(R1/ry); // covert the resistance to a temperature reading [K]
//    temp2readings[readIndex] = B / log(R2/ry);
    temp1readings[readIndex] = m * V1 + b; // solve for linear temp [K]
    temp2readings[readIndex] = m * V2 + b;
  
    temp1total += temp1readings[readIndex];
    temp2total += temp2readings[readIndex];
  
    readIndex += 1;
    if (readIndex >= numReadings) readIndex = 0;
  
    temp1 = temp1total / numReadings;
    temp2 = temp2total / numReadings;

    lastTime = now;
  }
}

void displayPrint(float setpoint, float temp1, float volts1, float temp2, float volts2, long timer)
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
  display.print(F("TRGT [F]: "));
  display.setTextSize(2);
  display.println(setpoint * (9.0/5.0) - 459.67, 1);
  display.setTextSize(1);
  display.setCursor(0,36);
  display.print(F("T1: "));
  display.print(temp1 * (9.0/5.0) - 459.67, 1);
  display.print(F(", T2: "));
  display.println(temp2 * (9.0/5.0) - 459.67, 1);

  display.setCursor(0,56);
  display.print(F("V1: "));
  display.print(min(max(volts1, 0),voltRange), 1);
  display.print(F(", V2: "));
  display.println(min(max(volts2, 0),voltRange), 1);

  if (go)
  {
    timer = timer / 1000; // convert timer to seconds
    display.setCursor(0,0);
    display.print(F("Mode: DROPPIN' SLUGS "));
    display.setCursor(100,8);
    display.print(timer / 60); // convert to minutes and print timer
    display.print(F(":"));
    display.print(timer % 60); // calculate seconds left in last minute
  }
  else
  {
    display.setCursor(0,0);
    display.println(F("Mode: DIPPING"));
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
      
      displayPrint(highTempSetpoint, temp1, volts1, temp2, volts2, (meltTimer - (meltTimerNow - meltTimerLastTime)));
      
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

      displayPrint(lowTempSetpoint, temp1, volts1, temp2, volts2, (meltTimer - (meltTimerNow - meltTimerLastTime)));

//      Serial.println(volts2);
//      for (int i = 0; i < numReadings; i++) {
//        Serial.println(temp1readings[i] * (9.0/5.0) - 459.67);
//        Serial.println(temp2readings[i] * (9.0/5.0) - 459.67);
//      }
    }
  }
}
