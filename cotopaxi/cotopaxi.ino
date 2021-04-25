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
PIDloop heater(Kp, Ki, Kd);

// Initialize Encoder (for the Knob)
Encoder encoder(encoderApin, encoderBpin);

// VARIABLES
bool go = false; // the state of the drive system (go or stop)
int buttonState = HIGH; // the current reading from the input pin
int currentPosition; //the current position [pulses]
int tempChange; // the amount to change the target temp by [K]
float temp; // temperature reading of the thermistor [degK]
const int numReadings = 4; // number of readings for moving average
int readIndex; // index to update the readings
float tempReadings[numReadings]; // for moving average
float tempTotal; // for moving average
bool error = false; // error for broken thermistors

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

  delay(3000);
  
  pinMode(heaterPin, OUTPUT);
  pinMode(thermistorPin, INPUT);

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
   
//  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  // setup the temp readings
  for (int i = 0; i < numReadings; i++) {
    tempReadings[i] = 0;
  }
  
  updateSensors();

  Serial.println("READY");
//  Serial.println("Time [ms], Setpoint [F], Temp1 [F], Volts [V]");
}

void voltageDriver(int volts, int PWMpin) {
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
  // update encoder knob, and thermistor

  // read encoder and calculate the speed
  int newPosition = encoder.read();
  tempChange = newPosition - currentPosition;
  currentPosition = newPosition;

  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;

  // Update temperature readings
  if (now - lastTime >= sampleTime)
  {
    tempTotal -= tempReadings[readIndex];
  
    int analogReading = analogRead(thermistorPin); //read the analog pin (raw 0 to 1023)
    float V = analogReading * (Vref / 1024.0); // convert the analog reading to voltage [V]

    // if there is an open circuit, trip the error variable
    if (abs(V - Vref) < 0.1) error = true;
    else error = false;

    tempReadings[readIndex] = m * V + b; // solve for linear temp [K]
    tempTotal += tempReadings[readIndex];
  
    readIndex += 1;
    if (readIndex >= numReadings) readIndex = 0;
  
    temp = tempTotal / numReadings;

    lastTime = now;
  }
}

void displayPrint(float setpoint, float temp, float volts)
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
  display.print(F("TEMP: "));
  display.print(temp * (9.0/5.0) - 459.67, 1);

  display.setCursor(0,56);
  display.print(F("V: "));
  display.print(min(max(volts, 0),voltRange), 1);

  if (error)
  {
    display.setCursor(0,0);
    display.println(F("Mode: ERROR"));
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
  
  updateSensors();

  if (error)
  {
    voltageDriver(0, heaterPin);

    displayPrint((0 + 459.67) * 5.0/9.0, temp, 0);
  }

  else
  {
    unsigned int now = millis();
    static unsigned int lastTime = now - sampleTime;
    tempSetpoint += tempChange / 4.0 / (9.0/5.0); // convert to F and add to setpoint
    if (now - lastTime >= sampleTime){

      int volts = heater.computePID(tempSetpoint, temp);
      voltageDriver(volts, heaterPin);

      // save static variables for next round
      lastTime = now;

      displayPrint(tempSetpoint, temp, volts);

//      Serial.println(volts2);
//      for (int i = 0; i < numReadings; i++) {
//        Serial.println(temp1readings[i] * (9.0/5.0) - 459.67);
//        Serial.println(temp2readings[i] * (9.0/5.0) - 459.67);
//      }
    }
  }
}
