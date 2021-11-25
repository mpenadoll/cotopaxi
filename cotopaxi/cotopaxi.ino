// Spice Rack Controller
#include "settings.h"
#include "heater.h"
#include "button.h"
#include <Encoder.h>

// Import required libraries for display
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

// Pin definitions
#define OLED_RESET  16  // Pin 15 -RESET digital signal

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

ArducamSSD1306 display(OLED_RESET); // FOR I2C

// Initialize the PIDloops for the heaters
heater heater1(thermistor1pin, heater1pin, Kp, Ki, Kd, tempSetpoint);
heater heater2(thermistor2pin, heater2pin, Kp, Ki, Kd, tempSetpoint);

// Initialize Encoder (for the Knob)
Encoder encoder(encoderApin, encoderBpin);

// Initialize momentary button for alternating control
toggleButton indexButton(buttonPin, debounceDelay);
bool toggle = false; // toggle for alternating through heater adjustment

// VARIABLES
int currentPosition; //the current position [pulses]
int tempChange; // the amount to change the target temp by [K]
int heaterIndex = 0; // heater index to cycle knob for setting temp
const int numHeaters = 2; // number of heaters

void setup()
{
//  Serial.begin(9600);
  
  // SSD1306 Init
  display.begin();  // Switch OLED
  display.clearDisplay(); // Clear the buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(F("Good Morning, Mary!"));
  display.display();

  delay(3000);

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
   
//  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  
  updateKnob();

  Serial.println("READY");
//  Serial.println("Time [ms], Setpoint [F], Temp1 [F], Volts [V]");
}

static inline int8_t sgn(float val)
{
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void updateKnob()
{
  // update encoder knob for temperature change command
  int newPosition = encoder.read();
  if (abs(newPosition - currentPosition) > 3) 
  {
    tempChange = (newPosition - currentPosition)/3;
    currentPosition = newPosition;
  }
  else tempChange = 0;

  if (indexButton.updateButton(toggle))
  {
    heaterIndex += 1;
    if (heaterIndex >= numHeaters) heaterIndex = 0;
    toggle = false;
  }
}

void loop()
{
  updateKnob();

  // if (heaterIndex == 0) heater1.changeTarget(tempChange);
  // else heater2.changeTarget(tempChange);
  // Linked Setpoints
  heater1.changeTarget(tempChange);
  heater2.changeTarget(tempChange);

  unsigned int now = millis();
  static unsigned int lastTime = now - sampleTime;

  if (now - lastTime >= sampleTime)
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    // if (heaterIndex == 0)
    // {
    //   display.print(F("HEATER 1"));
    //   display.setCursor(60,0);
    //   display.println(F("|HEATER 2"));
    // }
    // else
    // {
    //   display.print(F("HEATER 1"));
    //   display.setCursor(60,0);
    //   display.println(F("|HEATER 2"));
    // }
    display.print(F("HEATER 1"));
    display.setCursor(60,0);
    display.println(F("|HEATER 2"));
    display.println(F("          |"));
    display.println(F("          |"));
    display.println(F("          |"));
    display.println(F("          |"));
    display.println(F("          |"));
    display.println(F("          |"));
    display.println(F("          |"));

    display.setTextSize(2);
    display.setCursor(0,16);

    if (heater1.run()) display.println(heater1.getTarget() * (9.0/5.0) - 459.67, 1);
    else display.println(F("ERROR"));

    display.setTextSize(1);
    display.setCursor(0,36);
    display.print(F("TMP:"));
    display.print(heater1.getTemp() * (9.0/5.0) - 459.67, 1);

    display.setCursor(0,56);
    display.print(F("V: "));
    display.print(heater1.getVolts(), 1);
    

    display.setTextSize(2);
    display.setCursor(66,16);

    if (heater2.run()) display.println(heater2.getTarget() * (9.0/5.0) - 459.67, 1);
    else display.println(F("ERROR"));

    display.setTextSize(1);
    display.setCursor(66,36);
    display.print(F("TMP:"));
    display.print(heater2.getTemp() * (9.0/5.0) - 459.67, 1);

    display.setCursor(66,56);
    display.print(F("V: "));
    display.print(heater2.getVolts(), 1);

    display.display();
    
    // save static variables for next round
    lastTime = now;

//      Serial.println(volts2);
//      for (int i = 0; i < numReadings; i++) {
//        Serial.println(temp1readings[i] * (9.0/5.0) - 459.67);
//        Serial.println(temp2readings[i] * (9.0/5.0) - 459.67);
//      }
  }
}
