//Settings file for the spice rack and motor

// Set PID Controller Settings for Position Control
const float Kp = 15.0; // proportional gain [V / K]
const float Ki = 0.02; // integral gain [V / (K*s)]
const float Kd = 15.0; // derivative gain [V * s / K]

// CONSTANTS
const unsigned int sampleTime = 200; //sample time for derivative measurements [ms]
const unsigned int debounceDelay = 30;  // the debounce time; increase if the output flickers
float tempSetpoint = (165 + 459.67) * 5.0/9.0; // temp for dipping [F to K]
const int numReadings = 4; // number of readings for temperature moving average
const float Vref = 5.0; // reference voltage from the Aref pin [V]
const float m = -25.558; // linearization slope of therm temp [K / V]
const float b = 399.733; // linearization y-intercept of therm temp [K]
const int voltRange = 120; // absolute range of the voltage output [V]
const int voltMax = 120 * 0.5; // artificial max on the volts [V]

// PINS
const int encoderApin = 3;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 2;  //Best Performance: both pins have interrupt capability
const int thermistor1pin = A1; //Analog Read pin for the Thermistor (Voltage Divider)
const int thermistor2pin = A3; //
const int heater1pin = 9; //PWM pin for the first heater, goes to mosfet gate
const int heater2pin = 10;
const int buttonPin = 4; //pin for encoder momentary button

/*
OLED Display I2C pins
A4   SDA
A5   SCL
*/
