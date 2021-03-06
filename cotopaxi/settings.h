//Settings file for the spice rack and motor

// Set PID Controller Settings for Position Control
const float Kp = 15.0; // proportional gain [V / K]
const float Ki = 0.02; // integral gain [V / (K*s)]
const float Kd = 15.0; // derivative gain [V * s / K]

// CONSTANTS
const unsigned int sampleTime = 150; //sample time for derivative measurements [ms]
const unsigned int debounceDelay = 50;  // the debounce time; increase if the output flickers
const int voltRange = 120; // absolute range of the voltage output [mV]
const float Vref = 5.0; // reference voltage from the Aref pin [V]
const float m = -25.558; // linearization slope of therm temp [K / V]
const float b = 399.733; // linearization y-intercept of therm temp [K]
float tempSetpoint = (160 + 459.67) * 5.0/9.0; // temp for dipping [F to K]

// PINS
const int encoderApin = 2;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 3;  //Best Performance: both pins have interrupt capability
const int thermistorPin = A2; //Analog Read pin for the Thermistor (Voltage Divider)
const int heaterPin = 9; //PWM pin for the first heater, goes to mosfet gate
