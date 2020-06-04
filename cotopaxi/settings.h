//Settings file for the spice rack and motor

// Set PID Controller Settings for Position Control
const float Kp = 8.0; // proportional gain [V / K]
const float Ki = 0.07; // integral gain [V / (K*s)]
const float Kd = 20.0; // derivative gain [V * s / K]

// CONSTANTS
const unsigned int sampleTime = 150; //sample time for derivative measurements [ms]
const unsigned int debounceDelay = 50;  // the debounce time; increase if the output flickers
const int voltRange = 110; // absolute range of the voltage output [mV]
const float Vref = 5.0; // reference voltage from the Aref pin [V]
//const float B = 3977.0; // Beta value of the thermistors
//const float R0 = 10000.0; // resistance value of the thermistor at 25degC
//const float ry = R0*exp(-B/298.15); // r @ infinity
//const float Rs = 2200.0; // resistance of series resistor in voltage divider
const float m = -25.558; // linearization slope of therm temp [K / V]
const float b = 399.733; // linearization y-intercept of therm temp [K]
float lowTempSetpoint = (155 + 459.67) * 5.0/9.0; // temp for dipping [F to K]
float highTempSetpoint = (200 + 459.67) * 5.0/9.0; // temp for dropping slugs [F to K]
const unsigned long meltTimer = 5.0 * 60000.0; // time that the melting cycle takes (when the button is clicked) [minutes] * [ms / min] = [ms]

// PINS
const int buttonPin = 5;  //pushbutton signal in
const int encoderApin = 2;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 3;  //Best Performance: both pins have interrupt capability
const int thermistorPin1 = A2; //Analog Read pin for the Thermistor (Voltage Divider)
const int thermistorPin2 = A3; //Analog Read pin for the second Thermistor (Voltage Divider)
const int heaterPin1 = 9; //PWM pin for the first heater, goes to mosfet gate
const int heaterPin2 = 10; //PWM pin for the second heater, goes to mosfet gate
