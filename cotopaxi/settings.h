//Settings file for the spice rack and motor

// Set PID Controller Settings for Position Control
const float Kp = 8.0; // proportional gain [V / K]
const float Ki = 0.07; // integral gain [V / (K*s)]
const float Kd = 20.0; // derivative gain [V * s / K]

// CONSTANTS
const unsigned int sampleTime = 200; //sample time for derivative measurements [ms]
const unsigned int debounceDelay = 50;  // the debounce time; increase if the output flickers
const int voltRange = 110; // absolute range of the voltage output [mV]
const float Vref = 5.0; // reference voltage from the Aref pin [V]
const float B = 3435.0; // Beta value of the thermistors
const float R0 = 10000.0; // resistance value of the thermistor at 25degC
const float ry = R0*exp(-B/298.15); // r @ infinity
const float Rs = 2200.0; // resistance of series resistor in voltage divider
float lowTempSetpoint = 341.483; // 342.0 degK = 156 degF
float highTempSetpoint = 355.372; // 355.4 degK = 180 degF
const unsigned long meltTimer = 60.0 * 60000.0; // time that the melting cycle takes (when the button is clicked) [minutes] * [ms / min] = [ms]

// PINS
const int buttonPin = 13;  //pushbutton signal in
const int encoderApin = 2;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 3;  //Best Performance: both pins have interrupt capability
const int thermistorPin1 = A0; //Analog Read pin for the Thermistor (Voltage Divider)
const int thermistorPin2 = A1; //Analog Read pin for the second Thermistor (Voltage Divider)
const int heaterPin1 = 9; //PWM pin for the first heater, goes to mosfet gate
const int heaterPin2 = 10; //PWM pin for the second heater, goes to mosfet gate
