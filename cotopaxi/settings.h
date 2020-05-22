//Settings file for the spice rack and motor

// Motion Profile Variables
//float strokeMM = 500.0;     // stroke [mm] 431.8
//float pulleyRadius = 24.41;  // radius of the pulley [mm]
//float maxSpeedMM = 80.0;     // max speed of rack [mm/s]
//float homeSpeedMM = 30.0; // homing speed [mm/s]
//float accelMM = 150.0;        // acceleration of rack [mm/s^2]
//float PPR = 1440.0;  // number of pulses of encoder per rev

// Set PID Controller Settings for Position Control
float Kp = 8.0; // proportional gain [V / K]
float Ki = 0.07; // integral gain [V / (K*s)]
float Kd = 20.0; // derivative gain [V * s / K]
//float pulseKp, pulseKi, pulseKd; // pulse conversion declarations

// CONSTANTS
const unsigned int sampleTime = 1000; //sample time for derivative measurements [ms]
const unsigned int debounceDelay = 50;  // the debounce time; increase if the output flickers
//const int error = 5; // error [pulses] allowable for position control
//const unsigned int limitTime = 300; // time to move into the limit switch [ms]
//const int homeStep = 3; // distance to travel each homing step/loop
const int voltRange = 110; // absolute range of the voltage output [mV]
const float Vref = 5.0; // reference voltage from the Aref pin [V]
const float B = 3435.0; // Beta value of the thermistors
const float R0 = 10000.0; // resistance value of the thermistor at 25degC
const float ry = R0*exp(-B/298.15); // r @ infinity
const float Rs = 2200.0; // resistance of series resistor in voltage divider
float lowTempSetpoint = 341.483; // 342.0 degK = 156 degF
float highTempSetpoint = 355.372; // 355.4 degK = 180 degF
unsigned long meltTimer = 60.0 * 60000.0; // time that the melting cycle takes (when the button is clicked) [minutes] * [ms / min] = [ms]

// PINS
//const int dirPin = 4;  //pin to enable (high) driver
//const int PWMpin = 10; //pin to set pwm on driver for up
const int buttonPin = 13;  //pushbutton signal in
//const int limitSwitchPin = 9; //limitSwitch signal in
const int encoderApin = 2;  //Best Performance: both pins have interrupt capability
const int encoderBpin = 3;  //Best Performance: both pins have interrupt capability
const int thermistorPin1 = A0; //Analog Read pin for the Thermistor (Voltage Divider)
const int thermistorPin2 = A1; //Analog Read pin for the second Thermistor (Voltage Divider)
const int heaterPin1 = 9; //PWM pin for the first heater, goes to mosfet gate
const int heaterPin2 = 10; //PWM pin for the second heater, goes to mosfet gate

// Calculate variables in units of enconder pulses. Note - gear ratio was removed
//float stroke = PPR * 2 * strokeMM / (pulleyRadius * 2 * 3.14);  // stroke [pulses]
//float maxSpeed = PPR * 2 * maxSpeedMM / (pulleyRadius * 2 * 3.14 * 1000); // max speed [pulses/ms]
//float homeSpeed = PPR * 2 * homeSpeedMM / (pulleyRadius * 2 * 3.14 * 1000); // max speed [pulses/ms]
//float accel = PPR * 2 * accelMM / (pulleyRadius * 2 * 3.14 * 1000 * 1000);  // acceleration [pulses/ms^2]
//float accelTime = maxSpeed / accel; //time to accelerate and decelerate from stop [ms]
//float homeAccelTime = homeSpeed / accel;
//float accelDistance = 0.5 * accel * accelTime * accelTime; //the minimum distance to accelerate to max speed [pulses]
//float homeAccelDistance = 0.5 * accel * homeAccelTime * homeAccelTime;
