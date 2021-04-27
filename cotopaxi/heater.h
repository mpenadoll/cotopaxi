/* Heater
The heater object includes all components for a heater:

Thermistor
 * computes the temperature reading for a thermistor input
 * running average to smooth reading
 * input: K
 * output: V

PIDcontroller
 * computes the PID controller output
 * converts the gains to the appropriate units
 * input: K
 * output: V


Voltage Driver

 */

class heater
{
  // class member varibales - thermistor
  float temp; // temperature reading of the thermistor [degK]
  const int numReadings = 4; // number of readings for moving average
  int readIndex; // index to update the readings
  float tempReadings[numReadings]; // for moving average
  float tempTotal; // for moving average
  const int thermistorPin; // analog pin for thermistor reading
  const float Vref = 5.0; // reference voltage from the Aref pin [V]
  const float m = -25.558; // linearization slope of therm temp [K / V]
  const float b = 399.733; // linearization y-intercept of therm temp [K]

  // class member variables - PID Loop
  float pulseKp, pulseKi, pulseKd;
  float error;
  float errSum;
  float lastErr;
  float dErr;
  unsigned int timeChange;
  unsigned int lastTime;
  float tempSetpoint;

  // class member variables - voltage driver
  const int heaterPin; // digital pin for heater PWM control
  const int voltRange = 120; // absolute range of the voltage output [mV]

  //CONSTRUCTOR
  public:
  thermistor (int thermPin, int heatPin, float Kp, float Ki, float Kd, float initialTemp)
  {
    thermistorPin = thermPin;
    pinMode(thermistorPin, INPUT);

    heaterPin = heatPin;
    pinMode(heaterPin, OUTPUT);

    // setup the temp readings
    for (int i = 0; i < numReadings; i++)
    {
      tempReadings[i] = 0;
    }

    // Initialize the PIDloop by calculating the constants in correct units
    pulseKp = Kp ; //proportional gain [V / K]
    pulseKi = Ki / 1000.0; //integral gain [V / (ms * K)]
    pulseKd = Kd * 1000.0; //derivative gain [V * ms / K]

    tempSetpoint = initialTemp; // set the initial temperature setpoint
  }

  // runs the heater, returns true if it ran correctly, returns false if there was a bad thermistor reading
  bool run()
  {
    //THERMISTOR
    tempTotal -= tempReadings[readIndex];
  
    int analogReading = analogRead(thermistor1pin); //read the analog pin (raw 0 to 1023)
    float V = analogReading * (Vref / 1024.0); // convert the analog reading to voltage [V]

    // if there is an open circuit, trip the error variable
    if (abs(V - Vref) < 0.1)
    {
      temp = 0;
      volts = 0;
      analogWrite(heaterPin, 0);
      return false;
    }

    tempReadings[readIndex] = m * V + b; // solve for linear temp [K]
    tempTotal += tempReadings[readIndex];
  
    readIndex += 1;
    if (readIndex >= numReadings) readIndex = 0;
  
    temp = tempTotal / numReadings;

    //PID LOOP
    unsigned int now = millis();
    timeChange = now - lastTime;
    
    // update the error
    error = tempSetpoint - temp;

    if (abs(error) < 3.0) errSum += error * timeChange; // prevents integral from growing when error is large
    dErr = (error - lastErr) / (float)timeChange;
    
    // Save static variables for next round
    lastErr = error;
    lastTime = now;

    volts = pulseKp * error + pulseKi * errSum + pulseKd * dErr;

    //VOLTAGE DRIVER
    volts = constrain(volts, 0, voltRange);

    if (volts <= 0)
    {
      analogWrite(heaterPin, 0);
    }
    else
    {
      analogWrite(heaterPin, map(abs(volts),0,voltRange,0,255));
    }

    return true;
  }

  // change the temperature setpoint, and return the new setpoint
  float changeTarget(float tempChange)
  {
    tempSetpoint += tempChange / 4.0 / (9.0/5.0); // convert to F and add to setpoint
    return tempSetpoint;
  }

  // get the current temperature setpoint
  float getTarget()
  {
    return tempSetpoint;
  }

  // return the actual temperature reading
  float getTemp()
  {
    return temp;
  }

  // return the current voltage drive
  float getVolts()
  {
    return volts;
  }
};