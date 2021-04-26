/* Thermistor
 * computes the temperature reading for a thermistor input
 * running average to smooth reading
 * input: K
 * output: V
 */

class thermistor
{
  // class member varibales
  int tempChange; // the amount to change the target temp by [K]
  float temp; // temperature reading of the thermistor [degK]
  const int numReadings = 4; // number of readings for moving average
  int readIndex; // index to update the readings
  float tempReadings[numReadings]; // for moving average
  float tempTotal; // for moving average

  // constructor
  public:
  thermistor (int pin)
  {
    thermistorPin = pin;
    pinMode(thermistorPin, INPUT);

    // setup the temp readings
    for (int i = 0; i < numReadings; i++) {
      tempReadings[i] = 0;
    }
  }

  float getTemp()
  {
    tempTotal -= tempReadings[readIndex];
  
    int analogReading = analogRead(thermistor1pin); //read the analog pin (raw 0 to 1023)
    float V = analogReading * (Vref / 1024.0); // convert the analog reading to voltage [V]

    // if there is an open circuit, trip the error variable
    if (abs(V - Vref) < 0.1) error = true;
    else error = false;

    tempReadings[readIndex] = m * V + b; // solve for linear temp [K]
    tempTotal += tempReadings[readIndex];
  
    readIndex += 1;
    if (readIndex >= numReadings) readIndex = 0;
  
    temp = tempTotal / numReadings;

    return temp;
  }
};
