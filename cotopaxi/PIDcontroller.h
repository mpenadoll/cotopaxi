/* PIDcontroller
 * computes the PID controller output
 * converts the gains to the appropriate units
 * input: K
 * output: V
 */

class PIDloop
{
  // Class member variables
  float pulseKp, pulseKi, pulseKd;
  float error;
  float errSum;
  float lastErr;
  float dErr;
  unsigned int timeChange;
  unsigned int lastTime;
//  float output;
//  bool integrateStart;
  
  // Constructor 
  public:
  PIDloop(float Kp, float Ki, float Kd)
  {
    // Initialize the PIDloop by calculating the constants in correct units
    pulseKp = Kp ; //proportional gain [V / K]
    pulseKi = Ki / 1000.0; //integral gain [V / (ms * K)]
    pulseKd = Kd * 1000.0; //derivative gain [V * ms / K]
  }
  
  int computePID(float setpoint, float feedback)
  {
    // update the time
    unsigned int now = millis();
    timeChange = now - lastTime;
    
    // update the error
    error = setpoint - feedback;

    if (abs(error) < 3.0) errSum += error * timeChange; // prevents integral from growing when error is large
//    if (error < 2.0 && error > 0) errSum += error * timeChange; // prevents integral from growing when error is large or negative
//    else errSum = 0; // unless you are in the integral window, reset errSum to 0
    dErr = (error - lastErr) / (float)timeChange;
    
    // Save static variables for next round
    lastErr = error;
    lastTime = now;
    
    /*Compute PID Output*/
//    output = pulseKp * error + pulseKi * errSum + pulseKd * dErr;
//    static unsigned int lastPrintTime = now;
//    if (now - lastPrintTime > 1000)
//    {
//      Serial.print("error: ");
//      Serial.println(error);
//      Serial.print("errSum: ");
//      Serial.println(errSum);
//      Serial.print("dErr: ");
//      Serial.println(dErr);
//      Serial.print("output: ");
//      Serial.println(pulseKp * error + pulseKi * errSum + pulseKd * dErr);
//      lastPrintTime = now;
//    }
//    
    return pulseKp * error + pulseKi * errSum + pulseKd * dErr;
  }
};
