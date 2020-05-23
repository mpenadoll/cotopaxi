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
  float output;
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
  
  float computePID(float setpoint, float feedback)
  {
    // update the time
    unsigned int now = millis();
    timeChange = now - lastTime;
    
    // update the error
    error = setpoint - feedback;
    
    if (abs(error) < 2.0) errSum += error * timeChange; // prevents integral from growing when error is large
    dErr = (error - lastErr) / (float)timeChange;
    
    // Save static variables for next round
    lastErr = error;
    lastTime = now;
    
    /*Compute PID Output*/
    output = pulseKp * error + pulseKi * errSum + pulseKd * dErr;
    
    return output;
  }
};
