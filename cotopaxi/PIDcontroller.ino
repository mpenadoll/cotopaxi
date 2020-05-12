/* PIDcontroller
 * computes the PID controller output in milliVolts
 * converts the gains to the appropriate units
 */

float computePID(int setpoint, int feedback){
  
  unsigned int now = millis();

  /*Compute all the working error variables*/
  int error = setpoint - feedback;
  static unsigned int lastTime = now - 1;
  static float errSum = 0;
  static int lastErr = error;
  if (integrateStart) {
    lastTime = now - 1;
    errSum = 0;
    lastErr = error;
    integrateStart = false;
  }
  unsigned int timeChange = now - lastTime;
  errSum += (float)error * timeChange;
  float dErr = (error - lastErr) / (float)timeChange;
  
  // Save static variables for next round
  lastErr = error;
  lastTime = now;
  
  /*Compute PID Output (float math, but returns int)*/
  float output = pulseKp * error + pulseKi * errSum + pulseKd * dErr;
  
  return output;
}

// Convert the gains to the correct units
void setGains(){  
  pulseKp = Kp ; //proportional gain [mV / degK]
  pulseKi = Ki ; //integral gain [mV / (ms * degK)]
  pulseKd = Kd ; //derivative gain [mV * ms / degK]
}
