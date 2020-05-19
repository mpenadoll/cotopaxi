/* PIDcontroller
 * computes the PID controller output
 * converts the gains to the appropriate units
 * input: K
 * output: V
 */

float computePID(float setpoint, float feedback){
  
  unsigned int now = millis();
  static unsigned int lastTime = now - 1;
  
  /*Compute all the working error variables*/
  float error = setpoint - feedback;
  static float errSum = 0;
  static float lastErr = error;
  if (integrateStart) {
    lastTime = now - 1;
    errSum = 0;
    lastErr = error;
    integrateStart = false;
  }
  unsigned int timeChange = now - lastTime;
  if (abs(error) < 2.0) errSum += error * timeChange; // prevents integral from growing when error is large
  float dErr = (error - lastErr) / (float)timeChange;
  
  // Save static variables for next round
  lastErr = error;
  lastTime = now;
  
  /*Compute PID Output*/
  float output = pulseKp * error + pulseKi * errSum + pulseKd * dErr;

//  Serial.print("Kp: ");
//  Serial.print(pulseKp * error);
//  Serial.print("
  
  return output;
}

// Convert the gains to the correct units
void setGains(){  
  pulseKp = Kp ; //proportional gain [V / K]
  pulseKi = Ki / 1000.0; //integral gain [V / (ms * K)]
  pulseKd = Kd * 1000.0; //derivative gain [V * ms / K]
}
