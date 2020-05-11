void twist_to_cmd_RPM( const geometry_msgs::Twist& cmd_msg) {
  double linear_x  = cmd_msg.linear.x;
  double angular_z = cmd_msg.angular.z;
  double linear_y  = cmd_msg.angular.y;  // zero
  
if (angular_z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = linear_x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (linear_x == 0) {
    // convert rad/s to rpm
    rpm_req2 = angular_z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = - rpm_req2;
  }
  else {
    rpm_req1 = linear_x*60/(pi*wheel_diameter) - angular_z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = linear_x*60/(pi*wheel_diameter) + angular_z*track_width*60/(wheel_diameter*pi*2);
  }
   rpm.motor1 = constrain(rpm_req1, -max_rpm_, max_rpm_);
   rpm.motor2 = constrain(rpm_req2, -max_rpm_, max_rpm_);
   return rpm;
}

cmd_twist_VEL(int rpm1, int rpm2)
 {
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    average_rps_x = ((float)(rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s
    
    average_rps_y = ((float)(-rpm1 + rpm2 ) / 2) / 60; // RPM
    vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 ) / total_wheels_) / 60;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s
    return vel;
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", twist_to_cmd_RPM);

geometry_msgs::Vector3Stamped rpm_msg;

ros::Publisher rpm_pub("rpm", &rpm_msg);

ros::Time current_time;

ros::Time last_time;

void setup() {
 AFMS.begin();  // create with the default frequency 1.6KHz
 /* count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 */
 
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);
  
 //pinMode(encodPinA1, INPUT); 
// pinMode(encodPinB1, INPUT); 
// digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
// digitalWrite(encodPinB1, HIGH);
 attachInterrupt(1, encoder1, RISING);

// pinMode(encodPinA2, INPUT); 
// pinMode(encodPinB2, INPUT); 
// digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
// digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, encoder2, RISING);
// motor1->setSpeed(0);
// motor2->setSpeed(0);
// motor1->run(FORWARD);
// motor1->run(RELEASE);
// motor2->run(FORWARD);
// motor2->run(RELEASE);
}

void loop() {

  runROS()
  /* nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);
    PWM_val1 = computePid( PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = computePid( PWM_val2, rpm_req2, rpm_act2);

    if(PWM_val1 > 0) direction1 = FORWARD;
    else if(PWM_val1 < 0) direction1 = BACKWARD;
    if (rpm_req1 == 0) direction1 = RELEASE;
    if(PWM_val2 > 0) direction2 = FORWARD;
    else if(PWM_val2 < 0) direction2 = BACKWARD;
    if (rpm_req2 == 0) direction2 = RELEASE;
    motor1->run(direction1);
    motor2->run(direction2);

    motor1->setSpeed(abs(PWM_val1));
    motor2->setSpeed(abs(PWM_val2));
    
    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }

*/
  
}


runROS()
{

//this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

 //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

  
}
void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}
/*
int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4095.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}
*/

int computePid(int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error = 0;
  static double int_error = 0;

  
  error = targetValue-currentValue;
    int_error += error;
    pidTerm = Kp*error + Kd*(error-last_error) + Ki*int_error;
    last_erro1 = error;
  
  new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4095.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}


void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

/*
void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
*/
void encoder(int pinA , int pinB) 
{

  A = digitalRead(pinA);
  B = digitalRead(pinB);

  if ((A==HIGH)&&(B==HIGH)) 
        state = 1;
  if ((A==HIGH)&&(B==LOW)) 
       state = 2;
  if ((A==LOW)&&(B==LOW)) 
       state = 3;
  if((A==LOW)&&(B==HIGH)) 
       state = 4;
       
  switch (state)
  {
    case 1:
    {
      if (statep == 2) count++;
      if (statep == 4) count--;
      break;
    }
    case 2:
    {
      if (statep == 1) count--;
      if (statep == 3) count++;
      break;
    }
    case 3:
    {
      if (statep == 2) count --;
      if (statep == 4) count ++;
      break;
    }
    default:
    {
      if (statep == 1) count++;
      if (statep == 3) count--;
    }
  }
  statep = state;

  
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}
