volatile long count = 0;
boolean A,B;
unsigned long timep, time, etime;
byte state, statep;
#define pwm  11 
#define dir1 4 
#define dir2 5 

int speed_req = 120;
int speed_actual = 0;
double Kp = 0.5;
double Kd = -1;
int PWM_val = 0;
unsigned long lastTime = 0;
unsigned long lastTime_print = 0;
#define LOOPTIME 100

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(0,Achange,CHANGE);
  attachInterrupt(1,Bchange,CHANGE);
  timep = millis(); 
  A = digitalRead(2);
  B = digitalRead(3);
  if ((A==HIGH)&&(B==HIGH)) statep = 1;
  if ((A==HIGH)&&(B==LOW)) statep = 2; 
  if ((A==LOW)&&(B==LOW)) statep = 3;
  if((A==LOW)&&(B==HIGH)) statep = 4;
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm, OUTPUT);
}

void loop() {
  if((millis() - lastTime) >= LOOPTIME) {
    lastTime = millis(); 
    findMotorData();
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm, PWM_val);
    }
   printMotorInfo();
}

void findMotorData() {
  static long count_prev = 0;
  speed_actual = (count - count_prev)*60*(1000/LOOPTIME)/4000;
  count_prev = count;
  
  float pidTerm = 0;
  int error = 0;
  int last_error = 0;
  error = abs(speed_req) - abs(speed_actual);
  pidTerm = (Kp * error) + (Kd * (error - last_error));
  last_error = error;
  PWM_val = constrain(pidTerm, 0, 255);
}

void printMotorInfo() {
  if((millis() - lastTime_print) >= 500) {
    lastTime_print = millis();
    Serial.print("Setpoint: ");    Serial.println(speed_req);
    Serial.print("Speed RPM: ");    Serial.println(speed_actual);
  }
}

void Achange() 
{
  A = digitalRead(2);
  B = digitalRead(3);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
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

void Bchange()
{
  A = digitalRead(2);
  B = digitalRead(3);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
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
