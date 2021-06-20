byte servoID1 = 0x01;
byte servoID2 = 0x02;

unsigned char f1_H, f1_L, f2_H, f2_L;
byte notchecksum;
unsigned char motortorque[14] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x47, 0x02, servoID1, f1_L, f1_H, servoID2, f2_L, f2_H, notchecksum};

unsigned char v1_H, v1_L, v2_H, v2_L;
unsigned char motorspeed[14] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x20, 0x02, servoID1, v1_L, v1_H, servoID2, v2_L, v2_H, notchecksum};

byte checksum1 = ~lowByte(0xFE + 0x09 + 0x92 + 0x02 + servoID1 + 0x24 + 0x02 + servoID2 + 0x24);
unsigned char getposition[13] = {0xFF, 0xFF, 0xFE, 0x09, 0x92, 0x00, 0x02, servoID1, 0x24, 0x02, servoID2, 0x24, checksum1};
unsigned char positiondata[16];

byte checksum2 = ~lowByte(0xFE + 0x09 + 0x92 + 0x02 + servoID1 + 0x26 + 0x02 + servoID2 + 0x26);
unsigned char getspeed[13] = {0xFF, 0xFF, 0xFE, 0x09, 0x92, 0x00, 0x02, servoID1, 0x26, 0x02, servoID2, 0x26, checksum2};
unsigned char speeddata[16];

unsigned char imu[9] = {0x75, 0x65, 0x0C, 0x03, 0x03, 0x03, 0x05, 0xF4, 0xED};
unsigned char data[38];

unsigned long t_now, t_last, time1, time2;

// sensor
float p1_now, p2_now; // motor position read now
float p1_last, p2_last; // motor position read last
float p1_diff, p2_diff;
float n1, n2; // number of turns
float v1, v2, p1, p2; // motor velocity & position calculated (v: 0-1023; p:0-4095)
float vx, vy, px, py; // omniwheel velocity & position calculated (v*0.1144 [rpm]; p*0.088 [deg])
float phi1dot, phi2dot, phi1, phi2;
float pitchdot_last, rolldot_last, v1_last, v2_last; // for filtering

float pitch, roll, yaw, pitchdot, rolldot, yawdot; // body orientation
//float pitch_last, roll_last;

// actuate
float vx_goal, vy_goal; // omniwheel goal velocity
float v1_goal, v2_goal; // motor goal speed
float f1_goal, f2_goal; // motor goal torque

// calibration
float t1_offset, t2_offset; // motor position initial
float pitch_sum, roll_sum, yaw_sum;
float pitch_offset, roll_offset, yaw_offset;
int samplenum = 10;

float f = 125; // 125Hz
float dt = 1 / f;

// control
float vref1, vref2, psi1dot, psi2dot, theta1, theta2; // theta1 - roll
float e1, e2, e1_int, e2_int;
float K11 = 2400; // 2400
float K12 = 110; // 110
float K13 = K12 / 50; // 0.92
float K21 = 1500; // 1500
float K22 = 90; // 90
float K23 = K22 / 50; // 0.0326
float H1 = 1.00; // 1.00
float H2 = 0.08; // 0.035
float Kp1 = 0.05; // 0.05
float Ki1 = 0.05; // 0.05
float Kp2 = 0.0035; // 0.0035
float Ki2 = 0.004; // 0.004

float ch1 = 1500;
float ch2 = 1500;
unsigned long current_time;
unsigned long timer_start;
int last_ch1;
int last_ch2;

void setup() {
  DDRH |= 1 << 5; //pinMode(pin485,OUTPUT);
  //pinMode(2, INPUT);
  //attachInterrupt(digitalPinToInterrupt(2), calc1, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(3), calc2, CHANGE);
  
  Serial.begin(115200);
  Serial1.begin(115200); // IMU
  Serial3.begin(115200); // Motors
  Serial.flush();
  Serial1.flush();
  Serial3.flush();

  Serial1.setTimeout(3.5);
  Serial3.setTimeout(3);

  //Serial.println("Calibration...");
  delay(10);
  Calibration();
  delay(10);
  //EnableTorque(servoID1);
  delay(100);
  //EnableTorque(servoID2);
  delay(100);
  //Serial.print(pitch_offset);
  //Serial.print(roll_offset);
}

void loop() {
  //time1 = micros();
  //ch1 = pulseIn(2, HIGH, 25000);
  // sensor
  //ReadPosition();
  //delayMicroseconds(200);
  ReadSpeed();
  ReadOrientation();
  
  // calculate
  CalculatePositionVelocity();

  // control
  //vref1 = (ch1 - 1500) / 5000;
  //vref2 = (ch2 - 1500) / 5000;
  Control(vref1, vref2);
  //f1_goal = f1_goal*40 - 1*sign(f2_goal);
  //f2_goal = f2_goal*30 + 1*sign(f2_goal);

  // actuate
  actuate_speed(servoID1, f1_goal, servoID2, (f2_goal - f1_goal));
  //actuate_torque(servoID1, f1_goal, servoID2, f2_goal);
  
  //Serial.print(ch1);
  //Serial.print(" ");
  //Serial.println(ch2);
  
  //Serial.print(v1);
  //Serial.print(" ");
  //Serial.println(v2);
  //Serial.print(" ");
  //Serial.print(roll*57.296);
  //Serial.print(" ");
  //Serial.println(pitch*57.296);

  //if (t_now - t_last > 3700){Serial.println("stop");}
  Wait(1000000 * dt); // 10000 us -> 100Hz
  //time2 = micros();
  //Serial.println(time2-time1);
}

void ReadOrientation()
{
  Serial1.write(imu, 9);
  Serial1.readBytesUntil(-1, data, 38);
  pitch = floattodecimal(data[6], data[7], data[8], data[9]); // rad
  roll = floattodecimal(data[10], data[11], data[12], data[13]);
  pitchdot = floattodecimal(data[22], data[23], data[24], data[25]); // rad
  rolldot = floattodecimal(data[26], data[27], data[28], data[29]);
  //rolldot = (roll - roll_last)/dt;
  //pitchdot = (pitch - pitch_last)/dt;
  //roll_last = roll;
  //pitch_last = pitch;
  pitchdot = 0.2 * pitchdot + 0.8 * pitchdot_last;
  pitchdot_last = pitchdot;
  rolldot = 0.2 * rolldot + 0.8 * rolldot_last;
  rolldot_last = rolldot;
}

typedef union
{
  unsigned char u8[4];
  float f32;
} t_F32;

float floattodecimal(char byte0, char byte1, char byte2, char byte3)
{
  t_F32 t;
  t.u8[0] = byte3;
  t.u8[1] = byte2;
  t.u8[2] = byte1;
  t.u8[3] = byte0;
  return t.f32;
}

void Calibration()
{
  int i = 0;
  while (i < samplenum)
  {
    //GetOrientation();
    //pitch_sum += pitch / samplenum;
    //roll_sum += roll / samplenum;
    //yaw_int += yaw / samplenum;
    ReadPosition();
    t1_offset += p1_now / samplenum;
    t2_offset += p2_now / samplenum;
    i++;
  }
}

float sign(float x)
{
  if (x < 0) return -1;
  if (x > 0) return 1;
  return 0;
}

void Wait(float tspan)
{
  while (micros() - t_last < tspan) {};
  //Serial.println(micros() - t_last);
  t_last = micros();
}

void ReadPosition()
{
  PORTH |= 1 << 5; //digitalWrite(pin485,HIGH); //put MAX485 into transmitting mode
  Serial3.write(getposition, 13);
  Serial3.flush(); // critical!!!!!!!!!
  PORTH &= 0 << 5; //digitalWrite(pin485,LOW); //put MAX485 back into receiving mode

  Serial3.readBytesUntil(-1, positiondata, 16);

  p1_now = positiondata[5] + positiondata[6] * 256;
  p2_now = positiondata[13] + positiondata[14] * 256;
}

void ReadSpeed()
{
  PORTH |= 1 << 5; //digitalWrite(pin485,HIGH); //put MAX485 into transmitting mode
  Serial3.write(getspeed, 13);
  Serial3.flush(); // critical!!!!!!!!!
  PORTH &= 0 << 5; //digitalWrite(pin485,LOW); //put MAX485 back into receiving mode

  Serial3.readBytesUntil(-1, speeddata, 16);

  v1 = speeddata[5] + speeddata[6] * 256;
  v2 = speeddata[13] + speeddata[14] * 256;
  if (v1 > 1023)
  {
    v1 = 1024 - v1;
  }
  if (v2 > 1023)
  {
    v2 = 1024 - v2;
  }
}

void CalculatePositionVelocity()
{
  /*
    p1_diff = (p1_now - p1_last);
    p2_diff = (p2_now - p2_last);
    if (p1_diff > 3500)
    {
    n1--;
    //p1_diff -= 4096;
    }
    else if (p1_diff < -3500)
    {
    n1++;
    //p1_diff += 4096;
    }
    if (p2_diff > 3500)
    {
    n2--;
    //p2_diff -= 4096;
    }
    else if (p2_diff < -3500)
    {
    n2++;
    //p2_diff += 4096;
    }
    p1_last = p1_now;
    p2_last = p2_now;

    p1 = n1*4096 + p1_now - t1_offset;
    p2 = n2*4096 + p2_now - t2_offset;
  */
  // exponential filtering
  v1 = 0.2 * v1 + 0.8 * v1_last;
  v1_last = v1;
  v2 = 0.2 * v2 + 0.8 * v2_last;
  v2_last = v2;

  //px = p1;
  //py = (p1 - p2)*4;
  //vx = v1;
  //vy = (v1 - v2)*4;

  phi1 = p1 * 0.00153398078;
  phi2 = (p1 + p2) * 0.00613592315;
  phi1dot = v1 * 0.01198343064;
  phi2dot = (v2 + v1) * 0.04793372257;
}

void actuate_speed(byte servoID1, int v1, byte servoID2, int v2)
{
  if (v1 < 0) // CW 1024-2047
  {
    v1 = min(1024 - v1, 2047);
  }
  else // CCW 0-1023
  {
    v1 = min(v1, 1023);
  }
  //v1 = min(abs(v1), 1023) + 512*(1 + sign(v1)); // CW 1024-2047 & CCW 0-1023

  if (v2 < 0)
  {
    v2 = min(1024 - v2, 2047);
    //v2 = min(v2, 1023);
  }
  else
  {
    v2 = min(v2, 1023);
    //v2 = min(1024 - v2, 2047);
  }

  //v1_H = v1 >> 8;  //same as /256 and truncating
  //v1_L = v1 % 256;
  //v2_H = v2 >> 8;  //same as /256 and truncating
  //v2_L = v2 % 256;
  //notchecksum = ~lowByte(0x1B0 + v1_L + v1_H + v2_L + v2_H);
  //byte notchecksum = ~lowByte(0xFE + 0x0A + 0x83 + 0x20 + 0x02 + servoID_1 + v1_L + v1_H + servoID_2 + v2_L + v2_H);
  //unsigned char motorspeed[14] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x20, 0x02, servoID_1, v1_L, v1_H, servoID_2, v2_L, v2_H, notchecksum};
  motorspeed[8] = v1 % 256;
  motorspeed[9] = v1 >> 8;
  motorspeed[11] = v2 % 256;
  motorspeed[12] = v2 >> 8;
  motorspeed[13] = ~lowByte(0x1B0 + motorspeed[8] + motorspeed[9] + motorspeed[11] + motorspeed[12]);

  PORTH |= 1 << 5; //digitalWrite(pin485,HIGH); //put MAX485 into transmitting mode
  Serial3.write(motorspeed, 14);
  Serial3.flush(); // critical!!!!!!!!!
  PORTH &= 0 << 5; //digitalWrite(pin485,LOW); //put MAX485 back into receiving mode
}

void actuate_torque(byte servoID1, int f1, byte servoID2, int f2) {
  if (f1 < 0) // CW 1024-2047
  {
    f1 = min(1024 - f1, 2047);
  }
  else // CCW 0-1023
  {
    f1 = min(f1, 1023);
  }

  if (f2 < 0)
  {
    f2 = min(1024 - f2, 2047);
    //f2 = min(f2, 1023);
  }
  else
  {
    f2 = min(f2, 1023);
    //f2 = min(1024 - f2, 2047);
  }
  motortorque[8] = f1 % 256;
  motortorque[9] = f1 >> 8;
  motortorque[11] = f2 % 256;
  motortorque[12] = f2 >> 8;
  motortorque[13] = ~lowByte(0x1D7 + motortorque[8] + motortorque[9] + motortorque[11] + motortorque[12]);

  PORTH |= 1 << 5; //digitalWrite(pin485,HIGH); //put MAX485 into transmitting mode
  Serial3.write(motortorque, 14);
  Serial3.flush(); // critical!!!!!!!!!
  PORTH &= 0 << 5; //digitalWrite(pin485,LOW); //put MAX485 back into receiving mode
}

void EnableTorque(byte servoID)
{
  unsigned char enable[8] = {0xFF, 0xFF, servoID, 0x04, 0x03, 0x46, 0x01, 0x00};
  enable[7] = ~lowByte(0x4E + servoID);
  PORTH |= 1 << 5; //digitalWrite(pin485,HIGH); //put MAX485 into transmitting mode
  Serial3.write(enable, 8);
  Serial3.flush(); // critical!!!!!!!!!
  PORTH &= 0 << 5; //digitalWrite(pin485,LOW); //put MAX485 back into receiving mode
}

void Control(float vref1, float vref2) // velocity tracking
{
  psi1dot = vref1 / 0.102;
  psi2dot = vref2 / 0.0142;

  e1 = psi1dot - rolldot - phi1dot;
  e2 = psi2dot - pitchdot - phi2dot;
  e1_int += e1 * dt;
  e2_int += e2 * dt;

  theta1 = 0.001 * 0 + roll - Kp1 * e1 - Ki1 * e1_int;
  theta2 = 0.011 * 0 + pitch - Kp2 * e2 - Ki2 * e2_int;

  f1_goal =  (K11 * theta1 + K12 * rolldot + K13 * phi1dot - H1 * psi1dot);
  f2_goal =  (K21 * theta2 + K22 * pitchdot + K23 * phi2dot - H2 * psi2dot) * 4;
}

void LinePath(float velocity, float angle)
{
  if (millis() > 6000)
  {
    vref1 = 0.0354; // 5cm/s
    vref2 = -0.0354; // 5cm/s
    if (millis() > 21000)
    {
      vref1 = 0;
      vref2 = 0;
    }
  }
}

void TriPath()
{
  if (millis() > 6000)
    {
    vref1 = 0.0; // 5cm/s
    vref2 = -0.05; // 5cm/s
    if (millis() > 21000)
    {
      vref1 = 0;
      vref2 = 0;
      if (millis() > 23000)
      {
        vref1 = 0.04;
        vref2 = 0.025;
        if (millis() > 38000)
        {
          vref1 = 0;
          vref2 = 0;
          if (millis() > 40000)
          {
            vref1 = -0.04;
            vref2 = 0.025;
            if (millis() > 55000)
            {
              vref1 = 0;
              vref2 = 0;
           }
          }
        }
      }
    }
    }
}

void calc1()
{
 current_time = micros();
 if(digitalRead(2) == HIGH)
 {
  if (last_ch1 == 0)
  {
    last_ch1 = 1;
    timer_start = current_time;
  }
 }
 else if (last_ch1 == 1)
 {
  last_ch1 = 0;
  ch1 = current_time - timer_start;
 }
}

void calc2()
{
 current_time = micros();
 if(digitalRead(3) == HIGH)
 {
  if (last_ch2 == 0)
  {
    last_ch2 = 1;
    timer_start = current_time;
  }
 }
 else if (last_ch2 == 1)
 {
  last_ch2 = 0;
  ch2 = current_time - timer_start;
 }
}
