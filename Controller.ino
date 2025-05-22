#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

//Defining the variables.

Servo esc1;
Servo esc2;
float speed1;
float speed2;
float pitch =0;
float ang_vel=0;
float angle_offset=0;
float Kp  = 0.3;
float Kd  = 0.1;
float Ki = 0.0;
float correction = 0;
float corr_i = 0;

void setup()
{
  Serial.begin(115200);
  esc1.attach(9,1000,2000);
  esc2.attach(2,1000,2000);

  esc1.write(180);
  esc2.write(180);
  delay(5000);
  esc1.write(0);
  esc2.write(0);
  delay(2000);
  speed1 = 20;
  speed2 = speed1;

  while(!mpu.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu.calibrateGyro(100);
  delay(3000);
}

void loop()
{
  // Read normalized values
  esc1.write(speed1);
  esc2.write(speed2);
  int i=0;
  //Averaging out every 3 inputs to get more accurate reading.
  while(i<3){
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();
  pitch += -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  ang_vel += normGyro.YAxis;
  i+=1;
  delay(5);
  }
  pitch /=3;
  ang_vel /=3;

  //Adding a high pass filter inorder to reduce oscillations if pitch is near 0.
  if(ang_vel<5 && ang_vel>-5){
  if(pitch >3 && pitch<-3){
    pitch = 0;
    ang_vel = 0;
    }
  }

  if((pitch>30)&&(pitch<-30)){
  corr_i += Ki*pitch;}

  correction = -(Kp*pitch + Kd*ang_vel+corr_i);

  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print(" Ang_Vel = ");
  Serial.print(ang_vel);
  Serial.print(" Correction = ");
  Serial.print(correction);

  if((speed1+correction/2)>70){
    speed1=100;
    }
  else if((speed1+correction/2)<10){
    speed1=10;
    }
  else{
    speed1 = 30+correction/2;
  }
  if((speed2-correction/2)<10){
    speed2 = 10;
  }
  else if((speed2-correction/2)>70){
    speed2 = 70;
  }
  else{
    speed2 = 30-correction/2;
  }
}
