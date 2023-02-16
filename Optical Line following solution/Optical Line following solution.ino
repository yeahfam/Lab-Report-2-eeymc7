const int left3 = 35;
const int left2 = 27;
const int left1 = 26;
const int right1 = 33;
const int right2 = 32;
const int right3 = 25;

float relPos=0;
int left3out = 0;
int left2out = 0;  
int left1out = 0;  
int right1out = 0;  
int right2out = 0;  
int right3out = 0;           // value read from the pot
int outputValue = 0;
int Kp=1.8;
int Ki=1;
int Kd=1;
int n=1;
float error;
float u;
int centreAngle = 80;
float prevError;

const int trigPin = 18;
const int echoPin = 5;
float angle = 0;
float distance = 999;
long duration;
float distancecm;
int leftMotor_speed, rightMotor_speed, servoAngle;
#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
int x = 0;
void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop()
{
 
  n=n+1;
  left3out = analogRead(left3);
  left2out = analogRead(left2);
  left1out = analogRead(left1);
  right1out = analogRead(right1);
  right2out = analogRead(right2);
  right3out = analogRead(right3);

  //Forward 1 second
  leftMotor_speed = 0;//-255;
  rightMotor_speed = 0;//255;
  servoAngle = 80;

  relPos=((((-37.5)*(4095-left3out))+((-22.5)*(4095-left2out))+((-7.5)*(4095-left1out))+((7.5)*(4095-right1out))+((22.5)*(4095-right2out))+((37.5)*(4095-right3out)))/(1+((4095-right3out)+(4095-right2out)+(4095-right1out)+(4095-left1out)+(4095-left2out)+(4095-left3out))));
 
  error=relPos;
  if ((prevError >=37.4) && (error == 0))
  {
    while (error==0)
    {
      leftMotor_speed = 120;
      rightMotor_speed = -120;
      servoAngle = 80;
      Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
      delay(5);

      left3out = analogRead(left3);
      left2out = analogRead(left2);
      left1out = analogRead(left1);
      right1out = analogRead(right1);
      right2out = analogRead(right2);
      right3out = analogRead(right3);

      relPos=((((-37.5)*(4095-left3out))+((-22.5)*(4095-left2out))+((-7.5)*(4095-left1out))+((7.5)*(4095-right1out))+((22.5)*(4095-right2out))+((37.5)*(4095-right3out)))/(1+((4095-right3out)+(4095-right2out)+(4095-right1out)+(4095-left1out)+(4095-left2out)+(4095-left3out))));
      error=relPos;
     
    }
   
  }

  else if ((prevError <=-37.4) && (error == 0))
  {
    while (error==0)
    {
      leftMotor_speed = 120;
      rightMotor_speed = -120;
      servoAngle = 80;
      Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
      delay(50);

      left3out = analogRead(left3);
      left2out = analogRead(left2);
      left1out = analogRead(left1);
      right1out = analogRead(right1);
      right2out = analogRead(right2);
      right3out = analogRead(right3);

      relPos=((((-37.5)*(4095-left3out))+((-22.5)*(4095-left2out))+((-7.5)*(4095-left1out))+((7.5)*(4095-right1out))+((22.5)*(4095-right2out))+((37.5)*(4095-right3out)))/(1+((4095-right3out)+(4095-right2out)+(4095-right1out)+(4095-left1out)+(4095-left2out)+(4095-left3out))));
      error=relPos;
    }
  }
  u=(error*Kp);//(error*n*Ki)

  servoAngle=centreAngle+u;

  leftMotor_speed = -160-0.8*u;
 
  rightMotor_speed = 160-0.8*u;

  Serial.print(relPos);
  Serial.print("\n");
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);


  //delay(1000);
  //stop
  delay(50);
  prevError=error;
}

void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}

float accelerometer()
{
  mpu6050.update();
  return (mpu6050.getAngleZ());
}

float ultrasonic()
{
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034 / 2);
}