//UGV

#include <ros.h>
#include <std_msgs/String.h>
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#define M1_PWM 9 
#define M2_PWM 10
#define M1_DIR 12
#define M2_DIR 13
#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling
int dummy=0;
MPU9250 myIMU;

void Clockwise(int pin);
void AntiClockwise(int pin);
void Speed(int pwm,int pin);
void Forward(int pwm);

ros::NodeHandle  nh;

int test=-1; 

void messageCb( const std_msgs::UInt8MultiArray& msg){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  
  // msg[0]: Direction of motor #1
  // msg[1]: Speed of motor #1
  // msg[2]: Direction of motor #2
  // msg[3]: Speed of motor #2
  test=msg.data[0];
  if(msg.data[0] == 1) 
    {Clockwise(M1_DIR);
     //digitalWrite(LED_BUILTIN,HIGH);
    }
  else 
    {AntiClockwise(M1_DIR);
     //digitalWrite(LED_BUILTIN,LOW);
    }
  if(msg.data[2] == 1)
    {
      Clockwise(M2_DIR);
    }
  else 
     {
      AntiClockwise(M2_DIR);
     }
  
  Speed(msg.data[1],M1_DIR);
  Speed(msg.data[3],M2_DIR);
}

//UnInt8Multi Array
std_msgs:: Int8MultiArray msg;
ros::Publisher chatter("MCU_Output", &msg);
ros::Subscriber<std_msgs::UInt8MultiArray> sub("MCU_Input", &messageCb );

void setup()
{
  //SET PWM Frequency For Motor Driver
  
  
  //IMU Routine
  Wire.begin();
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.magCalibration);

  //Motor Routine
  pinMode(M1_DIR,OUTPUT);
  pinMode(M2_DIR,OUTPUT);
  pinMode(M1_PWM,OUTPUT);
  pinMode(M2_PWM,OUTPUT);
  pinMode(13,OUTPUT);
  //setPwmFrequency(M1_PWM, 8); //9,8 ...3.9Khz
  //setPwmFrequency(M2_PWM, 8);
  analogWrite(M1_PWM,50);
  analogWrite(M2_PWM,50);
  
  //Publisher Setup
  msg.data = (int8_t *)malloc(sizeof(int8_t)*4);
  msg.data_length=4;

  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 100)
    {
      
      myIMU.count = millis();
      
    } 
  } 
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
       myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;
      dummy=(int)myIMU.yaw;
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } 
  }
  msg.data[0]=test;
  msg.data[1]=1;
  msg.data[2]=dummy;
  msg.data[3]=0;
  chatter.publish( &msg );
  
  nh.spinOnce();
  delay(1);
}

void Clockwise(int pin)
{
  digitalWrite(pin,HIGH);
}

void AntiClockwise(int pin)
{
  digitalWrite(pin,LOW);
}

void Speed(int pwm,int pin)
{
  analogWrite(pin, pwm);
}

void Forward(int pwm)
{
  Clockwise(M1_DIR);
  Speed(pwm,M1_DIR);
  Clockwise(M2_DIR);
  Speed(pwm,M2_DIR);
}





