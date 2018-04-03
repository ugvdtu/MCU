//UGV

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define M1_PWM 9 
#define M2_PWM 10
#define M1_DIR 12
#define M2_DIR 13

void Clockwise(int pin);
void AntiClockwise(int pin);
void Speed(int pwm,int pin);
void Forward(int pwm);
void Left_Right(int pwml,int pwmr);
void brake();

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


ros::NodeHandle  nh;

char hello[13] = "hello world!";

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

std_msgs::String str_msg;
ros::Publisher chatter("MCU_Output", &str_msg);
ros::Subscriber<std_msgs::Empty> sub("MCU_Input", &messageCb );

void setup()
{
  pinMode(M1_DIR,OUTPUT);
  pinMode(M2_DIR,OUTPUT);
  pinMode(M1_PWM,OUTPUT);
  pinMode(M2_PWM,OUTPUT);
  pinMode(13,OUTPUT);
  setPwmFrequency(M1_PWM, 8); //9,8 ...3.9Khz
  setPwmFrequency(M2_PWM, 8);
  analogWrite(M1_PWM,50);
  analogWrite(M2_PWM,50);
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
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

void Left_Right(int pwml,int pwmr)
{
  Clockwise(M1_DIR);
  Speed(pwml,M1_DIR);
  Clockwise(M2_DIR);
  Speed(pwmr,M2_DIR);
}


void brake()
{
  Speed(0,M1_DIR);
  Speed(0,M2_DIR);
}



