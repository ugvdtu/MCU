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

void setup()
{
  pinMode(M1_DIR,OUTPUT);
  pinMode(M2_DIR,OUTPUT);
  pinMode(M1_PWM,OUTPUT);
  pinMode(M2_PWM,OUTPUT);
  pinMode(13,OUTPUT);
  setPwmFrequency(9, 8); //9,8 ...3.9Khz
  setPwmFrequency(10, 8);
  delay(1000);
  analogWrite(9,50);
  analogWrite(10,50);
}

void loop()
{
  //digitalWrite(12,HIGH);
  //digitalWrite(13,HIGH);
  Clockwise(M1_DIR);
  Clockwise(M2_DIR);
  delay(1000);
  //digitalWrite(12,LOW);
  //digitalWrite(13,LOW);
  AntiClockwise(M1_DIR);
  AntiClockwise(M2_DIR);
  delay(1000);
  
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

