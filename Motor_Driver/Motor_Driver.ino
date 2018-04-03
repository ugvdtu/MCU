#define M1_PWM1 9 
#define M1_PWM2 10
#define M1_DIR1 12
#define M1_DIR2 13

void Clockwise();
void AntiClockwise();
void Speed(int pwm);
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
  pinMode(M1_DIR1,OUTPUT);
  pinMode(M1_DIR2,OUTPUT);
  pinMode(M1_PWM1,OUTPUT);
  pinMode(M1_PWM2,OUTPUT);
  pinMode(13,OUTPUT);
  setPwmFrequency(9, 8); //9,8 ...3.9Khz
  delay(1000);
  analogWrite(9,200);
}

void loop()
{
  //digitalWrite(13,HIGH);
  //delay(1000);
  digitalWrite(13,LOW);
  //delay(1000);
}

void Clockwise()
{
  digitalWrite(M1_DIR1,HIGH);
  digitalWrite(M1_DIR2,LOW);
}

void AntiClockwise()
{
  digitalWrite(M1_DIR1,LOW);
  digitalWrite(M1_DIR2,HIGH);
}

void Speed(int pwm)
{
  analogWrite(M1_PWM2, pwm);
}

