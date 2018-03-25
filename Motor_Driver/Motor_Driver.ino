#define M1_PWM1 9 
#define M1_PWM2 10
#define M1_DIR1 12
#define M1_DIR2 13

void Clockwise();
void AntiClockwise();
void Speed(int pwm);
void brake();

void setup()
{
  pinMode(M1_DIR1,OUTPUT);
  pinMode(M1_DIR2,OUTPUT);
  pinMode(M1_PWM1,OUTPUT);
  pinMode(M1_PWM2,OUTPUT);
  analogWrite( M1_PWM1, 255);
}

void loop()
{
  Clockwise();
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
  for (int pwm = 5 ; pwm < 245; pwm += 10) // speed will get increased untill it reach 245
  {
    analogWrite( M1_PWM1, pwm);
    delay(100);
    if (pwm == 245) // at 245 speed becomes maximum
    {
      pwm = 255; // runs infinetly
      
    }
  }
    
}

