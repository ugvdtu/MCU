#define M1_PWM1 9 
#define M1_PWM2 10
#define M1_DIR1 11
#define M1_DIR2 12

void Clockwise()
void AntiClockwise()
void Speed(int pwm)
void brake()
void setup()
{
  pinMode(M1_DIR1,OUTPUT);
  pinMode(M2_DIR2,OUTPUT);
  pinMode(M1_PWM1,OUTPUT);
  pinMode(M2_PWM2,OUTPUT);
}

void loop()
{

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
  analogWrite(pwm,OUTPUT)
    digitalWrite(M1_PWM2 ,LOW)
    
}
