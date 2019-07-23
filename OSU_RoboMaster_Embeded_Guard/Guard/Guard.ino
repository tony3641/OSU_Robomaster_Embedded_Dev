#define motor1 5
#define motor2 3
#define motor3 9
#define motor4 6
#define Infrared1 A3
#define Infrared2 A1

#define GREEN_PIN 12
#define RED_PIN 13
#define BLUE_PIN 11

#define NUM 9
const float BM[NUM] = {
  6.020246598159e-18, 0.05528058611835, 0.1318813582209, 0.1995449383455,
  0.2265862346306, 0.1995449383455, 0.1318813582209, 0.05528058611835,
  6.020246598159e-18
};
//定义结构体
typedef struct
{
  float ybuf[NUM];
  float xbuf[NUM];
  float filtered_value;
  float raw_value;
} LPF_t;
//声明类型
LPF_t lpf1;
LPF_t lpf2;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  pinMode(Infrared1, INPUT);
  pinMode(Infrared2, INPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  Serial.begin(9600);
}
String state, last_state;
int swing_flag = 0;
int move_flag = 0;
void loop() {
  move_flag = 1;
  swing_flag = 0;
  if (swing_flag == 1 && move_flag == 0)
  {
    RightWheel(255);
    LeftWheel(-255);
    delay(1500);
    RightWheel(-200);
    LeftWheel(200);
    delay(1200);
  }
  else if (swing_flag == 0 && move_flag == 1)
  {
    lpf1.raw_value = analogRead(Infrared1);
    lpf2.raw_value = analogRead(Infrared2);
    LPF(&lpf1);//run lpf
    LPF(&lpf2);
    float a = lpf1.filtered_value;
    float b = lpf2.filtered_value;
    if (4 * a < 3000 && 2 * b > 1300 && last_state != "FORWARD")
    {
      state = "FORWARD";
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      Move(random(85, 150));

    }

    else if (2 * b < 1000 && last_state != "BACKWARD")
    {
      state = "BACKWARD";
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(RED_PIN, LOW);
      Move(random(-150, -85));
    }
    //  else
    //  {
    //    digitalWrite(RED_PIN, LOW);
    //    digitalWrite(GREEN_PIN, LOW);
    //    Move(0);
    //  }
    last_state = state;
    Serial.print(4 * a); Serial.print(",");
    Serial.println(2 * b);
  }


}

void Move(int Speed)
{
  RightWheel(Speed);
  LeftWheel(Speed);
}

void RightWheel(int Speed)
{
  if (Speed > 255)
  {
    Speed = 255;
  }
  else if (Speed < -255)
  {
    Speed = -255;
  }

  if (Speed >= 0)
  {
    analogWrite(motor1, Speed);
    analogWrite(motor2, 0);
  }
  else
  {
    analogWrite(motor1, 0);
    analogWrite(motor2, -Speed);
  }
}
void LeftWheel(int Speed)
{
  if (Speed > 255)
  {
    Speed = 255;
  }
  else if (Speed < -255)
  {
    Speed = -255;
  }

  if (Speed >= 0)
  {
    analogWrite(motor3, Speed);
    analogWrite(motor4, 0);
  }
  else
  {
    analogWrite(motor3, 0);
    analogWrite(motor4, -Speed);
  }
}

float LPF(LPF_t *F)
{
  int i;
  for (i = NUM - 1; i > 0; i--)
  {
    F->xbuf[i] = F->xbuf[i - 1];
  }
  F->xbuf[0] = F->raw_value;
  F->ybuf[0] = BM[0] * F->xbuf[0];
  for (i = 1; i < NUM; i++)
  {
    F->ybuf[0] = F->ybuf[0] + BM[i] * F->xbuf[i];
  }
  F->filtered_value = F->ybuf[0];
  return F->filtered_value;
}
