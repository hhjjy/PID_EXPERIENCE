                                                                                                                                                                                   #include "Arduino.h"
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <PID_v1.h>

#define PWM_PIN 4
#define AIN2 22
#define AIN1 23
#define STBY 24
#define ENCODER_A 3
#define ENCODER_B 19
int pos = 0; 
double speed  = 0 ; 
volatile int  pos_i =  0 ; 
volatile double speed_i = 0;
volatile int prevT = 0 ; 
int time = 0 ; 
// pid 
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=1, Ki=0.01, Kd=0;
PID myPID(&speed, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);// 輸入是速度(RPM) , 輸出是PWM(0-255) , 設定點是速度(RPM) 


void updateEncoder()
{
  int current  ; 
  int increment = 0 ; 
  if (digitalRead(ENCODER_B)>0) // forward 
  {
    increment = 1; 
  }
  else// backward                                                                                                                                                                                                                                   
  {
      increment = -1 ;  
  }
  pos_i += increment ;
  // update speed 
  current = micros() ;
  double deltaT = ((double)(current-prevT))/1e6 ; // 單次除以一次花的週期
  speed_i = increment/deltaT ; // 速度 = 增量 / 時間
  speed_i = speed_i*60.0/400.0; // 一秒轉幾圈換算
  prevT = current ;
}
/**
 * @brief Set the Motor object
 * 
 * @param dir 方向設定 1 正轉 , 0剎車, -1 反轉 
 * @param pwm_value 0-255
 */
void setMotor(int dir,int pwm_value)
{
  // pwm 控制
  analogWrite(PWM_PIN, pwm_value);
  // 方向控制
  switch (dir)
  {
  case 1:
    digitalWrite(AIN1, HIGH); 
    digitalWrite(AIN2, LOW);
    break;
  case 0:
    digitalWrite(AIN1, HIGH); 
    digitalWrite(AIN2, HIGH);
    break;
  case -1:
    digitalWrite(AIN1, LOW); 
    digitalWrite(AIN2, HIGH);
    break; 
  }

}
void setup() {
  // 初始化串口通訊
  Serial.begin(115200);

  // 設定腳位
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // 初始化腳位狀態
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWM_PIN, 0); // 設定PWM輸出為0
  // interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);
  //initialize the variables we're linked to
  Setpoint = 100;// 預設RPM=100 
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(100); // 設定PID的更新週期
  
}
void loop() {
  // 讀取編碼器的值
  int encoder_a = digitalRead(ENCODER_A);
  int encoder_b = digitalRead(ENCODER_B);
  // 打印編碼器的值
  time+= 1 ; 
  Serial.print("loop");
  Serial.println(time);
  // Serial.print(">EncoderA:");
  // Serial.println(encoder_a);
  // Serial.print(">EncoderB:");
  // Serial.println(encoder_b);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
    speed = speed_i;
  }
  // Serial.print(">Counts:");
  // Serial.println(pos);
  Serial.print(">Speed_RPM:");
  Serial.println(speed);
  Serial.print(">Error:");
  Serial.println(Setpoint - speed);

  myPID.Compute(); // 計算PID
  setMotor(1,(int)Output);

  // 稍作延遲
  delay(10);
}

