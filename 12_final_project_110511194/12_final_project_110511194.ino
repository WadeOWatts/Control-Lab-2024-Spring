//小車馬達控制
#include "Arduino.h"
#define CONTROL_RATE 2 // 2 Hz, i.e. 0.5 s
int u=0; //PID控制後的結果 從python送來 設初始值為0
int a=5; //移動模式 設起始為靜止
int m=0; //使前後左右移動正常的變數

#define Pin_Encoder_Right_A 2          //     E2A----------------2
#define Pin_Encoder_Right_B 3          //     E2B----------------3
#define Pin_Encoder_Left_A 20          //     E1A----------------20
#define Pin_Encoder_Left_B 21          //     E1B----------------21
//M1左輪   M2右輪
//     E2A----------------2
//     E2B----------------3
//     E1A----------------20
//     E1B----------------21
long theta_Right = 0, theta_Left = 0; // For encoders to record angles
unsigned long currentMillis;          // Record the current time
long previousMillis = 0;              // Set up timers

//直流馬達----------TB6612腳位----------ArduinoUNO腳位
//                             PWMA-----------------4
//                             AIN1--------------------6
//                             AIN2--------------------5
//                             STBY-------------------7
//                             PWMB-----------------8
//                             BIN1-------------------10
//                             BIN2-------------------9
//                     
//                             GND-------------------GND
//                             VM--------------------12V
//                             VCC-------------------5V
//                             GND------------------GND
//直流馬達----------TB6612腳位----------ArduinoUNO腳位

#define PWMA 4
#define AIN1 6
#define AIN2 5
#define STBY 7
#define PWMB 8
#define BIN1 10
#define BIN2 9

int const trigPin= 47;        // Pin for ultrasound sensor
int const echoPin= 49;        // Pin for ultrasound sensor
double Duration;              
int Distance = 1000;          // Initialize a very far distance to pretend that this is a wide space.

void Timer1Init(void){        // For timer interruption
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = (62500/CONTROL_RATE);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  interrupts(); 
 }

 ISR(TIMER1_COMPA_vect){
  Calculate_distance();       // Calculate distance with the data from the ultrasound sensor every 0.5 second
 }

int PwmA, PwmB;
void initMotor(){             // Code given
  //控制訊號初始化
  pinMode(AIN1, OUTPUT);//控制馬達A的方向，(AIN1, AIN2)=(1, 0)為正轉，(AIN1, AIN2)=(0, 1)為反轉
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);//控制馬達B的方向，(BIN1, BIN2)=(0, 1)為正轉，(BIN1, BIN2)=(1, 0)為反轉
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);//A馬達PWM
  pinMode(PWMB, OUTPUT);//B馬達PWM
  pinMode(STBY, OUTPUT);//TB6612致能,設置0則所有馬達停止,設置1才允許控制馬達
  //初始化TB6612馬達驅動模組
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
void initEncoder(){                              // Code given
  pinMode(Pin_Encoder_Right_A, INPUT_PULLUP);
  pinMode(Pin_Encoder_Right_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pin_Encoder_Right_A), doEncoder_Right, RISING);
  pinMode(Pin_Encoder_Left_A, INPUT_PULLUP);
  pinMode(Pin_Encoder_Left_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pin_Encoder_Left_A), doEncoder_Left, RISING);  
}


void setup() {
  Serial.begin(9600);               // Setup for communication with raspberry pi
  Serial2.begin(9600);
  initMotor();                      // Motor
  initEncoder();                    // Encoder
  previousMillis = millis();        // Present time
  pinMode(trigPin,OUTPUT);          // For ultrasound sensor
  pinMode(echoPin,INPUT);           // For ultrasound sensor  
  digitalWrite(trigPin,LOW);        // For ultrasound sensor
  Timer1Init();                     // For timer interruption
  Serial.setTimeout(2);             // Communication with raspberry pi 用1的話 在輸出u值時會有問題
}

void SetPWM(int motor, int pwm) {
  //motor=1代表控制馬達A，pwm>=0則(AIN1, AIN2)=(1, 0)為正轉
  if(motor==1 && pwm>=0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  }

  //motor=1代表控制馬達A，pwm<0則(AIN1, AIN2)=(0, 1)為反轉
    if(motor==1 && pwm<0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMB, -pwm);
  }
  
  //motor=2代表控制馬達B，pwm>=0則(BIN1, BIN2)=(0, 1)為正轉
    if(motor==2 && pwm>=0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, pwm);
  }
  
  //motor=2代表控制馬達B，pwm<0則(BIN1, BIN2)=(1, 0)為反轉
    if(motor==2 && pwm<0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -pwm);
  }
}

inline void doEncoder_Right() {                 // Code given
  if (digitalRead(Pin_Encoder_Right_B) == LOW)
    theta_Right = theta_Right - 1;
  else
    theta_Right = theta_Right + 1;
}//end of void doEncoder_Right_A()
inline void doEncoder_Left() {                  // Code given
if (digitalRead(Pin_Encoder_Left_B) == LOW)
    theta_Left = theta_Left + 1;
  else
    theta_Left = theta_Left - 1;
}//end of void doEncoder_Left_A()

//定義前進、後退、左轉、右轉、停止
void forward(){                     // 前進
  SetPWM( 1, 40);
  SetPWM( 2, -37);
}
void right(){                       // 右轉
  SetPWM( 1,  45);
  SetPWM( 2,  45);
}
void left(){                        // 左轉
  if(m==1){                         // Move forward a little bit so that the wheels will be smoother.
    forward();                      // Only for left turn and backwrad
    m=0;
  }
  SetPWM( 1, -45);
  SetPWM( 2, -45);
}
void back(){                        // 後退
  if(m==1){
    forward();
    m=0;
  }
  SetPWM( 1, -40);
  SetPWM( 2, 40);
}
void stopp(){                       // 停止
  SetPWM( 1, 0);
  SetPWM( 2, 0);
}

int PIDbaseSpeed=32;                // 基礎速度25
void PIDmove(int x){
  int y=x;
  int z=x;
  if((PIDbaseSpeed+x)<=15){         //若左輪速度低於15 則設為15
    y=15-PIDbaseSpeed;
  }
  if((-PIDbaseSpeed+x)>=-15){       //若右輪速度低於15 則設為15
    z=PIDbaseSpeed-15;
  }
  SetPWM( 1, PIDbaseSpeed+y);
  SetPWM( 2, -PIDbaseSpeed+z);
}

void loop() {
   if(Serial.available()){
    u = Serial.parseInt();          //從python得到u
    
    if(u==1000){                    //前進
      a=1;
    }
    else if(u==2000){               //左轉
      a=2;
      m=1;
    }
    else if(u==3000){               //後退
      a=3;
      m=1;
    }
    else if(u==4000){               //右轉
      a=4;
    }
    else if(u==5000){               //停止
      a=5;
    }
    else if(u==6000){               //PID控制移動
      u=0;
      a=0;
    }
    else if(u==8000){               //判斷出三角形朝左
      u=0;
      a=6;
      m=1;
    }
    else if(u==9000){               //判斷出三角形朝右
      u=0;
      a=7;
    }
    else if(u==7000){               // First turn in lidar mode
      a = 8;
    }
    else if(u == 5500){             // Second turn in lidar mode
      a = 9;
    }
    else if(u>=20){                 //速度限制
      u=20;                         // Preventing speeding up the motor too quickly
    }
    else if(u<=-20){                //速度限制
      u=-20;
    }
    
    switch(a) {
        case 0:                     //PID控制移動
            PIDmove(u);             //驅動馬達
            break;
        case 1:                     //前進
            forward();
            break;
        case 2:                     //左轉
            left();
            break;
        case 3:                     //後退
            back();
            break;
        case 4:                     //右轉
            right();
            break;
        case 5:                     //停止
            stopp();
            break;
        case 6:                     //判斷出三角形朝左
            if(Distance<50 && Distance>5){ // If robot is close enough
              stopp();              // Stop to let us know it detected the sign
              delay(1000);          // Stop for 1 sec
              forward();            // Move forward to the sign
              delay(3200);          // 2700:40 1500:32 3700:50
              left();               // Turn left
              delay(1450);          // For 1.45 sec, approx. 90 degrees
              u=1234;               // Send the flag back to raspberry pi
              a=0;                  // Move forward
            }
            else{
              PIDmove(u);           // If it is not close enough to the sign, just keep tracing the lane
            }
            break;
        case 7:                     //判斷出三角形朝右
            if(Distance<50 && Distance>5){  // If it close enough to the sign, below is the same logic as above
              stopp();
              delay(1000);
              forward();
              delay(3400);          // 2700:40 1500:32 (PID speed)
              right();
              delay(1350);          // 1500:11.9V 1850:11.6V best:1850
              a=0; 
            }
            else{
              PIDmove(u);
            }
            break;
         case 8:                    // Turn left slightly in order not to bump into the wall
            m=1;
            left();
            delay(400);
            stopp();                  // Stop to let us know the phase it is
            delay(1500);
            a = 1;
            break;
         case 9:                    // Trun left before stop in the parking lot
           forward();
           delay(3000);
           stopp();                 // Stop to let us know the phase it is
           delay(1000);
           a = 1;
           break;
    }
    if(u >= 1000)
      Serial.println(u);            //把u丟回python 確認是否正確
   } 
}

void Calculate_distance(){
  digitalWrite(trigPin,HIGH);       //發射超音波
  delay(1);
  digitalWrite(trigPin,LOW);
  Duration = pulseIn(echoPin,HIGH); //超音波發射到接收的時間
  Distance = Duration * 0.034 / 2;  //計算距離(cm) Assume the temperature is 25 degrees Celcius
}