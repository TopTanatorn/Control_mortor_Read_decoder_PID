#include <Arduino.h>
#include <SoftwareSerial.h>
#include<TaskScheduler.h>
#include <LiquidCrystal.h>
const int rs = PE11, en = PE12, d4 = PE7, d5 = PE8, d6 = PE9, d7 = PE10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//----------------------------------------------------------------
//SoftwareSerial MySerial(PA1, PA0); // RX, TX
//----------------------------------------------------------------
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
//----------------------------------------------------------------
// Motor Control PIN
//----------------------------------------------------------------
#define pwmL PD12
#define DirL PD11

#define pwmR PD14
#define DirR PD13

int Duty_L = 0;
int Duty_R = 0;
int MAX_PWM_DUTY = 128;
int Step_Size = 5;
//----------------------------------------------------------------
// Ultrasonic PIN
//----------------------------------------------------------------
// Front sensor
#define echoPinF PE4 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPinF PE5 //attach pin D3 Arduino to pin Trig of HC-SR04
// rear sensor
#define echoPinR PE2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPinR PE3 //attach pin D3 Arduino to pin Trig of HC-SR04
//----------------------------------------------------------------
// Global var
//----------------------------------------------------------------
int data_in = 0;
long durationF; // variable for the duration of sound wave travel
int distanceF = 100; // variable for the distance measurement
long durationR; // variable for the duration of sound wave travel
int distanceR = 100; // variable for the distance measurement
int MinDist = 40;
//----------------------------------------------------------------
void CalDistanceF();
void CalDistanceR();
void MotorControl();
void LCDdisplay();

Scheduler runner;
Task ReadDistanceF(500, TASK_FOREVER, &CalDistanceF);
Task ReadDistanceR(500, TASK_FOREVER, &CalDistanceR);
Task RobotControl(250, TASK_FOREVER, &MotorControl);
//Task LCD_Display(500, TASK_FOREVER, &LCDdisplay);
//----------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  //MySerial.begin(9600);
  Serial4.begin(9600);
  // Motor
  pinMode(PD11,OUTPUT);
  pinMode(PD12,OUTPUT);
  pinMode(PD13,OUTPUT);
  pinMode(PD14,OUTPUT);

  // Ultrasonics
  pinMode(trigPinF, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinF, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPinR, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinR, INPUT); // Sets the echoPin as an INPUT

  // LCD
  pinMode(PE11,OUTPUT);
  pinMode(PE12,OUTPUT);
  pinMode(PE7,OUTPUT);
  pinMode(PE8,OUTPUT);
  pinMode(PE9,OUTPUT);
  pinMode(PE10,OUTPUT);
  lcd.begin(16, 2);
  lcd.print("Waiting command");
  
  runner.addTask(ReadDistanceF);
  runner.addTask(ReadDistanceR);
  runner.addTask(RobotControl);
 // runner.addTask(LCD_Display);
  ReadDistanceF.enable();
  ReadDistanceR.enable();
  RobotControl.enable();
//  LCD_Display.enable();
}
//----------------------------------------------------------------
void loop() 
{ 
  Serial.println("Waiting command...");
 // if (MySerial.available() > 0) 
 if (Serial4.available() > 0) 
  {
    //data_in = MySerial.readString().toInt();
    data_in = Serial4.readString().toInt();
    Duty_L = 0;
    Duty_R = 0;
    //Serial.print("Receive command :");
    //Serial.println(data_in);
  }
  runner.execute();
 
}
//----------------------------------------------------------------
void MotorControl()
{   
    switch(data_in)
    {   
        case UP:
          if(distanceF >= MinDist )
          { lcd.clear();
            lcd.setCursor(0,0); // top left
            lcd.print("Command :");
            lcd.setCursor(10,0); // top left
            lcd.print("F");
            lcd.setCursor(0,1); // bottom left
            lcd.print("DistF :");
            lcd.setCursor(8,1); // bottom 
            lcd.print(distanceF);
            //Serial.println(" Forward");            
            if(Duty_L < MAX_PWM_DUTY)
                Duty_L+=Step_Size;
            if(Duty_L >= MAX_PWM_DUTY)
                Duty_L = MAX_PWM_DUTY; 
                
            if(Duty_R < MAX_PWM_DUTY)
                Duty_R+=Step_Size;
            if(Duty_R >= MAX_PWM_DUTY)
                Duty_R = MAX_PWM_DUTY;

            digitalWrite(DirL,LOW);
            digitalWrite(DirR,LOW);
            analogWrite(pwmL, Duty_L); //here's how to generate PWM signal from Digital arduino pin
            analogWrite(pwmR, Duty_R); //here's how to generate PWM signal from Digital arduino pin             
//            analogWrite(pwmL, 125); //here's how to generate PWM signal from Digital arduino pin
//            analogWrite(pwmR, 125); //here's how to generate PWM signal from Digital arduino pin              
            //break;
          }
          else
          { Duty_L = 0;
            Duty_R = 0;
            lcd.clear();
            lcd.setCursor(0,0); // top left
            lcd.print("Command :");
            lcd.setCursor(10,0); // top left
            lcd.print("F+O");
            lcd.setCursor(0, 1); // bottom left
            lcd.print("DistF :");
            lcd.setCursor(8,1); // bottom 
            lcd.print(distanceF);
            //Serial.println(" Detected front obstacle..");
          
            digitalWrite(DirL,LOW);
            digitalWrite(DirR,LOW);
            analogWrite(pwmL, 0); //here's how to generate PWM signal from Digital arduino pin
            analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
            //break;
          }
          break;
        case DOWN:
          if(distanceR >= MinDist )
          { lcd.clear();
            lcd.setCursor(0,0); // top left
            lcd.print("Command :");
            lcd.setCursor(10,0); // top left
            lcd.print("B");
            lcd.setCursor(0,1); // bottom left
            lcd.print("DistB :");
            lcd.setCursor(8,1); // bottom 
            lcd.print(distanceR);
            //Serial.println(" Backward");
            
            if(Duty_L < MAX_PWM_DUTY)
                Duty_L+=Step_Size;
            if(Duty_L >= MAX_PWM_DUTY)
                Duty_L = MAX_PWM_DUTY; 
                
            if(Duty_R < MAX_PWM_DUTY)
                Duty_R+=Step_Size;
            if(Duty_R >= MAX_PWM_DUTY)
                Duty_R = MAX_PWM_DUTY;
              
             digitalWrite(DirL,HIGH);
             digitalWrite(DirR,HIGH);
             analogWrite(pwmL, Duty_L); //here's how to generate PWM signal from Digital arduino pin
             analogWrite(pwmR, Duty_R); //here's how to generate PWM signal from Digital arduino pin
            //break;
          }
          else
          { Duty_L = 0;
            Duty_R = 0;
            lcd.clear();
            lcd.setCursor(0,0); // top left
            lcd.print("Command :");
            lcd.setCursor(10,0); // top left
            lcd.print("B+0");
            lcd.setCursor(0,1); // bottom left
            lcd.print("DistB :");
            lcd.setCursor(8,1); // bottom             
            lcd.print(distanceR);
            //Serial.println(" Detected rear obstacle..");
            digitalWrite(DirL,LOW);
            digitalWrite(DirR,LOW);
            analogWrite(pwmL, 0); //here's how to generate PWM signal from Digital arduino pin
            analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
            //break;
          }
          break;
        case LEFT:
          if(distanceF >= 20 )
          { 
            lcd.clear();
            lcd.setCursor(0, 0); // top left
            lcd.print("Command :");
            lcd.setCursor(10,0 ); // top left
            lcd.print("L");
            //Serial.println(" Turnleft");
            
            if(Duty_L < MAX_PWM_DUTY)
                Duty_L+=Step_Size;
            if(Duty_L >= MAX_PWM_DUTY)
                Duty_L = MAX_PWM_DUTY; 
             if(Duty_R < MAX_PWM_DUTY)
                Duty_R+=Step_Size;
             if(Duty_R >= MAX_PWM_DUTY)
                Duty_R = MAX_PWM_DUTY; 
                
             digitalWrite(DirL,HIGH);
             digitalWrite(DirR,LOW);
             analogWrite(pwmL, Duty_L); //here's how to generate PWM signal from Digital arduino pin
             analogWrite(pwmR, Duty_R); //here's how to generate PWM signal from Digital arduino pin 
            //break;
          }
          else
          {   lcd.clear();
              lcd.setCursor(0, 0); // top left
              lcd.print("Command :");
              lcd.setCursor(10,0 ); // top left
              lcd.print("L+O");
            
              digitalWrite(DirL,LOW);
              digitalWrite(DirR,LOW);
              analogWrite(pwmL, 0); //here's how to generate PWM signal from Digital arduino pin
              analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
              //break;
          }
          break;
        case RIGHT:
          if(distanceF >= 20 )
          { 
            lcd.clear();
            lcd.setCursor(0, 0); // top left
            lcd.print("Command :");
            lcd.setCursor(10,0 ); // top left
            lcd.print("R");
            //Serial.println(" TurnRight");
            
            if(Duty_L < MAX_PWM_DUTY)
                Duty_L+=Step_Size;
            if(Duty_L >= MAX_PWM_DUTY)
                Duty_L = MAX_PWM_DUTY; 
             if(Duty_R < MAX_PWM_DUTY)
                Duty_R+=Step_Size;
             if(Duty_R >= MAX_PWM_DUTY)
                Duty_R = MAX_PWM_DUTY;
                 
             digitalWrite(DirL,LOW);
             digitalWrite(DirR,HIGH);
             analogWrite(pwmL, Duty_L); //here's how to generate PWM signal from Digital arduino pin
             analogWrite(pwmR, Duty_R); //here's how to generate PWM signal from Digital arduino pin
             //analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
           
            //break;
          }
          else
          {
              lcd.clear();
              lcd.setCursor(0, 0); // top left
              lcd.print("Command :");
              lcd.setCursor(10,0 ); // top left
              lcd.print("R+O");
            
              digitalWrite(DirL,LOW);
              digitalWrite(DirR,LOW);
              analogWrite(pwmL, 0); //here's how to generate PWM signal from Digital arduino pin
              analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
              //break;
          }
          break;
        case STOP:
          lcd.clear();
          lcd.setCursor(0, 0); // top left
          lcd.print("Command :");
          lcd.setCursor(10,0 ); // top left
          lcd.print("STOP");
          //Serial.println(" Stop");
           if(Duty_L > 0)
                Duty_L-=Step_Size;
            if(Duty_L <= 0)
                Duty_L = 0; 
             if(Duty_R > 0)
                Duty_R-=Step_Size;
             if(Duty_R <= 0)
                Duty_R = 0;
          digitalWrite(DirL,LOW);
          digitalWrite(DirR,LOW);
          analogWrite(pwmL, Duty_L); //here's how to generate PWM signal from Digital arduino pin
          analogWrite(pwmR, Duty_R); //here's how to generate PWM signal from Digital arduino pin
          break;
        default:
          if(distanceF < MinDist || distanceR < MinDist)
          {
            lcd.clear();
            lcd.setCursor(0, 0); // top left
            lcd.print("No Command :");
            lcd.setCursor(10,0 ); // top left
            lcd.print("Detect Obstrc");
           
            digitalWrite(DirL,LOW);
            digitalWrite(DirR,LOW);
            analogWrite(pwmL, 0); //here's how to generate PWM signal from Digital arduino pin
            analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
          }
          else
          {
            lcd.clear();
            lcd.setCursor(0, 0); // top left
            lcd.print("No Command :");
           
            digitalWrite(DirL,LOW);
            digitalWrite(DirR,LOW);
            analogWrite(pwmL, 0); //here's how to generate PWM signal from Digital arduino pin
            analogWrite(pwmR, 0); //here's how to generate PWM signal from Digital arduino pin
          }
          break;
      }
}
//----------------------------------------------------------------
void CalDistanceF()
{
  digitalWrite(trigPinF, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinF, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationF = pulseIn(echoPinF, HIGH);
  // Calculating the distance
  distanceF = durationF * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  //Serial.print("Distance in front: ");
  //Serial.print(distanceF);
  //Serial.println(" cm");
}
//----------------------------------------------------------------
void CalDistanceR()
{
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationR = pulseIn(echoPinR, HIGH);
  // Calculating the distance
  distanceR = durationR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  //Serial.print("Distance in rear: ");
  //Serial.print(distanceR);
  //Serial.println(" cm");
}
//----------------------------------------------------------------
void LCDdisplay()
{/*
  // Turn off the cursor:
  lcd.noCursor();
  delay(100);
  // Turn on the cursor:
  lcd.cursor();
  delay(100);  
  */
}
