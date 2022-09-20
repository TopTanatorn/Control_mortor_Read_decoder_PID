// Encoder connect to digitalpin 2 and 3 on the Arduino.
#include<TaskScheduler.h>
volatile double counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile double prv_counter = 0;
volatile int round_of_rotation_loop = 0;
double v = 0;
float pwm = 1;
double defSpeed = 0; // 130 - -130 round/min
double pos = 0;
void Speed();
void Control();
Scheduler runner;
Task readSpeed(100, TASK_FOREVER, &Speed);
Task speedControl(100, TASK_FOREVER, &Control);
void setup() {
  Serial.begin (9600);
  
  pinMode(PC0, INPUT);           // set pin to input Encoder1
  pinMode(PC1, INPUT);           // set pin to input Encoder2
  pinMode(PA0, OUTPUT);           // set pin to input Direction 
  pinMode(PA1, OUTPUT);           // set pin to input PWM
  
  digitalWrite(PC0, HIGH);       // turn on pullup resistors
  digitalWrite(PC1, HIGH);       // turn on pullup resistors
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(PC0, ai0, RISING);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 4 on moust Arduino.
  attachInterrupt(PC1, ai1, RISING);
  runner.addTask(readSpeed);
  runner.addTask(speedControl);
  readSpeed.enable();
  speedControl.enable();
}

void loop() {
  defSpeed = -50;
  runner.execute();
}

void ai0()
{
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(PC1) == LOW)
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void ai1()
{
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(PC0) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}
void Control() {
  if (defSpeed > 0) {
    pos = 1;
    if ((pwm <= 255) && (v < defSpeed)) {
      pwm = pwm*1.05;
    }
    else if ((pwm >= 0) && (v > defSpeed)) {
      pwm = pwm*0.95;
    }
  }
  else if (defSpeed < 0) {
    pos = 0;
    if ((pwm <= 255) && (v > defSpeed)) {
      pwm = pwm*1.05;
    }
    else if ((pwm >= 0) && (v < defSpeed)) {
      pwm = pwm*0.95;
    }
  }
  
  digitalWrite(PA1, pos);
  analogWrite(PA0, pwm);
}
void Speed() {
  //  if(fabs(counter)>= 37000)
  //  {
  //    digitalWrite(8,LOW);
  //    analogWrite(9,0);
  //  }
  //  else{
  //    digitalWrite(8,LOW);
  //    analogWrite(9,200);
  //  }
  double range;
  double err;
  //   if((counter < 32000) & (prv_counter > -32768) & (counter >= - 32768)& (prv_counter <= 32768)){ // (for positive direction with deffent of sign)
  //      range = (32768 + counter) + (32768 - prv_counter);
  //      round_of_rotation_loop++;
  //      counter = 0;
  //      prv_counter = 0;
  //
  //   }
  //   else if((counter > 32000) & (prv_counter < -32000)& (counter <= 32768)& (prv_counter >= - 32768)){ //(for minus direction with deffent of sign)
  //      range = (32768 - counter) + ( - 32768 - prv_counter);
  //      round_of_rotation_loop--;
  //      counter = 0;
  //      prv_counter =0;
  //   }
  //   else{
  //      range = counter - prv_counter; // on the normal case with same of sign
  //      counter = 0;
  //      prv_counter =0;
  //
  //   }
  range = counter - prv_counter; // on the normal case with same of sign
  v = 60 * 10 * range / 37000;
  err = defSpeed - v;
  // round * 32768
  // Round
  Serial.print(round_of_rotation_loop);
  Serial.print(",");
  // Range
  Serial.print(range);
  Serial.print(",");
  // Counter
  Serial.print(counter);
  Serial.print(",");
  // Previous Counter
  Serial.print (prv_counter);
  Serial.print(",");
  // Velocity
  Serial.print (v);
  Serial.print(",");
  // Error Value
  Serial.println(err);
  //  prv_counter = counter;
  counter = 0;
  prv_counter = 0;
}
