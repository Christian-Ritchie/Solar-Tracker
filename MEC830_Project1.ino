#include <Stepper.h>
#include <Servo.h>
#include <PID_v1.h>

const int stepsPerRevolution = 64;

Servo servo;

Stepper Step1(stepsPerRevolution, 8, 9, 10, 11);
Stepper Step2(stepsPerRevolution, 11, 10, 9, 8);

int stepCount = 0;
int i;
double LeftSensor;
double RightSensor;
double Error;
double MaxValue=0;
int Max;
boolean Direction=false;

double Higher;
double Lower;
double Mid;

double Setpoint, Input, Output;
double Kp=1, Ki=0.1, Kd=0.02;
PID PID1(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void setup() {
  Setpoint=0;
  servo.attach(5);
  PID1.SetMode(AUTOMATIC);
  Serial.begin(9600);
  InitialAltitude();
}

void loop() {

  Azimuth();
  Serial.println(" Azimuth");
  Altitude();
  Serial.println(" Altitude");
}

void Azimuth (){
  Serial.print(" 1");
  LeftSensor = analogRead(A1);
  RightSensor = analogRead(A0);
  Error=LeftSensor-RightSensor;
  Input=Error;
  while (Error>20||Error<-20){
    LeftSensor = analogRead(A1);
    RightSensor = analogRead(A0);
    Serial.print("LeftSensor: ");
    Serial.print(LeftSensor);
    Serial.print(" RightSensor: ");
    Serial.print(RightSensor);
  
    Error=LeftSensor-RightSensor;
    Input=Error;
    Serial.print(" Error: ");
    Serial.print(Error);
    if (Error<20&&Error>-20){
      break;
    }

    if(Error < 0){
      Input=abs(Input);
      Direction=true;
    }else{
      Direction=false;
    }
  
    PID1.Compute();
    Serial.print(" Output: ");
    Serial.print(Output);
    int motorSpeed = map(Output, 0, 255, 0, 200);
 
    if (Direction==false) {
      Step1.setSpeed(motorSpeed);
      Step1.step(-stepsPerRevolution );
    } else if (Direction==true){
      Step1.setSpeed(motorSpeed);
      Step1.step(stepsPerRevolution );
    }
  }
}

void InitialAltitude (){
  Serial.print(" 3");
  for (int j=0;j<180;j++){
    Serial.print(" j: ");
    Serial.print(j);
    servo.write(j);
    delay(10);
    RightSensor = analogRead(A0);
    if (RightSensor>MaxValue){
      Max=j;
      MaxValue=RightSensor;
    }
  }
  Serial.print(" MaxAngle: ");
  Serial.print(Max);
  Serial.print(" MaxValue: ");
  Serial.print(MaxValue);
  servo.write(Max);
  i=Max;
  delay(1000);
}

void Altitude(){
  Mid = analogRead(A0);
  i+=5;
  servo.write(i);
  delay(100);
  Higher=analogRead(A0);
  i-=10;
  servo.write(i);
  delay(100);
  Lower=analogRead(A0);
  i+=5;

while(Higher>Mid+5||Lower>Mid+5){
  if(Higher>Mid+5&&Higher>Lower){
    RaiseAltitude();
    i+=5;
    servo.write(i);
    delay(100);
    Higher=analogRead(A0);
  }
  if(Lower>Higher&&Lower>Mid+5){
    LowerAltitude();
    i-=5;
    servo.write(i);
    delay(100);
    Lower=analogRead(A0);
  }
}
  Serial.print(" Higher: ");
  Serial.print(Higher);
  Serial.print(" Mid: ");
  Serial.print(Mid);
  Serial.print(" Lower: ");
  Serial.print(Lower);
  Serial.print(" i: ");
  Serial.print(i);
  servo.write(i);
  delay(100);
}

void RaiseAltitude(){
  i+=5;
  servo.write(i);
  delay(100);
  Mid=analogRead(A0);
}

void LowerAltitude(){
  i-=5;
  servo.write(i);
  delay(100);
  Mid=analogRead(A0);
}
