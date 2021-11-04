#include <Stepper.h>
#include <Servo.h>
#include <PID_v1.h>

int RightAnalog = 0;
int LeftAnalog = 1;

const int stepsPerRevolution = 64;

Servo servo;

Stepper Step1(stepsPerRevolution, 8, 9, 10, 11);
Stepper Step2(stepsPerRevolution, 11, 10, 9, 8);

int ErrorBand = 20;
int i;
int MeasurementDelay=150;
int AltRange=3;
double LeftSensor;
double RightSensor;
double SensorAverage;
double Error;
double MaxValue=0;
int Max;
boolean Direction=false;

double Higher;
double Lower;
double Mid;

double Setpoint, Input, Output;
double Kp=1, Ki=2, Kd=1;
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
  LeftSensor = analogRead(LeftAnalog);
  RightSensor = analogRead(RightAnalog);
  Error=LeftSensor-RightSensor;
  Input=Error;
  while (Error>ErrorBand||Error<-ErrorBand){
    LeftSensor = analogRead(LeftAnalog);
    RightSensor = analogRead(RightAnalog);
    Serial.print("LeftSensor: ");
    Serial.print(LeftSensor);
    Serial.print(" RightSensor: ");
    Serial.print(RightSensor);
  
    Error=LeftSensor-RightSensor;
    Input=Error;
    Serial.print(" Error: ");
    Serial.print(Error);

    if(Error < 0){
      Input=abs(Input);
      Direction=true;
    }else{
      Direction=false;
    }
  
    PID1.Compute();
    Serial.print(" Output: ");
    Serial.print(Output);

    if (Output<ErrorBand&&Output>-ErrorBand){
      break;
    }
    int motorSpeed = map(Output, 0, 255, 0, 200);
 
    if (Direction==false) {
      Step1.setSpeed(motorSpeed);
      Step1.step(-stepsPerRevolution);
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
    SensorAverage = Measure();
    if (SensorAverage>MaxValue){
      Max=j;
      MaxValue=SensorAverage;
    }
  }
  Serial.print(" MaxAngle: ");
  Serial.print(Max);
  Serial.print(" MaxValue: ");
  Serial.print(MaxValue);
  servo.write(Max);
  i=Max;
  delay(1000);
  if (i>90){
    RightAnalog=1;
    LeftAnalog=0;
  }else{
    RightAnalog=0;
    LeftAnalog=1;
  }
}

void Altitude(){
  Serial.print(" 2");
  Mid = Measure();
  i+=AltRange;
  servo.write(i);
  delay(MeasurementDelay);
  Higher=Measure();
  i-=AltRange*2;
  servo.write(i);
  delay(MeasurementDelay);
  Lower=Measure();
  i+=AltRange;

  while(Higher>Mid+AltRange||Lower>Mid+AltRange){
    if(Higher>Mid+AltRange&&Higher>Lower){
      RaiseAltitude();
      i+=AltRange;
      servo.write(i);
      delay(MeasurementDelay);
      Higher=Measure();
    }
    if(Lower>Higher&&Lower>Mid+AltRange){
      LowerAltitude();
      i-=AltRange;
      servo.write(i);
      delay(MeasurementDelay);
      Lower=Measure();
    }
    if (Higher==Lower){
      break;
    }
  }
  if (i<AltRange){
    i=AltRange;
  }
  if (i>180-AltRange){
    i=180-AltRange;
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
  if (i>90){
    RightAnalog=1;
    LeftAnalog=0;
  }else{
    RightAnalog=0;
    LeftAnalog=1;
  }
}

void RaiseAltitude(){
  i+=AltRange;
  servo.write(i);
  delay(MeasurementDelay);
  Mid=Measure();
}

void LowerAltitude(){
  i-=AltRange;
  servo.write(i);
  delay(MeasurementDelay);
  Mid=Measure();
}

double Measure(){
  LeftSensor = analogRead(LeftAnalog);
  RightSensor = analogRead(RightAnalog);
  SensorAverage=(LeftSensor+RightSensor)/2;
  return SensorAverage;
}
