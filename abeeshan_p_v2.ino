// Program By:     Abeeshan Poologarajah, 500845623
// Course:         MEC 830 Section 04
// Project:        Project #1 - 2-DOF Solar Tracker Robot Arm
// Date:           October 29, 2021

// Description:  Using 2 photoresistors, find azimuth and altitude for max light intensity

 
#include <Servo.h>
Servo myservo;

#define stepPin 6
#define dirPin 7 

#define servoPin 10

#define pr1Pin A0
#define pr2Pin A1

double pr1_val, pr2_val; // photoresistor light reading values
int delay_time = 50000;  // stepper motor delay time between each step it takes

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT);
  myservo.attach(servoPin);
  myservo.write(90); 
}

void loop(){
  azimuth();          // find ideal azimuth position by finding when the difference b/w the photoresistor values is 0
  delay(500);   
  altitude();         // find ideal altitude position by finding the servo angle position where light intensity is max
  myservo.write(90);  // after finding correct azimuth and altitude position, reset altitude back to 90 deg 
}

// solar tracker reads the photoresistor difference, compares difference with minimum difference until it hits 0 
void azimuth() {
  double min_difference = 1000;   //arbitary large value
  double current_difference;

  // dead while loop so that stepper turns until min_difference = 0 (when both photoresistors are reading the same value)
  while(1){
    take_step();
    current_difference = read_and_direction();
    Serial.println(current_difference);
    
    if (current_difference < min_difference){
      min_difference = current_difference;    // updates minimum difference
    }
    if (min_difference == 0){
      Serial.println(min_difference);
      break;  // break out of loop after ideal azimuth position is found
    }
  }
}

// sweeps servo from 0 deg to 135 deg and finds ideal position where max light intensity occurs by only reading one of the photoresistors
void altitude(){
  double max_intensity = 0 ;  // arbitray min value
  double current_intensity;
  int ideal_pos;

  for (int pos = 0; pos <= 135; pos++){
    myservo.write(pos);
    current_intensity = analogRead(pr1Pin);;
    Serial.println(current_intensity);
    if (current_intensity > max_intensity){
      max_intensity = current_intensity;  // update max light intensity value
      ideal_pos = pos;                    // update ideal pos for corresponding max light intensity value
    }
    delay(15);  // delay between each servo angle increment
  }
   
   myservo.write(ideal_pos);  // once servo sweeps and finds ideal position, servo should now go to that ideal pos
   delay(2000);               // wait 2 seconds before solar tracker starts to try to find new "sun" position
}

// reads photoresistor values, sets motor direction (CW/CCW), and returns the absolute value of the difference b/w the photoresistor light values
double read_and_direction(){
  pr1_val = analogRead(pr1Pin);
  pr2_val = analogRead(pr2Pin);

  if (pr1_val >= pr2_val){
     digitalWrite(dirPin, HIGH); // Set motor direction CW
  }
  if (pr1_val < pr2_val){
     digitalWrite(dirPin, LOW); // Set motor direction CW
  }
  double difference = abs(pr1_val - pr2_val);
  return difference;
}

// stepper motor takes 1 step in the direction determined in the "read_and_direction" function
void take_step(){
  digitalWrite(stepPin, HIGH); 
  delayMicroseconds(delay_time); 
  digitalWrite(stepPin, LOW); 
  delayMicroseconds(delay_time);
  //delay(50);
}
