//Ananth Sivakumar
//500649921
#include<Servo.h>     // library for servo motor 
#include <Stepper.h>
#define Right_LDR A0 // Set the photoresistor to ANALOG pin 0
#define Left_LDR A1// set the photoresistor to ANALOG pin 1

Servo myservo;
Stepper myStepper(500,8,9,10,11); //initialize the stepper motor to these digital pins

void setup() {
  Serial.begin(9600);
  myservo.attach(5);//
  pinMode(Right_LDR,INPUT);
  pinMode(Left_LDR, INPUT);
  delay(500);
}

void loop() {
  servo_movement();
  delay(200);
  azimuth();
  delay(200);

}

void azimuth()
{
  double current_difference;
  double minimum_difference = 800;
  while(true)
  {
    myStepper.step(1);
    delay(200);
    current_difference=stepper_movement();
    Serial.print(current_difference);

    if(current_difference < minimum_difference){
      minimum_difference = current_difference;
    }
    if(minimum_difference==0)
    {
      Serial.print(minimum_difference);
      break;
    }
  }
  
}


double difference,previous_val2, previous_val1;
#define stepperSpeed 500


double stepper_movement()//this function deals with the movement of the function
{
  previous_val1 = analogRead(Left_LDR);//left sensor value
  previous_val2 = analogRead(Right_LDR);//right sensor value

  if(previous_val1 > previous_val2)
  {
    myStepper.setSpeed(stepperSpeed); //this will set the stepper motor direction to clockwise
  }
  if(previous_val2 > previous_val1)
  {
    myStepper.setSpeed(stepperSpeed);//this will set stepper motor direction to Counterclockwise
  }
  double difference = abs(previous_val1-previous_val2);//based on difference we can verify which 
  //direction motor moves so if its negative the motor should move counterclockwise and positive it moves clockwise
  return difference;
}
int pos;
double currentvalue;
void servo_movement() 
{
  double servo_previous_value=0;
  int best_lighting = 0;//any arbitrary low value so that lighitng at position can replace it
  for (int pos = 0; pos <= 180; pos++){
  myservo.write(pos); //write the position to the servo 
  currentvalue = analogRead(Left_LDR);//logically both the sensor values should be close to each other
  Serial.println(currentvalue);//need this to verify value
  if (currentvalue > servo_previous_value)
  {
    servo_previous_value = currentvalue;  //where the light will be at its max analog value
    best_lighting = pos;//position where the corresponding best analog value is                   
    }
    delay(15);  //gives time for servo to react
  } 
   myservo.write(best_lighting); //moves servo to the best position 
   delay(1200);      
}       
