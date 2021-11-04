#include <Stepper.h>
#include <Servo.h>
const int p1 = 0;
const int p2 = 1;
const int servo_pin = 7;
const int led = 5;
 
//stepper motor parameter
const int step = 8; 
const int spd = 13; 

int angle = 90;

//mode switching parameter
int event = 0;
const int tolerance = 10;
bool tracking = true;
int event_wait_time = 500;

Servo servo;
Stepper stepper = Stepper(2048, 8, 10, 9, 11); //In1, In3, In2, In4

void setup() {
  stepper.setSpeed(spd);
  servo.attach(servo_pin);
  servo.write(angle);
  pinMode(led, OUTPUT);
}

void loop() {
  //Stepper tracking
  if (tracking)
  {
    if (analogRead(p1) - analogRead(p2) > tolerance)
    {
      stepper.step(-step);
    } else if (analogRead(p2) - analogRead(p1) > tolerance)
    {
      stepper.step(step);
    } else {
      //no change in light levels
      event += 1;
    }

    if (event > event_wait_time)
    {
      angle = returnPitch();
      servo.write(angle);
      //turn off tracking
      tracking = false;
      digitalWrite(led,LOW);
      event = 0;
    }
  }else if (!tracking)
  {
     // looking for change in light level
    if (abs(analogRead(p1) - analogRead(p2)) > 2*tolerance)
    {
      event += 1;
    } 
    if (event > event_wait_time)
    {
      tracking = true;
      digitalWrite(led,HIGH);
      event = 0;
    }
  }
}

int returnPitch(){
  int maxLi = 0;
  int Li;
  int maxPitch;

  //test response with smaller stepps (5 and 10)
  for (int i = 40; i <= 130; i += 10)
  {
    servo.write(i);
    Li = analogRead(p1) + analogRead(p2);
    if (Li > maxLi){
      maxLi = Li;
      maxPitch = i;
    }
    delay(200);
  }
  return maxPitch;
}
