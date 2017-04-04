// This Arduino example demonstrates bidirectional operation of a 
// 28BYJ-48, using a ULN2003 interface board to drive the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the 
// operating Frequency is 100pps. Current draw is 92mA. 
////////////////////////////////////////////////

//Using NewPing library http://playground.arduino.cc/Code/NewPing#Download
//Download and add to {Progarm Files}\Arduino\libraries\NewPing
#include <NewPing.h>

//Left motor uses analog pins because digital pins was not enough
//DigitalWrite method can also be used on analog pins
int leftMotorPin1 = A0;    
int leftMotorPin2 = A1;    
int leftMotorPin3 = A2;   
int leftMotorPin4 = A3;   

//Right motor uses digital pins
int rightMotorPin1 = 8;   
int rightMotorPin2 = 9;   
int rightMotorPin3 = 10;  
int rightMotorPin4 = 11;  
//28BYJ-48 pin 5 (VCC)
                          
int motorSpeed = 1; //simple delay between steps
int mottorSteps[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
enum Motors {LeftMotor, RightMotor};

NewPing left(7, 6, 200);
NewPing front(13, 12, 200);
NewPing right(5, 4, 200);

int leftDist = 0; 
int frontDist = 0; 
int rightDist = 0; 

long previousMillis;

enum DriveModes {AutoDrive, Folow, Manual, Stopped};
DriveModes driveMode = AutoDrive;

//---------------------------------------------------------------
//Setup part for inputs, outputs setup (runs single time)
void setup() 
{
  //declare the motor pins as outputs
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(leftMotorPin3, OUTPUT);
  pinMode(leftMotorPin4, OUTPUT);
  
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(rightMotorPin3, OUTPUT);
  pinMode(rightMotorPin4, OUTPUT);
  
  Serial.begin(9600);
}
//---------------------------------------------------------------
//Main application loop (runs forewer)
void loop()
{
  if (driveMode == AutoDrive)
  { 
    long currentMillis = millis();
    if(currentMillis - previousMillis >= 1500)
    {
      ReadSensors(); 
      previousMillis = currentMillis;
    }
    Drive(leftDist, frontDist, rightDist);
  }
}
//---------------------------------------------------------------
//Reads 3 ultrasonic (HC-SR04) sensors
void ReadSensors()
{
    int leftTemp = left.ping_cm();
    Serial.print("Left: "); Serial.println(leftTemp);
    if (leftTemp != 0)
      leftDist = leftTemp;
      
    int frontTemp = front.ping_cm();
    Serial.print("Front: "); Serial.println(frontTemp);
    if (frontTemp != 0)
      frontDist = frontTemp;
      
    int rightTemp = right.ping_cm();
    Serial.print("Right: "); Serial.println(rightTemp);
    if (rightTemp != 0)
      rightDist = rightTemp;
}
//---------------------------------------------------------------
//Main driving method depending on passed distance
void Drive(int leftDistance, int frontDistance, int rightDistance)
{
    //Conner Stuck Scenario (go back a bit)
    if(leftDistance < 6 || rightDistance < 6)
    {
      long stuckTime = millis();
      while (1)
      {
        MoveBack();
        if(stuckTime - millis() >= 4000)
          break;
      }
    }

    if (leftDistance > 15 && rightDistance > 15 && frontDistance > 15)
        MoveForward();
    else if (leftDistance > rightDistance)
        TurnAxisLeft();
    else
        TurnAxisRight();
} 
//---------------------------------------------------------------
//Move both stepper motors forward
void MoveForward()
{
  for(int i = 7; i >= 0; i--)
  {
    setLeftMotorOutput(i);
    setRightMotorOutput(i);
    delay(motorSpeed);
  }
}
//---------------------------------------------------------------
//Move both stepper motors backward
void MoveBack()
{
  for(int i = 0; i < 8; i++)
  {
    setLeftMotorOutput(i);
    setRightMotorOutput(i);
    delay(motorSpeed);
  }
}
//---------------------------------------------------------------
//Turn left (turns by own axis)
void TurnAxisLeft()
{
  for(int i = 7; i >= 0; i--)
  {
    //setRightMotorOutput(i);
    digitalWrite(leftMotorPin1, bitRead(mottorSteps[i], 0));
    digitalWrite(leftMotorPin2, bitRead(mottorSteps[i], 1));
    digitalWrite(leftMotorPin3, bitRead(mottorSteps[i], 2));
    digitalWrite(leftMotorPin4, bitRead(mottorSteps[i], 3));
    
    digitalWrite(rightMotorPin1, bitRead(mottorSteps[i], 0));
    digitalWrite(rightMotorPin2, bitRead(mottorSteps[i], 1));
    digitalWrite(rightMotorPin3, bitRead(mottorSteps[i], 2));
    digitalWrite(rightMotorPin4, bitRead(mottorSteps[i], 3));
    
    delay(motorSpeed);
  }
}
//--------------------------------------------------------------
//Turn right (turns by own axis)
void TurnAxisRight()
{
  for(int i = 7; i >= 0; i--)
  {
    //setLeftMotorOutput(i);
    digitalWrite(leftMotorPin1, bitRead(mottorSteps[i], 3));
    digitalWrite(leftMotorPin2, bitRead(mottorSteps[i], 2));
    digitalWrite(leftMotorPin3, bitRead(mottorSteps[i], 1));
    digitalWrite(leftMotorPin4, bitRead(mottorSteps[i], 0));
    
    digitalWrite(rightMotorPin1, bitRead(mottorSteps[i], 3));
    digitalWrite(rightMotorPin2, bitRead(mottorSteps[i], 2));
    digitalWrite(rightMotorPin3, bitRead(mottorSteps[i], 1));
    digitalWrite(rightMotorPin4, bitRead(mottorSteps[i], 0));
  
    delay(motorSpeed);
  }
}
//---------------------------------------------------------------
void setLeftMotorOutput(int out)
{
    //CounterClockwise
    digitalWrite(leftMotorPin1, bitRead(mottorSteps[out], 3));
    digitalWrite(leftMotorPin2, bitRead(mottorSteps[out], 2));
    digitalWrite(leftMotorPin3, bitRead(mottorSteps[out], 1));
    digitalWrite(leftMotorPin4, bitRead(mottorSteps[out], 0));
}

void setRightMotorOutput(int out)
{
    //Clockwise
    digitalWrite(rightMotorPin1, bitRead(mottorSteps[out], 0));
    digitalWrite(rightMotorPin2, bitRead(mottorSteps[out], 1));
    digitalWrite(rightMotorPin3, bitRead(mottorSteps[out], 2));
    digitalWrite(rightMotorPin4, bitRead(mottorSteps[out], 3));
}
//---------------------------------------------------------------
