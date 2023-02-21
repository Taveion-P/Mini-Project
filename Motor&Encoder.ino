//Nathan Wasniak 2/9/2023
//This code reads the encoder with the use of the interrupts and outputs the value every second.

//Some links that were helpful resources in writing this code:
//Interrupts: https://www.allaboutcircuits.com/technical-articles/using-interrupts-on-arduino/
//Encoders: https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
//PUllup: https://docs.arduino.cc/learn/microcontrollers/digital-pins
//Motor Control: https://github.com/pololu/dual-mc33926-motor-shield

#include "DualMC33926MotorShield.h"
 
DualMC33926MotorShield md;

//defines where the encoder wires should be connected to the Arduino.
#define CLK_A 2
#define DT_B 4

#define CLK_A2 3
#define DT_B2 5

//creates global variables that will be used in the future.
boolean movingLeft=false;
int count=0;
int A;
int B;
int Alast;
int Blast;
double velocityLeft=0;
unsigned long time1;
int deltaTimeLeft=0;
double thetaLeftNew=0;
double thetaLeftOld=0;

boolean movingRight=false;
int count2=0;
int A_2;
int B_2;
int Alast2;
int Blast2;
double velocityRight=0;
unsigned long time2;
int deltaTimeRight=0;
double thetaRightNew=0;
double thetaRightOld=0;

double radius=0.05;
double b = 0.1;

double posX=0;
double posY=0;
double theta=0;

int countsPerRotation = 800;

void setup() {
  // Starts Serial to be able to print out and read encoder values
  Serial.begin(9600);

  //defines the different pins on the Arduino as inputs to read the values of the encoder.
  pinMode(CLK_A, INPUT);
  pinMode(DT_B, INPUT);

  pinMode(CLK_A2, INPUT);
  pinMode(DT_B2, INPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);

  //defines Alast which will immediently be used in the loop for monitoring the count rotation state of the encoder.
  Alast = digitalRead(CLK_A);
  Alast2 = digitalRead(CLK_A2);

  //attachs the interrupt function
  //uses 0 or 1 for either the 2nd or 3rd pin interrupt, the function assossiacted with the interupt, and change in A.
  attachInterrupt(0,A_ISR , CHANGE);
  attachInterrupt(1,A_ISR2, CHANGE);

  //set the starting times to the current time
  time1=millis();
  time2=millis();

  md.init();

  //prints out the starting position values and time
  output();
}

void loop() {
    //continously prints out the position values every 100 ms or 0.1 seconds
    output();
    delay(100);

    //ramps up from 0 to 100 speed using the set
    for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  }


void A_ISR(){
  //This is the ISR function that triggers anytime that there is a change in A.

  //reads the current values of A and B
  A=digitalRead(CLK_A);
  B=digitalRead(DT_B);

  //checks whether the the encoder is moving clockwise or counter clockwise based on the current and past value of A and the current state of B.
  if(A != Alast){
    if(digitalRead(DT_B)!=A){
      count++;
    }else{
      count--;
    }
  }
  //updates the last value of A
  Alast=A;
  
}

void A_ISR2(){
  //This is the ISR function that triggers anytime that there is a change in A2.

  //reads the current values of A and B
  A_2=digitalRead(CLK_A2);
  B_2=digitalRead(DT_B2);
  
  //double thetaRightOld = toRadians(count2);
  //checks whether the the encoder is moving clockwise or counter clockwise based on the current and past value of A and the current state of B.
  if(A_2 != Alast2){
    if(digitalRead(DT_B2)!=A_2){
      count2++;
    }else{
      count2--;
    }
  }
  //updates the last value of A
  Alast2=A_2;

}

void output() {
  //grabs the new current theta position for each wheel in radians
  thetaLeftNew = toRadians(count);
  //thetaRightNew = toRadians(count2);
  
  //Updates the left and right wheel velocities every time the output function is called, giving a good average velocity
  //Before doing this it checks to see if there is a time difference as not to produce an error or infinity value when dividing by 0
  if(millis()-time1 != 0)
  velocityLeft = radius * 1000.0*(thetaLeftNew - thetaLeftOld)/(millis()-time1);

  //if(millis()-time2 != 0)
  //velocityRight = radius * 1000.0*(thetaRightNew - thetaRightOld)/(millis()-time2);

  //updates the last time the output was called and the moves the newly read theta value into the old theta value
  time1 = millis();
  //time2 = millis();
  thetaLeftOld = thetaLeftNew;
  //thetaRightOld = thetaRightNew;
  
  //calculates the X, Y, and phi positions using the given equations 
  //posX = posX + cos(theta)*(velocityLeft + velocityRight)/2.0;
  //posY = posY + sin(theta)*(velocityLeft + velocityRight)/2.0;
  //theta = theta + ( 1/b*(velocityLeft - velocityRight) );

  //prints out all the associated values to the terminal
  Serial.print( millis() );
  Serial.print("\t");
  //Serial.print(velocityLeft,2);
  //Serial.print("\t");
  Serial.print(velocityRight,2);
  Serial.print("\t");
  //Serial.print(posX,3);
  //Serial.print("\t");
  //Serial.print(posY,3);
  //Serial.print("\t");  
  //Serial.print(theta,3);
  //Serial.print("\t");  
  Serial.println(count);
  //Serial.print("\t");  
  //Serial.println(count2);
  
  
}

//Function that takes in the current count position of the encoder and returns the angle in radians
double toRadians(int count){
  return count*6.28318530718/countsPerRotation;
}

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}
