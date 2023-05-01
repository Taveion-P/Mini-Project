//Team Super Awesome Mega Cool Mini Project
//4/24/2023

//include Wire.h and address for easy i2c communication between the arduino and pi
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
//the arduino by default will use A4 and A5 for sending and recieving data.

//*****
//pin out definitions for where everything should be connected to.
//*****

//encoder pins for left (1) and right (2) encoders
#define CLK_A 2
#define DT_B 5

#define CLK_A2 3
#define DT_B2 6

//motor pins
#define M1PWM 9
#define M2PWM 10
#define M1DIR 7 
#define M2DIR 8
#define nD2 4

#define i2cdebug 1

//pin for a button that can reset the current motor position to be the starting zero position.
#define donePin 11

#define maxPWMstraight 200
#define maxPWM 255
#define minPWM 50
//*****
//creates global variables that will be used in the future.
//*****

//global variables for left (1st) encoder
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

//global variables for right (2nd) encoder.
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

//defines constant variables that will be used in the future.
double const radius=0.05;
double const b = 0.1;
double const rotations = 2.3; //2.522 was calculated roation ratio

//defines and sets starting positions for the robot.
double posX=0;
double posY=0;
double theta=0;

byte sentAngle = 0;
byte distance = 0;
byte quadrant = 0;
byte decimalVal = 0;
int trueAngle = 256;
float trueDistance = 0;
float distanceToMarker=0;

//setpoint that is read over i2c communcation that will tell the motor which position to move to. (will range from 1-4)
byte setpoint = 0;


//*****
//transfer function values
//*****

float previousError1 = 0;
float previousError2 = 0;
float integralError1 = 0;
float integralError2 = 0;
float derivativeError1 = 0;
float derivativeError2 = 0;

//use previous micros starting at zero to serve as a starting point for the sample rate of the main program
int interval = 5000; //use as the sample rate (the amount of time that the program should wait between sampling values)

//use the encoder code to repeatedly calculate a value for the angular position of the wheels
double previousAngularPosition1 = 0;
double currentAngularPosition1 = 0;

double previousAngularPosition2 = 0;
double currentAngularPosition2 = 0;

//the targetAngularPosition will be changed via the Aruco camera to 0, pi/2, pi, or 3pi/2
double targetAngularPosition1 = 0;
double targetAngularPosition2 = 0;

//the targetOfRotation will be input via code before demonstration
double targetOfRotation = 0;

double angularVelocity1 = 0;
double angularVelocity2 = 0;
//use the previous angularPosition and compare with the current angular position as well as the interval rate (time over which the position has changed)
//in order to determine the angularVelocity of the wheel

unsigned long previousmicros = 0;

float ut1;
float ut2;

float changeInTime;
int pwr1;
int pwr2;

int dir1;
int dir2;

float error1;
float error2;

//Constants for the controller (change to best fit the system)
float Kp1 = 1;
float Kd1 = 0;
float Ki1 = 5;

float Kp2 = 1;
float Kd2 = 0;
float Ki2 = 5;

int markerCount = 0;

enum states {
    initial, //spin robot until it sees aruco marker
    rotateToMarker, //reached when camera detects marker
    moveToMarker, //reached when robot is at correct angle of aruco marker
    final, //final end state to stop and do nothing
    finalFinal  //the actual final state after Jopherie moves to all the markers
  };
  
states currentState = initial;

void setup() {

  // Starts Serial to be able to print out and read encoder values
  Serial.begin(115200);

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  //define callbacks for i2c communication
  Wire.onReceive(receiveData);
  //Wire.onRequest(sendData);
  Serial.println("Ready!");


  //defines the different pins on the Arduino as inputs to read the values of the encoder.
  //pin for a button that can reset the current motor position to be the starting zero position.
  pinMode(donePin, OUTPUT);
  
  //pinmode for both motor encoders
  pinMode(CLK_A, INPUT);
  pinMode(DT_B, INPUT);
  pinMode(CLK_A2, INPUT);
  pinMode(DT_B2, INPUT);

  //pinmode for the motor and motor shield
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(nD2, OUTPUT);

  //immediately set D2 to high to allow motor to be moved
  digitalWrite(nD2, HIGH);

  //defines Alast which will immediently be used in the loop for monitoring the count rotation state of the encoder.
  Alast = digitalRead(CLK_A);
  Alast2 = digitalRead(CLK_A2);

  //attachs the interrupt function
  //uses 0 or 1 for either the 2nd or 3rd pin interrupt, the function assossiacted with the interupt, and change in A.
  attachInterrupt(0,A_ISR , CHANGE);
  attachInterrupt(1,A_ISR2, CHANGE);

  //set the starting times to the current time
  time1=micros();
  time2=micros();

  //prints out the starting position values and time
  //output();   

  //main code to make Jopherie movtryne in a certain direction and/or angle
}





void loop() {
  switch(currentState){
    case initial: //spin robot until it sees aruco marker
      delay(50);
      //turnAngle(10);
      if(trueAngle != 256){
        currentState = rotateToMarker;
        powerMotor1(1,0);
        powerMotor2(1,0);
        Serial.println("Heree");
        break;
      }
      delay(50);
      powerMotor1(1,64);
      powerMotor2(1,64);
      //Serial.println("scanning");
      //Serial.println(trueAngle);
      if(trueAngle != 256){
        currentState = rotateToMarker;
        powerMotor1(1,0);
        powerMotor2(1,0);
        Serial.println("There");
      }
    break;

    case rotateToMarker:  //reached when camera detects marker
      Kp1 = 1;
      Kd1 = 0;
      Ki1 = 7.5;
      
      Kp2 = 1;
      Kd2 = 0;
      Ki2 = 7.5;
      Serial.println(trueAngle);
      delay(1500);
      count = 0;
      count2 = 0;
      if(abs(trueAngle) <= 1){
        Serial.println("Angle Reached");
        currentState = moveToMarker;
        break;
      }else{
        Serial.println("at incorrect angle");
      }
      Serial.println("Moving To TrueAngle");
      Serial.println(trueAngle);
      turnAngle(trueAngle);
    break;

    case moveToMarker: //reached when robot is at correct angle of aruco marker
      Kp1 = 1;
      Kd1 = 0;
      Ki1 = 5;
      
      Kp2 = 1;
      Kd2 = 0;
      Ki2 = 5;
      Serial.println("Moving Forward");
      distanceToMarker = trueDistance;
      Serial.println(distanceToMarker);
      moveStraight(trueDistance*0.9);
      currentState = final;
    break;

    case final: //final state reverse backwards the same distance traveled forwards and then change to the initial state
      //Serial.println("Moving Backward");
      //Serial.println(-1*distanceToMarker);
      
      digitalWrite(donePin, HIGH);
      sentAngle = 0;
      quadrant = 0;
      distance = 0;
      decimalVal = 0;
      trueAngle=256;
      trueDistance=0;
      delay(1000);
      digitalWrite(donePin, LOW);
      //delay(2000);
      //moveStraight(-1*distanceToMarker);
      delay(4000);

      markerCount++;
      if (markerCount >= 6) {
        currentState = finalFinal;
      }else{
        currentState = initial;
      }
    break;
     
    case finalFinal:
        powerMotor1(0,0);
        powerMotor2(0,0);
        delay(999999999999999999);
    break;

    
    default: //default is to go back to initial state
      currentState = initial;
    break;
  }
}

void A_ISR(){
  //This is the ISR function that triggers anytime that there is a change in A.
  //Serial.println("In ISR");
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

double toRadians(int count){
  return count*2*PI/1600.0;
}

float radToDeg(float rad){
  return rad*180.0/PI;
}

float degToRad(float deg){
  return deg*PI/180.0;
}

//used to apply power and direction to a dc motor connected to the arduino
void powerMotor1(int direction, int setPWM){

  //use analog write to set the PWM value of the motor
  analogWrite(M1PWM, setPWM);

  //now based off the direction input (1 for forward !1 for backward) apply the voltage to the motor
  if(direction == 1){
    //motor moves wheel forward
    digitalWrite(M1DIR, LOW);
  } 
  else if(direction == -1){ 
    //motor moves wheel backward
    digitalWrite(M1DIR, HIGH);
  }

}

void powerMotor2(int direction, int setPWM){

  //use analog write to set the PWM value of the motor
  analogWrite(M2PWM, setPWM);

  //now based off the direction input (1 for forward !1 for backward) apply the voltage to the motor
  if(direction == 1){
    //motor moves wheel forward
    digitalWrite(M2DIR, HIGH);
  } 
  else if(direction == -1){ 
    //motor moves wheel backward
    digitalWrite(M2DIR, LOW);
  }

}

void moveStraight(double distance){
  count =0;
  count2=0;

  int poop=0;
  
  targetAngularPosition1 = -1*(distance/(5.8*PI/12.0))*2*PI;
  targetAngularPosition2 = -1*targetAngularPosition1;

  currentAngularPosition1 = toRadians(count);
  currentAngularPosition2 = toRadians(count2);

  error1 = targetAngularPosition1 - currentAngularPosition1;
  error2 = targetAngularPosition2 - currentAngularPosition2;

  //integral of the error
  integralError1 = integralError1 + error1*changeInTime;
  integralError2 = integralError2 + error2*changeInTime;

  //computation of the integral and derivative terms u(t) using the previously defined values
  ut1 = Kp1*error1 + Ki1*integralError1;
  ut2 = Kp2*error2 + Ki2*integralError2;

  //power the motor with speed and direction!
  //we want the PWM to be our control signal u(t), since PWM is always between 0 and 255 take the abs(ut);
  pwr1 = fabs(ut1 * maxPWMstraight);
  pwr2 = fabs(ut2 * maxPWMstraight);
  //pwr cannot exceed 255

  if(pwr1 > maxPWMstraight)
    pwr1 = maxPWMstraight;
  if(pwr2 > maxPWMstraight)
    pwr2 = maxPWMstraight;
  if(pwr1 < minPWM)
    pwr1 = minPWM;
  if(pwr2 < minPWM)
    pwr2 = minPWM;

  boolean keepMoving = true;
  long stationaryTime=0;
 
  while(keepMoving){
    unsigned long currentmicros = micros();
   
    //using the given sampling rate only sample when the rate has occurred
    if (currentmicros - previousmicros >= interval) {
     
      //find the change in time in seconds
      changeInTime = ((float)(currentmicros - previousmicros))/(1000000);
      previousmicros = currentmicros;
 
      //reads the current angular postion in radians
      currentAngularPosition1 = toRadians(count);
      currentAngularPosition2 = toRadians(count2);
 
      //current ERROR (distance between desired pos and current pos)
      error1 = targetAngularPosition1 - currentAngularPosition1;
      error2 = targetAngularPosition2 - currentAngularPosition2;
     
      //derivative of the error (not used or needed in this iteration)
      if(changeInTime > 0){
        derivativeError1 = (error1 - previousError1)/(changeInTime);
        derivativeError2 = (error2 - previousError2)/(changeInTime);
      }

      if(currentAngularPosition1 < 0.75*targetAngularPosition1){
        integralError1 = 0;
        integralError2 = 0;
      }
     
      //integral of the error
      integralError1 = integralError1 + error1*changeInTime;
      integralError2 = integralError2 + error2*changeInTime;
 
      //computation of the integral and derivative terms u(t) using the previously defined values
      ut1 = Kp1*error1 + Ki1*integralError1;
      ut2 = Kp2*error2 + Ki2*integralError2;
 
      //power the motor with speed and direction!
      //we want the PWM to be our control signal u(t), since PWM is always between 0 and 255 take the abs(ut);
      pwr1 = fabs(ut1 * maxPWMstraight);
      pwr2 = fabs(ut2 * maxPWMstraight);
      
      //pwr cannot exceed 255
   
      if(pwr1 > maxPWMstraight)
        pwr1 = maxPWMstraight;
      if(pwr2 > maxPWMstraight)
        pwr2 = maxPWMstraight;
      if(pwr1 < minPWM)
        pwr1 = minPWM;
      if(pwr2 < minPWM)
        pwr2 = minPWM;

      //Serial.println();
      //Serial.println(pwr1);
      //Serial.println(pwr2);
      //Serial.println();
      //set an initial direction for the motor to move
      dir1 = 1;
      dir2 = 1;
      //direction is forward (1) if ut > 0 and direction is backward/reverse if ut < 0
      if(ut1 < 0)
        dir1= -1;
      if(ut2 < 0)
        dir2 = -1;
     
      //call the motor function and pass the direction the motor should move and the PWM power to be supplied
      //ideally the motor counts should be exactly the same to be moving in a straight line
      //if the robot begins to veer off, then it should self correct by moving the motor that is behind by itself.
      if(fabs(count) - fabs(count2) > 30 || fabs(count2) - fabs(count) > 30){
        if(abs(count) > abs(count2)){
            powerMotor1(dir1, 0);
            powerMotor2(dir2,64);
        }
        if(abs(count) < abs(count2)){
            powerMotor1(dir1,64);
            powerMotor2(dir2,64);
        }
      }else{
        powerMotor1(dir1, pwr1);
        powerMotor2(dir2, pwr2);
      }
      //current error becomes previous error as the loop finishes
      previousError1 = error1;
      previousError2 = error2;
 
      //outputs serveral values for debugging and simulation needs.
//      output();
//      Serial.print(error1);
//      Serial.print("\t");
//      Serial.print(error2);
//      Serial.print("\t");
//      Serial.print(pwr1);
//      Serial.print("\t");
//      Serial.println(pwr2);
 
      //at the end of our sampling check to ensure that the duration we have spent taking our sample has not exceeded our actual sample rate
      //in the case that it has exceeded the sampling rate print an error
      if(currentmicros - previousmicros >= interval){
        Serial.print("ERROR: RUN TIME HAS EXCEEDED SAMPLING TIME");
      }

      //waits untill the error is below a certain threshold to leave the loop to move onto the next action
      if(abs(error1) <=0.1 && abs(error2) <= 0.1){
          poop++;
          if(poop>=400){
            keepMoving = false;
          }
      }
             
    }
  }
  //delay 1 seconds before moving on and turn motors off
  powerMotor1(dir1, 0);
  powerMotor2(dir2, 0);
  delay(1000);
}

void turnAngle(double angle){
  count = 0;
  count2= 0;

  int poop=0;
  
  targetAngularPosition1 = degToRad(angle);
  targetAngularPosition2 = degToRad(angle);

  currentAngularPosition1 = toRadians(count);
  currentAngularPosition2 = toRadians(count2);

  error1 = targetAngularPosition1*rotations - currentAngularPosition1;
  error2 = targetAngularPosition2*rotations - currentAngularPosition2;

  //computation of the integral and derivative terms u(t) using the previously defined values
  ut1 = Kp1*error1 + Ki1*integralError1;
  ut2 = Kp2*error2 + Ki2*integralError2;

  //power the motor with speed and direction!
  //we want the PWM to be our control signal u(t), since PWM is always between 0 and 255 take the abs(ut);
  pwr1 = abs(ut1 * maxPWM);
  pwr2 = abs(ut2 * maxPWM);
  //pwr cannot exceed maxPWM or go lower than minPWM
  if(pwr1 > maxPWM)
    pwr1 = maxPWM;
  if(pwr2 > maxPWM)
    pwr2 = maxPWM;
  if(pwr1 < minPWM)
    pwr1 = minPWM;
  if(pwr2 < minPWM)
    pwr2 = minPWM;

  boolean keepMoving = true;
  long stationaryTime=0;
  
  while(keepMoving){
    unsigned long currentmicros = micros();
  
    
    //using the given sampling rate only sample when the rate has occurred
    if (currentmicros - previousmicros >= interval) {
  
      //find the change in time in seconds
      changeInTime = ((float)(currentmicros - previousmicros))/(1000000);
      previousmicros = currentmicros;
  
      //reads the current angular postion in radians
      currentAngularPosition1 = toRadians(count);
      currentAngularPosition2 = toRadians(count2);
  
      //current ERROR (distance between desired pos and current pos)
      error1 = targetAngularPosition1*rotations - currentAngularPosition1;
      error2 = targetAngularPosition2*rotations - currentAngularPosition2;
  
      //derivative of the error (not used or needed in this iteration)
      if(changeInTime > 0){
        derivativeError1 = (error1 - previousError1)/(changeInTime);
        derivativeError2 = (error2 - previousError2)/(changeInTime);
      }
  
      //integral of the error
      if(currentAngularPosition1 >= 0.1*targetAngularPosition1){
        integralError1 = integralError1 + error1*changeInTime;
        integralError2 = integralError2 + error2*changeInTime;
      }
      else{
        integralError1 = 0;
        integralError2 = 0;
      }
 
      //computation of the integral and derivative terms u(t) using the previously defined values
      ut1 = Kp1*error1 + Ki1*integralError1 + Kd1*derivativeError1;
      ut2 = Kp2*error2 + Ki2*integralError2 + Kd2*derivativeError2;
  
      //power the motor with speed and direction!
      //we want the PWM to be our control signal u(t), since PWM is always between 0 and 255 take the abs(ut);
      pwr1 = fabs(ut1 * maxPWM);
      pwr2 = fabs(ut2 * maxPWM);
      //pwr cannot exceed 255
   
      if(pwr1 > maxPWM)
        pwr1 = maxPWM;
      if(pwr2 > maxPWM)
        pwr2 = maxPWM;
      if(pwr1 < minPWM)
        pwr1 = minPWM;
      if(pwr2 < minPWM)
        pwr2 = minPWM;
      //set an initial direction for the motor to move
      dir1 = 1;
      dir2 = 1;
      //direction is forward (1) if ut > 0 and direction is backward/reverse if ut < 0
      if(ut1 < 0)
        dir1= -1;
      if(ut2 < 0)
        dir2 = -1;
      
      //call the motor function and pass the direction the motor should move and the PWM power to be supplied
//      if(fabs(count) - fabs(count2) > 30 || fabs(count2) - fabs(count) > 30){
//        if(abs(count) > abs(count2)){
//            powerMotor1(dir1, 0);
//            powerMotor2(dir2,32);
//        }
//        if(abs(count) < abs(count2)){
//            powerMotor1(dir1,32);
//            powerMotor2(dir2,0);
//        }
//      }else{
        powerMotor1(dir1, pwr1);
        powerMotor2(dir2, pwr2);
      
  
      //current error becomes previous error as the loop finishes
      previousError1 = error1;
      previousError2 = error2;
  
      //outputs serveral values for debugging and simulation needs.
//      output();
//      Serial.print(error1);
//      Serial.print("\t");
//      Serial.print(error2);
//      Serial.print("\t");
//      Serial.print(pwr1);
//      Serial.print("\t");
//      Serial.println(pwr2);
  
      //at the end of our sampling check to ensure that the duration we have spent taking our sample has not exceeded our actual sample rate
      //in the case that it has exceeded the sampling rate print an error
      if(currentmicros - previousmicros >= interval){
        Serial.print("ERROR: RUN TIME HAS EXCEEDED SAMPLING TIME");
      }

      //waits untill the error is below a certain threshold to leave the loop to move onto the next action
      if(abs(error1) <=0.1 && abs(error2) <= 0.1){
          poop++;
          if(poop>=250){
            keepMoving = false;
          }
      }
      
    }
  }
  //delay 1 seconds before moving on and turn motors off
  powerMotor1(dir1, 0);
  powerMotor2(dir2, 0);
//  delay(1000);
}

//function to recieve data from pi
void receiveData(int byteCount) {

if (Wire.available ()) Wire.read();
if (Wire.available ()) sentAngle = Wire.read();
if (Wire.available()) quadrant = Wire.read();
if (Wire.available()) distance = Wire.read();
if (Wire.available()) decimalVal = Wire.read();

  if (quadrant == 1 || quadrant == 4){
      trueAngle = 0 + int(sentAngle);
  } else {
    trueAngle = 0 - int(sentAngle);
  }
  
  trueDistance = float(distance) + (float(decimalVal)/100.0);

//  Serial.println("I2C");
//  Serial.print(byteCount);
//  Serial.println(" Bytes");
  Serial.println(sentAngle);
//  Serial.println(quadrant);
//  Serial.println(distance);
//  Serial.println(decimalVal);
}


// callback for sending data
//void sendData(){
//  byte low = byte(count); //giving variable low first 8 bits to send
//  byte high = byte(count >> 8); //giving variable high second 8 bits
//  Wire.write(high); //sends high 8 bits
//  Wire.write(low); //sends low 8 bits
//}
