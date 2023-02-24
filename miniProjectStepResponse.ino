//begin entered code from git


//defines where the encoder wires should be connected to.e
#define CLK_A 2
#define DT_B 4

#define CLK_A2 3
#define DT_B2 5

//motor pins
#define PWM 9
#define pin1 7 
#define pin2 8

//creates global variables that will be used in the future.
unsigned long previousMillis = 0;
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
double pi = 3.14159;

double posX=0;
double posY=0;
double theta=0;
// end entered code from git


//transfer function values

float previousError = 0;
float integralError = 0;


//use previous millis starting at zero to serve as a starting point for the sample rate of the main program
int interval = 5000; //use as the sample rate (the amount of time that the program should wait between sampling values)

//use the encoder code to repeatedly calculate a value for the angular position of the wheels
double previousAngularPosition = 0;
double currentAngularPosition = 0;

//the targetAngularPosition will be changed via the Aruco camera to 0, pi/2, pi, or 3pi/2
double targetAngularPosition = (pi / 2);

double angularVelocity = 0; //use the previous angularPosition and compare with the current angular position as well as the interval rate (time over which the position has changed)
//in order to determine the angularVelocity of the wheel





void setup() {
  // put your setup code here, to run once:

//begin code entered from git



// Starts Serial to be able to print out and read encoder values
  Serial.begin(9600);

  //defines the different pins on the Arduino as inputs to read the values of the encoder.
  pinMode(CLK_A, INPUT);
  pinMode(DT_B, INPUT);

  pinMode(CLK_A2, INPUT);
  pinMode(DT_B2, INPUT);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(PWM, OUTPUT);


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

  //prints out the starting position values and time
  output();   



//end code entered from git

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

  //prints out the starting position values and time
  output();

//end entered code from git






}

void loop() {
  // put your main code here, to run repeatedly:

unsigned long currentMillis = millis();

//using the given sampling rate only sample when the rate has occurred
if (currentMillis - previousMillis >= interval) {

//Constants for the controller (change to best fit the system)
float Kp = 0.1;
float Kd = 0;
float Ki = 0.02;

//find the change in time IN SECONDS 
float changeInTime = ((float)(currentMillis = previousMillis))/(1000);
previousMillis = currentMillis;

//sets the current angular position to count converted into radians
currentAngularPosition = toRadians(count);
//current ERROR (distance between desired pos and current pos)
int error = targetAngularPosition - currentAngularPosition;

//derivative of the error
float derivativeError = (error - previousError)/(changeInTime);

//integral of the error

integralError = integralError + error*changeInTime;


//below would be where the computation of the integral and derivative terms of the function u(t) would be calculated in this instance we
//only need the proportional term

float ut = Kp*error + Kd*derivativeError + Ki*integralError;

//power the motor with speed and direction!
//we want the PWM to be our control signal u, since PWM is always between 0 and 255 take the abs(ut);

float pwr = fabs(ut);
//pwr cannot exceed 255
if(pwr > 255){
  pwr = 255;
}
//set an initial direction for the motor to move
int direction = 1;
//direction is forward (1) if ut > 0 and direction is backward/reverse if ut < 0
if(ut < 0){
  direction = -1;
}

//call the motor function and pass it the direction the motor should move, the PWR to be supplied, the PWM, and the pins it will be using

powerMotor(direction, pwr, PWM, pin1);

//current error becomes previous error as the loop finishes
previousError = error;


//output the required values to the serial monitor for testing purposes
//display the following:

//current time
//motor Voltage (PWM value between 0 and 255)
//angular Velocity
output();
Serial.println(pwr);



//at the end of our sampling check to ensure that the duration we have spent taking our sample has not exceeded our actual sample rate
//in the case that it has exceeded the sampling rate print an error

if(currentMillis - previousMillis >= interval){

Serial.print("ERROR: RUN TIME HAS EXCEEDED SAMPLING TIME");

}
Serial.println("END SAMPLE PERIOD");

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
  thetaRightNew = toRadians(count2);
  
  //Updates the left and right wheel velocities every time the output function is called, giving a good average velocity
  //Before doing this it checks to see if there is a time difference as not to produce an error or infinity value when dividing by 0
  if(millis()-time1 != 0)
  velocityLeft = radius * 1000.0*(thetaLeftNew - thetaLeftOld)/(millis()-time1);

  if(millis()-time2 != 0)
  velocityRight = radius * 1000.0*(thetaRightNew - thetaRightOld)/(millis()-time2);

  //updates the last time the output was called and the moves the newly read theta value into the old theta value
  time1 = millis();
  time2 = millis();
  thetaLeftOld = thetaLeftNew;
  thetaRightOld = thetaRightNew;
  
  //calculates the X, Y, and phi positions using the given equations 
  posX = posX + cos(theta)*(velocityLeft + velocityRight)/2.0;
  posY = posY + sin(theta)*(velocityLeft + velocityRight)/2.0;
  theta = theta + ( 1/b*(velocityLeft - velocityRight) );

  //prints out all the associated values to the terminal
  Serial.print( millis() );
  Serial.print("\t");
  Serial.print(velocityLeft,2);
  Serial.print("\t");
  //Serial.print(velocityRight,2);  
  Serial.print("\t");
  //Serial.print(posX,3);
  Serial.print("\t");
  //Serial.print(posY,3);
  Serial.print("\t");  
  //Serial.print(theta,3);
  Serial.print("\t");  
  //Serial.print(count);
  Serial.print("\t");  
  //Serial.println(count2);
  
  
}

double toRadians(int count){
  return count*6.28318530718/40.0;
}

//used to apply power and direction to a dc motor connected to the arduino
void powerMotor(int direction, int setPWM, int pwm, int motor1){

//use analog write to set the PWM value of the motor
analogWrite(pwm, setPWM);

//now based off the direction input (1 for forward !1 for backward) apply the voltage to the motor

//motor moves wheel forward
if(direction == 1){
  //digitalWrite(pin1, HIGH);
  //digitalWrite(pin2, LOW);

  digitalWrite(pin1, LOW);

}
//motor moves wheel backward
else if(direction == -1){
  //digitalWrite(pin1, LOW);
  //digitalWrite(pin2, HIGH);

  digitalWrite(pin1, HIGH);

}
//motor doesn't move/apply voltage at all
else{
  //digitalWrite(pin1, LOW);
  //digitalWrite(pin2, LOW);
}

}



