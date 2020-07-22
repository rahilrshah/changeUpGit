/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Ldrive1              motor         17              
// Rdrive1              motor         16              
// Rdrive2              motor         20              
// Intake1              motor         13              
// Intake2              motor         1               
// Serial               motor         15              
// Index                motor         2               
// Ldrive2              motor         19              
// middleBall           line          A               
// bottomBall           line          B               
// middleBall2          line          D               
// intakeBall           line          C               
// topBall              sonar         E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here


void reset(){
  Rdrive1.resetRotation();
  Rdrive1.resetPosition();
  Rdrive2.resetRotation();
  Rdrive2.resetPosition();
  Ldrive1.resetRotation();
  Ldrive1.resetPosition();
  Ldrive2.resetRotation();
  Ldrive2.resetPosition();
  Intake1.resetRotation();
  Intake1.resetPosition();
  Intake2.resetRotation();
  Intake2.resetPosition();
  Serial.resetRotation();
  Serial.resetPosition();
  Index.resetRotation();
  Index.resetPosition();
}

void stop(){
  Rdrive1.stop();
  Rdrive2.stop();
  Ldrive1.stop();
  Ldrive2.stop();
  Serial.stop();
  Index.stop();
  Intake1.stop();
  Intake2.stop();
}



// fwd constants
double leftkP = 1;
double rightkP = 1;
double leftkI = 0.000;
double rightkI = 0.000;
double leftkD = 0.0;
double rightkD = 0.0;
double leftIntActZone = 10.0;
double rightIntActZone = 10.0;

//strafe constants
double frontkP = 0.7;
double backkP = 0.7;
double frontkI = 0.0003;
double backkI = 0.0003;
double frontkD = 0.13;
double backkD = 0.13;
double frontIntActZone = 10.0;
double backIntActZone = 10.0;

//strafe variables
int frontError; //current - desired
int backError;
int frontPrevError = 0; //error 20 msec ago
int backPrevError = 0;
int frontDerivative; //error - prevError
int backDerivative;
int frontTotalError = 0; //error + totalError
int backTotalError = 0;
int frontIntegral;
int backIntegral;
double frontPower;
double backPower;

//fwd variables
int leftError; //current - desired
int rightError;
int leftPrevError = 0; //error 20 msec ago
int rightPrevError = 0;
int leftDerivative; //error - prevError
int rightDerivative;
int leftTotalError = 0; //error + totalError
int rightTotalError = 0;
int leftIntegral;
int rightIntegral;
double leftPower;
double rightPower;



void fwdPID(float leftDist, float rightDist, int leftTopSpeed, int rightTopSpeed, int leftLowSpeed, int rightLowSpeed, int fwdTime){
  motor_group Rdrive(Rdrive1,Rdrive2);
  motor_group Ldrive(Ldrive1,Ldrive2);
  Brain.Timer.reset();
  int leftValue = leftDist/.03490658503;
  int rightValue = rightDist/.03490658503;
  while(Brain.Timer.time(msec) <= fwdTime){
  while(((abs(Ldrive.position(degrees)) < abs(leftValue)-1) && (abs(Rdrive.position(degrees)) < abs(rightValue)-1)) || ((abs(Ldrive.position(degrees)) < abs(leftValue)-1) || (abs(Rdrive.position(degrees)) < abs(rightValue)-1))){
    int Lposition = (Ldrive1.position(degrees));
    int Rposition = (Rdrive1.position(degrees));
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    leftError = leftValue - Lposition;
    rightError = rightValue - Rposition;

    //I
    leftTotalError += leftError;
    if(abs(leftError) <= abs(leftIntActZone)){
      leftIntegral = leftTotalError*leftkI;
    }
    else{
      leftIntegral = 0;
    }

    rightTotalError += rightError;
    if(abs(rightError) <= abs(rightIntActZone)){
    rightIntegral = rightTotalError*rightkI;
    }
    else{
      rightIntegral = 0;
    }
    //D
    leftDerivative = leftError - leftPrevError;
    rightDerivative = rightError - rightPrevError;

    leftPower = (leftError*leftkP) + leftIntegral + (leftDerivative*leftkD);
    rightPower = (rightError*rightkP) + rightIntegral + (rightDerivative*rightkD);

    if(abs(leftPower) > abs(leftTopSpeed)){
      leftPower = leftTopSpeed;
    }

    if(abs(rightPower) > abs(rightTopSpeed)){
      rightPower = rightTopSpeed;
    }

    if(abs(leftPower) < abs(leftLowSpeed)){
      leftPower = leftLowSpeed;
    }

    if(abs(rightPower) < abs(rightLowSpeed)){
      rightPower = rightLowSpeed;
    }

    Ldrive.spin(forward, leftPower, rpm);
    Rdrive.spin(forward, rightPower, rpm);

    leftPrevError = leftError;
    rightPrevError = rightError;
    vex::task::sleep(20);
  }
  }
}

void strafePID(float frontDist, float backDist, int frontTopSpeed, int backTopSpeed, int frontLowSpeed, int backLowSpeed, int fwdTime){
  motor_group Fdrive(Rdrive1,Ldrive1);
  motor_group Bdrive(Rdrive2,Ldrive2);
  Brain.Timer.reset();
  int frontValue = frontDist/.03490658503;
  int backValue = backDist/.03490658503;
  while(Brain.Timer.time(msec) <= fwdTime){
  while(((abs(Rdrive1.position(degrees)) < abs(frontValue)-1) && (abs(Ldrive1.position(degrees)) < abs(frontValue)-1) && (abs(Rdrive2.position(degrees)) < abs(backValue)-1) && abs(Ldrive2.position(degrees)) < abs(backValue)-1) || ((abs(Rdrive1.position(degrees)) < abs(frontValue)-1) || (abs(Ldrive1.position(degrees)) < abs(frontValue)-1) || (abs(Rdrive2.position(degrees)) < abs(backValue)-1) || abs(Ldrive2.position(degrees)) < abs(backValue)-1)){
    int Fposition = ((Ldrive1.position(degrees)-Rdrive1.position(degrees))/2);
    int Bposition = ((Rdrive2.position(degrees)-Ldrive2.position(degrees))/2);
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    frontError = frontValue - Fposition;
    backError = backValue - Bposition;

    //I
    frontTotalError += frontError;
    if(abs(frontError) <= abs(frontIntActZone)){
      frontIntegral = frontTotalError*frontkI;
    }
    else{
      frontIntegral = 0;
    }

    backTotalError += backError;
    if(abs(backError) <= abs(backIntActZone)){
    backIntegral = backTotalError*backkI;
    }
    else{
      backIntegral = 0;
    }
    //D
    frontDerivative = frontError - frontPrevError;
    backDerivative = backError - backPrevError;

    frontPower = (frontError*frontkP) + frontIntegral + (frontDerivative*frontkD);
    backPower = (backError*rightkP) + backIntegral + (backDerivative*backkD);

    if(abs(frontPower) > abs(frontTopSpeed)){
      frontPower = frontTopSpeed;
    }

    if(abs(backPower) > abs(backTopSpeed)){
      backPower = backTopSpeed;
    }

    if(abs(frontPower) < abs(frontLowSpeed)){
      frontPower = frontLowSpeed;
    }

    if(abs(backPower) < abs(backLowSpeed)){
      backPower = backLowSpeed;
    }

    Rdrive1.spin(forward, -frontPower, rpm);
    Ldrive1.spin(forward, frontPower, rpm);
    Rdrive2.spin(forward, backPower, rpm);
    Ldrive2.spin(forward, -backPower, rpm);

    frontPrevError = frontError;
    backPrevError = backError;
    vex::task::sleep(20);
  }
  }
  Fdrive.stop();
  Bdrive.stop();
}

void leftPID(float leftDist, int leftTopSpeed, int leftLowSpeed, int fwdTime){
  motor_group Rdrive(Rdrive1,Rdrive2);
  motor_group Ldrive(Ldrive1,Ldrive2);
  Brain.Timer.reset();
  int leftValue = leftDist/.03490658503;
  while(Brain.Timer.time(msec) <= fwdTime){
  while(((abs(Ldrive.position(degrees)) < abs(leftValue)-1))){
    int Lposition = (Ldrive1.position(degrees));
    int Rposition = (Rdrive1.position(degrees));
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    leftError = leftValue - Lposition;

    //I
    leftTotalError += leftError;
    if(abs(leftError) <= abs(leftIntActZone)){
      leftIntegral = leftTotalError*leftkI;
    }
    else{
      leftIntegral = 0;
    }
    
    //D
    leftDerivative = leftError - leftPrevError;

    leftPower = (leftError*leftkP) + leftIntegral + (leftDerivative*leftkD);

    if(abs(leftPower) > abs(leftTopSpeed)){
      leftPower = leftTopSpeed;
    }

    if(abs(leftPower) < abs(leftLowSpeed)){
      leftPower = leftLowSpeed;
    }

    Ldrive.spin(forward, leftPower, rpm);;

    leftPrevError = leftError;
    vex::task::sleep(20);
  }
  }
}

void RightPID(float rightDist, int rightTopSpeed, int rightLowSpeed, int fwdTime){
  motor_group Rdrive(Rdrive1,Rdrive2);
  motor_group Ldrive(Ldrive1,Ldrive2);
  Brain.Timer.reset();
  int rightValue = rightDist/.03490658503;
  while(Brain.Timer.time(msec) <= fwdTime){
  while(((abs(Rdrive.position(degrees)) < abs(rightValue)-1)) ){
    int Rposition = (Rdrive1.position(degrees));
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    rightError = rightValue - Rposition;

    //I
    rightTotalError += rightError;
    if(abs(rightError) <= abs(rightIntActZone)){
    rightIntegral = rightTotalError*rightkI;
    }
    else{
      rightIntegral = 0;
    }
    //D
    rightDerivative = rightError - rightPrevError;

    rightPower = (rightError*rightkP) + rightIntegral + (rightDerivative*rightkD);

    if(abs(rightPower) > abs(rightTopSpeed)){
      rightPower = rightTopSpeed;
    }

    if(abs(rightPower) < abs(rightLowSpeed)){
      rightPower = rightLowSpeed;
    }

    Rdrive.spin(forward, rightPower, rpm);

    rightPrevError = rightError;
    vex::task::sleep(20);
  }
  }
}

// define your global instances of motors and other devices here
//6 200s
int Lprofile1[30] = {0,5,10,20,30,50,80,100,150,100,80,40,20,10,0,0,0,20,50,60,60,80,100,80,60,60,50,20,10,0};
int Rprofile1[30] = {0,10,20,40,70,100,100,110,150,150,180,190,200,200,190,180,150,120,80,80,80,100,80,80,80,50,30,20,10,0};
void fwdProfile1(){
  reset();
  motor_group Rdrive(Rdrive1,Rdrive2);
  motor_group Ldrive(Ldrive1,Ldrive2);
  int speed1 = 1;

  while(speed1 < 30){
    Rdrive.spin(forward,Rprofile1[speed1],rpm);
    Ldrive.spin(forward,Lprofile1[speed1],rpm);
    wait(50,msec);
    speed1++;
    if(speed1 == 18){
      Intake1.stop();
      Intake2.stop();
    }
  }
  Index.stop();
  //fwdPID(6,22,50,50, 0, 0, 20);//14
  Rdrive.stop();
  Ldrive.stop();
}

int Lprofile2[24] = {0,-10,-20,-70,-110,-150,-180,-190,-200,-200,-200,-200,-200,-190,-180,-170,-160,-150,-150,-120,-80,-30,-10,0};
int Rprofile2[23] = {0,0,0,0,0,0,0,0,0,0,-5,-10,-30,-70,-170,-120,-90,-70,-40,-20,-10,0};
void fwdProfile2(){
  reset();
  motor_group Ldrive(Ldrive1,Ldrive2);
  motor_group Rdrive(Rdrive1,Rdrive2);
  int speed1 = 1;

  while(speed1 < 24){
    Ldrive.spin(forward,Lprofile2[speed1],rpm);
    Rdrive.spin(forward,Rprofile2[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  //leftPID(-19,-50, 0, 20);//14
  Ldrive.stop();
}

int Fprofile1[27] = {0,10,20,40,70,100,130,150,150,150,150,150,150,130,100,100,100,100,100,80,80,50,50,30,30,10,0};
int Bprofile1[27] = {0,10,20,40,70,100,130,150,150,150,150,150,150,130,100,100,100,100,100,80,80,50,50,30,30,10,0};
void fwdProfile3(){
  reset();
  motor_group Fdrive(Rdrive1,Ldrive1);
  motor_group Bdrive(Rdrive2,Ldrive2);
  int speed1 = 1;

  while(speed1 < 27){
    Rdrive1.spin(forward,-Fprofile1[speed1],rpm);
    Ldrive1.spin(forward,Fprofile1[speed1],rpm);
    Rdrive2.spin(forward,Bprofile1[speed1],rpm);
    Ldrive2.spin(forward,-Bprofile1[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  Fdrive.stop();
  Bdrive.stop();
}

int Fprofile2[51] = {0,5,10,10,20,20,40,40,70,70,110,150,180,190,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,190,150,150,120,80,80,50,30,30,20,20,10,10,0};
int Bprofile2[53] = {0,5,10,10,20,20,40,40,70,70,110,150,180,190,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,190,150,150,120,80,80,50,30,30,20,20,10,10,0};
void fwdProfile4(){
  reset();
  motor_group Fdrive(Rdrive1,Ldrive1);
  motor_group Bdrive(Rdrive2,Ldrive2);
  int speed1 = 1;

  while(speed1 < 53){
    Rdrive1.spin(forward,Fprofile2[speed1],rpm);
    Ldrive1.spin(forward,-Fprofile2[speed1],rpm);
    Rdrive2.spin(forward,-Bprofile2[speed1],rpm);
    Ldrive2.spin(forward,Bprofile2[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  //strafePID(-62,-62,-50,-50,0,0,0);
  Fdrive.stop();
  Bdrive.stop();
}

int Lprofile3[23] = {0,5,10,20,40,70,110,150,170,190,200,200,200,200,190,170,150,150,120,80,30,10,0};
int Rprofile3[23] = {0,5,10,20,40,70,110,150,170,190,200,200,200,200,190,170,150,150,120,80,30,10,0};
void fwdProfile5(){
  reset();
  int speed1 = 1;

  while(speed1 < 23){
    Rdrive1.spin(forward,Rprofile3[speed1],rpm);
    Ldrive1.spin(forward,Lprofile3[speed1],rpm);
    Rdrive2.spin(forward,Rprofile3[speed1],rpm);
    Ldrive2.spin(forward,Lprofile3[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  //fwdPID(14,14,50,50,0,0,50);//14
  Rdrive1.stop();
  Ldrive1.stop();
  Rdrive2.stop();
  Ldrive2.stop();
}

int Lprofile4[15] = {0,-5,-10,-40,-70,-110,-150,-180,-150,-120,-65,-30,-10,0};
int Rprofile4[15] = {0,-5,-10,-40,-70,-110,-150,-180,-150,-120,-65,-30,-10,0};
void fwdProfile6(){
  reset();
  int speed1 = 1;

  while(speed1 < 15){
    Rdrive1.spin(forward,Rprofile4[speed1],rpm);
    Ldrive1.spin(forward,Lprofile4[speed1],rpm);
    Rdrive2.spin(forward,Rprofile4[speed1],rpm);
    Ldrive2.spin(forward,Lprofile4[speed1],rpm);
    wait(50,msec);
    speed1++;
    if(speed1 == 11){
      Intake1.spin(forward,-200,rpm);
      Intake2.spin(forward,-200,rpm);
    }
  }
  //fwdPID(-6,-6,-50,-50,0,0,20);//14
  Rdrive1.stop();
  Ldrive1.stop();
  Rdrive2.stop();
  Ldrive2.stop();
}

int Fprofile3[26] = {0,10,20,40,70,100,140,170,190,200,200,200,200,200,200,200,190,170,140,100,80,80,30,30,10,0};
int Bprofile3[26] = {0,10,20,40,70,100,140,170,190,200,200,200,200,200,200,200,190,170,140,100,80,80,30,30,10,0};
void fwdProfile7(){
  reset();
  motor_group Fdrive(Rdrive1,Ldrive1);
  motor_group Bdrive(Rdrive2,Ldrive2);
  int speed1 = 1;

  while(speed1 < 26){
    Rdrive1.spin(forward,Fprofile3[speed1],rpm);
    Ldrive1.spin(forward,-Fprofile3[speed1],rpm);
    Rdrive2.spin(forward,-Bprofile3[speed1],rpm);
    Ldrive2.spin(forward,Bprofile3[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  Fdrive.stop();
  Bdrive.stop();
}

int Lprofile5[19] = {0,-5,-10,-20,-40,-70,-110,-150,-160,-160,-160,-150,-120,-80,-80,-50,-30,-10,0};
int Rprofile5[19] = {0,5,10,20,40,70,110,150,160,160,160,150,120,80,80,50,30,10,0};
void fwdProfile8(){
  reset();
  int speed1 = 1;

  while(speed1 < 19){
    Rdrive1.spin(forward,Rprofile5[speed1],rpm);
    Ldrive1.spin(forward,Lprofile5[speed1],rpm);
    Rdrive2.spin(forward,Rprofile5[speed1],rpm);
    Ldrive2.spin(forward,Lprofile5[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  //fwdPID(-16.5,16.5,-50,50,0,0,50);//14
  Rdrive1.stop();
  Ldrive1.stop();
  Rdrive2.stop();
  Ldrive2.stop();
}

int Rprofile9[38] = {0,10,20,40,70,100,140,140,140,140,140,140,140,140,100,80,50,30,30,30,10,10,10,10,10,10,10,40,90,150,200,150,100,70,40,20,10,0};
int Lprofile9[41] = {0,10,20,40,70,100,140,160,160,160,160,160,160,160,160,160,160,160,160,160,160,100,70,40,20,10,10,10,10,40,90,150,200,200,150,100,70,40,20,10,0};
void fwdProfile12(){
  reset();
  int speed1 = 1;

  while(speed1 < 39){
    Rdrive1.spin(forward,Rprofile9[speed1],rpm);
    Ldrive1.spin(forward,Lprofile9[speed1],rpm);
    Rdrive2.spin(forward,Rprofile9[speed1],rpm);
    Ldrive2.spin(forward,Lprofile9[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  //fwdPID(-16.5,16.5,-50,50,0,0,50);//14
  Rdrive1.stop();
  Ldrive1.stop();
  Rdrive2.stop();
  Ldrive2.stop();
}

int Fprofile4[15] = {0,10,20,40,70,100,80,50,50,30,30,20,15,10,0};
int Bprofile4[15] = {0,10,20,40,70,100,80,50,50,30,30,20,15,10,0};
void fwdProfile9(){
  reset();
  motor_group Fdrive(Rdrive1,Ldrive1);
  motor_group Bdrive(Rdrive2,Ldrive2);
  int speed1 = 1;

  while(speed1 < 15){
    Rdrive1.spin(forward,-Fprofile4[speed1],rpm);
    Ldrive1.spin(forward,Fprofile4[speed1],rpm);
    Rdrive2.spin(forward,Bprofile4[speed1],rpm);
    Ldrive2.spin(forward,-Bprofile4[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  Fdrive.stop();
  Bdrive.stop();
}

int Lprofile6[19] = {0,5,10,20,40,70,110,150,180,200,200,180,150,120,80,30,10,0};
int Rprofile6[19] = {0,5,10,20,40,70,110,150,180,200,200,180,150,120,80,30,10,0};
void fwdProfile10(){
  reset();
  int speed1 = 1;
  Intake1.spin(forward,200,rpm);
  Intake2.spin(forward,200,rpm);
  while(speed1 < 19){
    Rdrive1.spin(forward,Rprofile6[speed1],rpm);
    Ldrive1.spin(forward,Lprofile6[speed1],rpm);
    Rdrive2.spin(forward,Rprofile6[speed1],rpm);
    Ldrive2.spin(forward,Lprofile6[speed1],rpm);
    wait(100,msec);
    speed1++;
    if(speed1 == 16){
      Intake1.stop();
      Intake2.stop();
    }
  }
  //fwdPID(14,14,50,50,0,0,50);//14
  Rdrive1.stop();
  Ldrive1.stop();
  Rdrive2.stop();
  Ldrive2.stop();
}

int Lprofile7[9] = {-50,-70,-110,-150,-120,-80,-30,-10,0};
int Rprofile7[9] = {-50,-70,-110,-150,-120,-80,-30,-10,0};
void fwdProfile11(){
  reset();
  int speed1 = 1;

  while(speed1 < 9){
    Rdrive1.spin(forward,Rprofile7[speed1],rpm);
    Ldrive1.spin(forward,Lprofile7[speed1],rpm);
    Rdrive2.spin(forward,Rprofile7[speed1],rpm);
    Ldrive2.spin(forward,Lprofile7[speed1],rpm);
    wait(50,msec);
    speed1++;
  }
  //fwdPID(-6,-6,-50,-50,0,0,20);//14
  Rdrive1.stop();
  Ldrive1.stop();
  Rdrive2.stop();
  Ldrive2.stop();
}

//sets the booleans used to determine if a ball is in a slot
bool slot1; 
bool slot2;
bool slot2space;
bool slot3;
bool shoot;
bool L1;
bool L2;
bool intake;
float shootSpeed; //used to oscillate the indexer when shooting

//this function just dictates how the indexer will run depending on the circumstances
void slotMove(int slot){
  if(slot == 1){ //if the input into the function is 1, this means that the ball will move to slot 1.
    if(slot1 == false){ //this is a simple if-then loop which says that if slot 1 is empty,
      Index.spin(forward,75,percent); //and the command is given to mave a ball, it will be moved to slot 1.
    }
    else{
      Index.stop();
    }
  }
  if(slot == 2){ //this says that if the input is 2,
    if(slot2 == false){ //but slot 2 is still empty,
        Index.spin(forward,75,percent); //the ball in slot 3 will move up to slot 2.
      }
      else{
      Index.stop();
      }
  }
  if(slot == 3){ //if the input is 3,
  if(slot2 == false){
    if(slot3 == false){ //and there is no ball found in slot 3,
      Index.spin(forward,75,percent); //then the ball in the robot will come down from either slot 1 or 2 and into slot 3.
    }
    else{
      Index.stop();
    }
  }
  else if(slot2 == true){
    if(slot3 == false){
      Index.spin(forward,-75,percent);
    }
    else{
      Index.stop();
    }
  }
  }
  

}

//this function just dictates how the indexer will run depending on the circumstances
void fastSlotMove(int slot){
  if(slot == 1){ //if the input into the function is 1, this means that the ball will move to slot 1.
    if(slot1 == false){ //this is a simple if-then loop which says that if slot 1 is empty,
      Index.spin(forward,600,rpm); //and the command is given to mave a ball, it will be moved to slot 1.
    }
    else{
      Index.stop();
    }
  }
  if(slot == 2){ //this says that if the input is 2,
      if(slot2 == false){ //but slot 2 is still empty,
        Index.spin(forward,600,rpm); //the ball in slot 3 will move up to slot 2.
      }
      else{
      Index.stop();
      }
  }
  if(slot == 3){ //if the input is 3,
    if(slot3 == false){ //and there is no ball found in slot 3,
      Index.spin(forward,600,rpm); //then the ball in the robot will come down from either slot 1 or 2 and into slot 3.
    }
    else{
      Index.stop();
    }
  }

}


// void shooter(){
//   if(topBall.value(percent) > 66){
//     Index.spin(forward,600,rpm);
//   }
//   else{
//     Index.stop();
//   }
// }

int ballNum = 0;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
motor_group Intake(Intake1,Intake2);
  // Intake.spin(forward,200,rpm);
  // Index.spin(forward,600,rpm);
  // fwdProfile1();
  // Intake.stop();
  // Index.stop();
  // Serial.spin(forward,200,rpm);
  // wait(20,msec);
  // Index.spinFor(500,degrees,600,rpm);
  // wait(150,msec);
  // stop();
  // fwdProfile2();
  // fwdProfile3();
  // Index.resetRotation();
  // Intake.spin(forward,200,rpm);
  // Index.spin(forward,600,rpm);
  // fwdProfile4();
  // Intake.stop();
  // Index.stop();
  // fwdProfile5();
  // Serial.spin(forward,200,rpm);
  // wait(20,msec);
  // Index.spin(forward,600,rpm);
  // Intake.spin(forward,200,rpm);
  // Rdrive2.spin(forward,200,rpm);
  // Ldrive2.spin(forward,200,rpm);
  // wait(150,msec);
  // Rdrive2.stop();
  // Ldrive2.stop();
  // fwdProfile6();
  // Serial.stop();
  // if(middleBall.value(percent) <= 68){
  //   Intake.stop();
  //   Index.stop();
  // }
  // fwdProfile7();
  // Intake.stop();
  // Index.stop();
  // fwdProfile8();
  // //fwdProfile9();
  // // fwdProfile10();
  // Intake.spin(forward,200,rpm);
  // Index.spin(forward,600,rpm);
  // fwdProfile12();
  // Serial.spin(forward,200,rpm);
  // Rdrive2.spin(forward,200,rpm);
  // Ldrive2.spin(forward,200,rpm);
  // Index.spin(forward,300,rpm);
  // wait(300,msec);
  // Rdrive2.stop();
  // Ldrive2.stop();
  // wait(600,msec);
  // Intake.stop();
  // wait(600,msec);
  // fwdProfile11();

  fwdPID(15,15,200,200,0,0,50000);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {

    //sets booleans (slot1,slot2,slot2space,slot3) equal to their respective line trackers, also sets what dictates true and false
  // if(topBall.value(percent) <= 68){
  //   slot1 = true;
  // }
  // else{
  //   slot1 = false;
  // }

  if(middleBall.value(percent) <= 68){
    slot2 = true;
  }
  else{
    slot2 = false;
  }

  if(middleBall2.value(percent) <= 68){
    slot2space = true;
  }
  else{
    slot2space = false;
  }

  if(bottomBall.value(percent) <= 68){
    slot3 = true;
  }
  else{
    slot3 = false;
  }

  if(intakeBall.value(percent) <= 69){
    intake = true;
  }
  else{
    intake = false;
  }

  if(Controller1.ButtonL1.pressing() && L1 == 0){
    L1 = 1;
  }
  else if(Controller1.ButtonL1.pressing() && L1 == 1){
    L1 = 0;
  }

 //this code is basically the same as tank drive would be on a regular drive train, but strafing is added on one of the axes.
   int Lback2 = (Controller1.Axis3.position()^3)  - (Controller1.Axis4.position()^3);
   int Lback = Lback2;
   if(abs(Lback2) < 10){
     Lback = 0;
   }
   Ldrive2.spin(forward, Lback, percent);

   int Rfront2 = (Controller1.Axis2.position()^3) - (Controller1.Axis4.position()^3);
   int Rfront = Rfront2;
   if(abs(Rfront2) < 10){
     Rfront = 0;
   }
   Rdrive1.spin(forward, Rfront, percent);

   int Lfront2 = (Controller1.Axis4.position()^3) + (Controller1.Axis3.position()^3);
   int Lfront = Lfront2;
   if(abs(Lfront2) < 10){
     Lfront = 0;
   }
   Ldrive1.spin(forward, Lfront, percent);

   int Rback2 = (Controller1.Axis2.position()^3) + (Controller1.Axis4.position()^3);
   int Rback = Rback2;
   if(abs(Rback2) < 10){
     Rback = 0;
   }
   Rdrive2.spin(forward, Rback, percent);


//  //run intakes forward on button R2
// if(Controller1.ButtonR2.pressing()){
//      Intake1.setVelocity(200,rpm);
//      Intake2.setVelocity(200,rpm);
//      Intake1.spin(forward);
//      Intake2.spin(forward);
// }
// //run intakes backward on button R1
// else if(Controller1.ButtonR1.pressing()){
//      Intake1.setVelocity(-200,rpm);
//      Intake2.setVelocity(-200,rpm);
//      Intake1.spin(forward);
//      Intake2.spin(forward);
// }
// else if(intake == true){
//    Intake1.spin(forward,20,rpm);
//    Intake2.spin(forward,20,rpm);
//  }
// else{
//   Intake1.stop();
//   Intake2.stop();
// }

// if(Controller1.ButtonL2.pressing()){//this says that when L2 is pressed,
//   Serial.spin(forward,200,rpm);
//   shooter();
// }
// else if(slot1 == true){
//   Index.stop();
// }
// else if(slot2space == true){ //this bit of code is saying that if a ball is found in the gap between slot 2 and 3
//   Index.spin(forward,600,rpm); //and there isn't a ball in slot 1, then the indexer should push the ball up to slot 2
// }
// else if(intake == true && slot3 == true && slot2 == false && slot1 == false){ //this part of the if loop says that if a ball is in slot 3 but no ball is in slot 2,
//   slotMove(2); //then move the ball up from slot 3 to slot 2
// }
// else if(intake == true && slot3 == true && slot2 == true && slot1 == false){ //this part of the loop says if a ball is in slots 2 and 3,
//   slotMove(1); //then move them up to slots 1 and 2
// }
// else if(slot1 == true && slot3 == true && slot2 == false){ //this part says that if a ball is in slot 1 but there isn't a ball in slot 2,
//   slotMove(2);
// }
// else if(intake == true && slot3 == false){
//   slotMove(3);
// }
// //use the up button to stop the top wheel
// else if(Controller1.ButtonL1.pressing()){
//   Serial.spin(forward,-200,rpm);
// }
// //use the left button to spin the indexer backwards
// else{
//   Index.stop();
//   Serial.stop();
// }
if(Controller1.ButtonR2.pressing() == true && Controller1.ButtonL2.pressing() == true){
  Intake1.spin(forward,200,rpm);
  Intake2.spin(forward,200,rpm);
  Serial.spin(forward,200,rpm);
  Index.spin(forward,600,rpm);
}
else if(Controller1.ButtonR1.pressing() == true && Controller1.ButtonL1.pressing() == true){
  Intake1.spin(forward,-200,rpm);
  Intake2.spin(forward,-200,rpm);
  Index.spin(forward,-600,rpm);
  Serial.spin(forward,-200,rpm);
}
else if(Controller1.ButtonR1.pressing()){
  Intake1.spin(forward,-200,rpm);
  Intake2.spin(forward,-200,rpm);
}
else if(Controller1.ButtonL1.pressing()){
  Serial.spin(forward,-200,rpm);
}
else if(Controller1.ButtonR2.pressing() == true && Controller1.ButtonL2.pressing() == false){
  Intake1.spin(forward,200,rpm);
  Intake2.spin(forward,200,rpm);
  Index.spin(forward,600,rpm);
}
else if(Controller1.ButtonR2.pressing() == false && Controller1.ButtonL2.pressing() == true){
  Serial.spin(forward,200,rpm);
  Index.spin(forward,600,rpm);
}
else{
  Intake1.stop();
  Intake2.stop();
  Index.stop();
  Serial.stop();
}


    //this little bit of code just says that when the down button is pressed, our motor temperature values will be displayed on the joystick
   if(Controller1.ButtonDown.pressing()){
     float backLefttemp = Ldrive1.temperature(fahrenheit);
     float frontLefttemp = Ldrive2.temperature(fahrenheit);
     float backRighttemp = Rdrive1.temperature(fahrenheit);
     float frontRighttemp = Rdrive2.temperature(fahrenheit);
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(2,1);
     Controller1.Screen.print(backLefttemp);
     Controller1.Screen.setCursor(1,1);
     Controller1.Screen.print( frontLefttemp);
     Controller1.Screen.setCursor(1,5);
     Controller1.Screen.print(frontRighttemp);
     Controller1.Screen.setCursor(2,5);
     Controller1.Screen.print(backRighttemp);
   }

       wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}