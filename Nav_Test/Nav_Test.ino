/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->	http://www.adafruit.com/products/1438
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Encoder.h"


//PID Stuff
double f = 0.5;                    //frequency in Hz

//Encoder and motor globals
double GearRatio = 70;        //the gear ratio
int countsPerRev_motor = 64;  //the counts per revolution of the motor shaft
long counts = 0;               //Globally initialize encoder counts


//time variables
unsigned long t_ms = 0;
double t = 0;                 //current time
double t_old_left, t_old_right = 0;             //previous time
double dt = 0;

double Pos = 0;               //current pos
double Vel_left, Vel_right = 0;              //current velocity
double Pos_old_left, Pos_old_right = 0;          //previous pos

//CONTROL VARIABLES
double error_old_left, error_old_right = 0.0;
double Pos_des = 0;
double error_left, error_right, error = 0.0;
double dErrordt = 0;
double integError_left, integError_right = 0;
int M = 0;
float V = 0;//100000, 1, .55
double kp_left = 85;
double kd_left = .85;
double ki_left = 20;
double kp_right = 50;
double kd_right = .50;
double ki_right = 0;
double input;

SoftwareSerial myserial(10, 11); //(10, 11);        //debugMyRobot(2,3); // RX, TX
const int analogPin0 = 0;                   //Center sensor
const int analogPin1 = 1;                   //Left sensor
const int analogPin2 = 2;                   //Right sesnor
const int analogPin8 = 8;                   //Center2 sensor
const int analogPin7 = 7;                   //mag sensor

const int digitalPin22 = 22;                // LED out
const int digitalPin12 = 12;                //motorsheild input
const int digitalPin43 = 43;                //MC1
const int digitalPin42 = 42;                //Servo pin
const int digitalPin44 = 44;                //Enable for H-bridge
const int digitalPin45 = 45;                //MC2

//Constants
const double pi = 3.141592654;

//Rando stuff
const int height = 5;
int course_stage = 0;
int auto_mode = 0;                         //flag for auto_mode
int line_follower = 0;                     //flag for line_follower
int f_wall = 0;                            //flag for follow_wall
char val;                                  // Data received from the serial port
int radius = 16;
int angle = 45;
int done, right_speed, left_speed = 0;
boolean D_rection = true;
int count, inc, pause = 1;
int global_count, flag, flag1 = 0;
int index, lastIndex, wall, edge, obstacle, line = 0;
float distance = 0;
long duration;
float cm, cm_right, cm_left;

//Statemachne things...
int CS = 1;
boolean goCMD = true;

//TESTING
int test1 = 50;
int test2 = 100;                            //Do not touch!!!!!! without asking Charlie or Matt!
int test3 = 50;
int test4 = 150;

//Charlie's Stuff
int stack_ptr = 0;
float global_memory[15];
float slope = 0;

//Wheel O shit
Encoder leftEnc(18, 19);           //Motor2
Encoder rightEnc(2, 3);            //Motor1
double encoder_scale = 4480;
double wheel_circumference = 2 * pi * 2.54;
double left_counts , right_counts, left_last_count, right_last_count;

//Wheel o-shit variables
//unsigned long t_ms = 0;
double Pos_right, Pos_left; //Vel_left, Vel_right;
//double t, t_old, Pos_left, Pos_right, Vel_left, Vel_right, Pos_left_old, Pos_right_old, rads; // check if shit breaks

//Right_Tilt servo
Servo Right_Tilt;                                //Center servo pin Digital 24
Servo Left_Tilt;                               //Right servo pin Digital 23
Servo Front_Tilt;                               //Left servo pin Digital 22
Servo Front_Pan;                               //Center 2nd servo pin Digital 21

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

//H-bridge Stuff
void setup()
{
  AFMS.begin();  // create with the default frequency 1.6KHz

  Serial.begin(9600); // Start serial communication at 9600 bps
  Serial.println("Serial Started");

  // Open serial communications with the other Arduino board
  myserial.begin(9600);

  //attach correct servos to the correct pins
  Right_Tilt.attach(24);
  Left_Tilt.attach(23);
  Front_Tilt.attach(22);
  Front_Pan.attach(42);

  Serial.println("Servos Attached");

  //set servos starting positions
  Right_Tilt.write(55);
  delay(250);
  Left_Tilt.write(55);
  delay(250);
  Front_Tilt.write(25);
  delay(250);
  Front_Pan.write(90);
  delay(250);

  myStepper->setSpeed(1);

  pinMode(digitalPin12, OUTPUT);
  pinMode(digitalPin22, OUTPUT);
  pinMode(digitalPin43, OUTPUT);
  pinMode(digitalPin44, OUTPUT);
  pinMode(digitalPin45, OUTPUT);

  Serial.println("Pins Set");

  //Settle time
  delay(500);

  //setTunings(25,5, .75); //Best tune so far (100000,1,.55)
  //SetSampleTime(100); //Might be the fix as to why were were getting a NAN!
  Serial.println("PID Tuned");
  Serial.println("Entering Loop");
}
int once = 1;
void loop()
{
  /*cm_right = filter(analogPin2, 50);
    cm_left = filter(analogPin1, 50);
    Serial.print("Right:");
    Serial.println(cm_right);
    Serial.print("Left:");
    Serial.println(cm_left);
    Serial.println("------------");
  */
  //encoders();
  //Drive_Straight(100);
  //cm = IR_Distance(analogPin0);
  //Serial.println(cm);
  //testLineFollower();
  //FollowWall();
  //if (done == 0)  alignWall();
  //Quick Tsst for testing
  //FollowWall();
  //myStepper->step(100,FORWARD,DOUBLE);
  //delay(120);
  //drive_forward(100);
  //myStepper->release();
  //inBetweenWalls();
  //motorToggle(1);
  //Drive_Straight(1);
  //Drive_Straight(100);
  //Test_IR_Sensors();
  //motorToggle(0);
  //testServoAdjust();
  //Serial.print("Reading from right side ");
  //Serial.println(cm_right);
  //Serial.print("Reading from left side ");
  //Serial.println(cm_left);
  //Serial.print("Reading from center ");
  //Serial.println(cm);
  
  /*if (once == 1)
  {
    acquire_line();
    once = 0;
  }
  testLineFollower();
  */
  
  //motorToggle();
  //detectMag();
  //Odometry Tests
  //Drive_Circle(30);
  //Drive_Straight(100);
  //drive_reverse(30);            //roughly 8inchs
  //delay(2000);
  /*
     StateMachine TIME!!!
  */
  if (goCMD)
  {
    switch (CS)
    {
      case 1://Paddleboard state
        set_Ratio(4);
        inBetweenObjects();
        break;
      case 2://Wall-lift state
        set_Ratio(4);
        acquire_line();
        WallLift();
      case 3://U-turn state
        acquire_line();
        while(1)
        {
          testLineFollower();
        }
      case 4://Bars bb almost there
      case 5://pull ups yo...
      case 9://Reset state...
      default:
        //Ideal state???
        break;

    }
  }


  /*if(debugMyRobot.available())
    {
       UI(debugMyRobot.read());
    }*/
  /*
    if(auto_mode) auto_drive();
    if(line_follower) testLineFollower();
    if(f_wall)
    {
    if (done == 0)  alignWall();
    FollowWall();
    }
    /*else
    {
    if (done == 0)  alignWall();
     FollowWall();
    }*/
}
