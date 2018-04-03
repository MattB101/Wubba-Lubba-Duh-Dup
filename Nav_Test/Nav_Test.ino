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

SoftwareSerial debugMyRobot(10, 11);        //debugMyRobot(2,3); // RX, TX
const int analogPin0 = 0;                   //Center sensor
const int analogPin1 = 1;                   //Left sensor
const int analogPin2 = 2;                   //Right sesnor
const int analogPin3 = 3;                   //Center2 sensor
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
int done = 0;
boolean D_rection = true;
int count, inc, pause = 1;
int global_count = 0;
int index, lastIndex, wall, edge, obstacle, line = 0;
float distance = 0;
long duration;
float cm, cm_right, cm_left;

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

//PID Controls
//Set these wherever convenient
//Set using function setTunings()
double input, output, setPoint, kP, kI, kD , errSum, lastErr;
int sampleTime = 1;

//Set by algorithm
unsigned long last_time;

//Wheel o-shit variables
unsigned long t_ms = 0;
double t, t_old, Pos_left, Pos_right, Vel_left, Vel_right, Pos_left_old, Pos_right_old, rads; // check if shit breaks

//Sonar servo
Servo sonar;                                //Center servo pin Digital 24
Servo sonar1;                               //Right servo pin Digital 23
Servo sonar2;                               //Left servo pin Digital 22
Servo sonar3;                               //Center 2nd servo pin Digital 21

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
  debugMyRobot.begin(9600);

  //attach correct servos to the correct pins
  sonar.attach(24);
  sonar1.attach(23);
  sonar2.attach(22);
  sonar3.attach(42);

  Serial.println("Servos Attached");

  //set servos starting positions
  sonar.write(55);
  delay(250);
  sonar1.write(100);
  delay(250);
  sonar2.write(55);
  delay(250);
  sonar3.write(90);
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

  SetSampleTime(1); //Might be the fix as to why were were getting a NAN!
  delay(50);
  setTunings(100000, 1, .55); //Best tune so far (100000,1,.55)
  Serial.println("PID Tuned");
  Serial.println("Entering Loop");
}

void loop()
{
  //cm = IR_Distance(analogPin0);
  //Serial.println(cm);
  //testLineFollower();
  //FollowWall();
  //if (done == 0)  alignWall();
  //Quick Tsst for testing
  //FollowWall();
  
  /*for (int i = 1; i < 850; i++)
  {
    //myStepper->setSpeed(i);
    //myStepper->step(5, FORWARD, DOUBLE);
    myStepper->onestep(FORWARD, DOUBLE);
    //myStepper->setSpeed(100);
    //myStepper->step(25, FORWARD, DOUBLE);
    //myStepper->setSpeed(215);
    //myStepper->step(25, FORWARD, DOUBLE);
    //myStepper->release();
    //delay(250);
  }

  for (int i = 1; i < 1050; i++)
  {
    //myStepper->setSpeed(i);
    //myStepper->step(5, FORWARD, DOUBLE);
    myStepper->onestep(BACKWARD, DOUBLE);
    //myStepper->setSpeed(100);
    //myStepper->step(25, FORWARD, DOUBLE);
    //myStepper->setSpeed(215);
    //myStepper->step(25, FORWARD, DOUBLE);
    //myStepper->release();
    //delay(250);
  }
  myStepper->release();
  exit(0);*/
    digitalWrite(digitalPin44, HIGH);
    digitalWrite(digitalPin43, HIGH);
    digitalWrite(digitalPin45, LOW);  
  // myStepper->step(100,FORWARD,DOUBLE);
  // delay(120);
  // myStepper->release();
  //inBetweenWalls();
  //motorToggle(1);
  //Drive_Straight(1);
  //Drive_Straight(20);
  //motorToggle(0);
  //testServoAdjust();
  //Serial.print("Reading from right side ");
  //Serial.println(cm_right);
  //Serial.print("Reading from left side ");
  //Serial.println(cm_left);
  //Serial.print("Reading from center ");
  //Serial.println(cm);
  //testLineFollower();
  //motorToggle();
  //detectMag();
  //Odometry Tests
  //Drive_Circle(30);
  //Drive_Straight(100);
  //drive_reverse(30);            //roughly 8inchs
  //delay(2000);

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
