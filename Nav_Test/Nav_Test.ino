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
//#include <SoftwareSerial.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Encoder.h"

//SoftwareSerial debugMyRobot(10,11);         //debugMyRobot(2,3); // RX, TX
const int analogPin0 = 0;                   //Center sensor
const int analogPin1 = 1;                   //Left sensor 
const int analogPin2 = 2;                   //Right sesnor
const int analogPin7 = 7;                   //mag sensor

const int digitalPin22 = 22;                // LED out
const int digitalPin12 = 12;                //motorsheild input
const int digitalPin52 = 52;                //digital toggle for wall lift

//const float pi = 3.14;
const int height = 5;
int course_stage = 0;
int auto_mode = 0;                         //flag for auto_mode
int line_follower = 0;                     //flag for line_follower
int f_wall = 0;                            //flag for follow_wall
char val;                                  // Data received from the serial port
int radius = 16;
int angle = 45;
int done = 0;
//int data[5]; //currently unused
boolean D_rection = true;
int count, inc, pause = 1;
int global_count = 0;
int index, lastIndex, wall, edge, obstacle, line = 0;
float distance = 0;
long duration;
float cm, cm_right,cm_left;
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

Encoder leftEnc(18, 19);             //Motor1
Encoder rightEnc(2, 3);            //Motor2

double GearRatio = 50*64;
double countsPerRev_motor = 64;
double left_counts;
double right_counts;
double left_last_count;
double right_last_count;

//time variables
unsigned long t_ms = 0;
double t, t_old, Pos_left, Pos_right, Vel_left, Vel_right, Pos_left_old, Pos_right_old= 0;
double rads = 0;
double pi = 3.141592654;

                        
//Sonar servo 
Servo sonar;                                //Center servo pin Digital 24
Servo sonar1;                               //Right servo pin Digital 23
Servo sonar2;                               //Left servo pin Digital 22

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1); 
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

void setup() 
{
  AFMS.begin();  // create with the default frequency 1.6KHz
    
  Serial.begin(9600); // Start serial communication at 9600 bps

  // Open serial communications with the other Arduino board
  //debugMyRobot.begin(9600);

  //attach correct servos to the correct pins
  sonar.attach(24);
  sonar1.attach(23);
  sonar2.attach(22);
  
  //set servos starting positions
  sonar.write(55);
  delay(250);
  sonar1.write(100);
  delay(250);
  sonar2.write(55);
  //delay(250);
  
  myStepper->setSpeed(215);
  pinMode(digitalPin12,OUTPUT);
  pinMode(digitalPin22,OUTPUT);
  pinMode(digitalPin52,OUTPUT);

  delay(500);
  //calibrate();
  /*Serial.println("Calibration>");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println();
  Serial.print("Max Distance: ");
  Serial.print(distance * 1.5);
  Serial.println();
  Serial.println("Calibration Done");*/
}

void loop() 
{
  //cm = IR_Distance(analogPin0);
  //Serial.println(cm);
  //testLineFollower();
  //FollowWall();
  //if (done == 0)  alignWall();
  //FollowWall();  
  //inBetweenWalls();
  //testServoAdjust();
  
  //Serial.print("Reading from right side ");
  //Serial.println(cm_right);
  //Serial.print("Reading from left side ");
  //Serial.println(cm_left);
  //Serial.print("Reading from center ");
  //Serial.println(cm);
  //testLineFollower();
  //motorTogle();
  //detectMag();

  Drive_Circle(30);
  //Drive_Straight(10);
  /*if(debugMyRobot.available())
  {  
       UI(debugMyRobot.read());
  }*/
  /*
  if(auto_mode)
  {
     auto_drive(); 
  }
  if(line_follower)
  {
    testLineFollower();
  }
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
