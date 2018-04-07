void comp_left()
{
  t_ms = micros();
  t = t_ms / 1000000.0;                         //current time
  dt = t - t_old_left;

  Vel_left = Pos_left / t;

  error = .00000001 - Vel_left;

  dErrordt = (error - error_old_left) / dt;
  integError_left = integError_left + error * dt;

  V = kp_left * error + kd_left * dErrordt + ki_left * integError_left;

  if (!isnan(V))
  {
    V = (V * 2.5);
    V = constrain(V, 0, 200);
    M4->setSpeed(V);
  }

  t_old_left = t;
  Pos_old_left = Pos_left;
  error_old_left = error_left;
}

void comp_right()
{
  t_ms = micros();
  t = t_ms / 1000000.0;                         //current time
  dt = t - t_old_right;

  Vel_right = Pos_right / t;

  error = .00000001 - Vel_right;

  dErrordt = (error - error_old_right) / dt;
  integError_right = integError_right + error * dt;

  V = kp_right * error + kd_right * dErrordt + ki_right * integError_right;

  if (!isnan(V))
  {
    V = (V * 2.5);
    V = constrain(V, 0, 200);
    M3->setSpeed(V);
  }

  t_old_right = t;
  Pos_old_right = Pos_right;
  error_old_right = error_right;
}
void sense_wall(int mode)
{
  cm = filter(analogPin0, 50);
  if (cm < 5)
  {
    wall = 1;
  }
  else
    wall = 0;

  if (mode == 0 || wall == 0)
  {
    Right_Tilt.write(100);
  }
  else if (mode == 1 || wall == 1)
  {
    Right_Tilt.write(50);
  }
}

void FollowWall()
{
  drive_forward(25);
  cm = filter(analogPin2, 50);
  global_memory[stack_ptr] = cm;
  slope = 0;

  if (stack_ptr > 1)
  {
    for (int h = 1; h < stack_ptr - 1; h++)
    {
      slope = slope + ((global_memory[stack_ptr - h] - global_memory[stack_ptr - h - 1]) / 2);
    }
    slope = slope / 3;
  }

  stack_ptr++;
  if (stack_ptr > 3) stack_ptr = 0;

  if (cm > (distance * 1.3) && slope < .5)
  {
    drive_right(5);
    drive_reverse(15);
    drive_left(5);
    global_count++;
  }
  else if (cm < distance && slope > -.5)
  {
    drive_left(5);
    drive_reverse(15);
    drive_right(5);
    global_count++;
  }
  else if (slope > .5 || slope < -.5)
  {
    drive_reverse(10);
    alignWall();
    stack_ptr = 0;
    global_count++;
  }

  if (global_count > 1)
  {
    alignWall();
    global_count = 0;
  }
}

void calibrate()
{
  distance = filter(analogPin2, 50);
}

void alignWall()
{
  float scan[180];
  float average = 0;
  int filter_win = 7;
  int counter = 0;
  int check_count = 0;
  String look_direction = "right";

  while (look_direction == "right")
  {
    left(1, 2);
    if (counter > 179) counter = 0;
    scan[counter] = filter(analogPin2, 50);
    //Serial.println(scan[counter]);

    if (counter > (filter_win - 1))
    {
      for (int h = 0; h < filter_win; h++)
      {
        average = average + scan[counter - h];
      }

      average = average / filter_win;

      if (scan[counter] > average || average >= 14)
        look_direction = "left";
    }
    counter++;
  }
  counter = 0;

  if (average < 16)
  {
    while (look_direction == "left")
    {
      right(1, 2);
      if (counter > 179) count = 0;
      scan[counter] = filter(analogPin2, 50);

      if (counter > (filter_win - 1))
      {
        for (int h = 1; h < filter_win; h++)
        {
          average = average + scan[counter - h];
        }

        average = average / filter_win;

        if (scan[counter] > (average + .5) || average >= 14) //.5 IS MAGIC NUMBERish.
        {

          check_count++;
          int low = scan[counter];
          int replace_low = 0;

          for (int i = filter_win; i > 0; i--)
          {
            if (scan[counter - i] < low)
            {
              low = scan[counter - i];
              replace_low = i;
            }
          }

          if (check_count > 5) //5 is ALSO MAGIC NUMBERish.
          {
            drive_left(replace_low);
            done = 1;
            break;
          }
        }
      }
      counter++;
    }
  }
  else
  {
    auto_mode = 1;
    stack_ptr = 0;
  }
}

void Test_IR_Sensors()
{
  float left = filter(analogPin1, 50);
  float right = filter(analogPin2, 50);
  float center_short = filter(analogPin0, 50);
  float center_long = filter_long(analogPin8, 50);
  //Serial.print("Left Sens: ");
  //Serial.println(left);
  //Serial.print("Right Sens: ");
  //Serial.println(right);
  Serial.print("Center_Short: ");
  Serial.println(center_short);
  //Serial.print("Center_Long: ");
  //Serial.println(center_long);
  //myserial.write(center_long);
}

void WallLift()
{
  float dist_to_wall = 0;
  //Front_Tilt.write(95);
  drive_forward(60);
  motorToggle(1);
  drive_forward(50);
  motorToggle(0);
  CS = 3;
}

void inBetweenObjects()
{
  float thresh = 2.5; //We can make this small if needed...
  if (flag == 0)
  {
    for (int i = 1; i < 350; i++)
    {
      myStepper->onestep(FORWARD, DOUBLE);
      flag = 1;
    }
    myStepper->release();
  }

  cm_right = filter(analogPin2, 50);
  cm_left = filter(analogPin1, 50);
  /*Serial.print("Right:");
    Serial.println(cm_right);
    Serial.print("Left:");
    Serial.println(cm_left);
    Serial.println("------------");
  */

  if ((cm_right > cm_left) && abs(cm_right - cm_left) >= thresh)
  {
    left(3, 1);
    reverse(3, 1);
    right(4, 1);
  }
  else if ((cm_left > cm_right) && abs(cm_left - cm_right) >= thresh)
  {
    right(3, 1);
    reverse(3, 1);
    left(4, 1);
  }
  else if (cm_right > 15 && cm_left > 15)
  {
    if (flag1 == 0)
    {
      for (int i = 1; i < 350; i++)
      {
        myStepper->onestep(BACKWARD, DOUBLE);
      }
      myStepper->release();
      flag1 = 1;
    }

    set_Ratio(3);
    drive_forward(10);
    //if we detect the wall in front set this state!
    //if ()
    //{
    CS = 2;
    //}
  }
  else
  {
    Serial.println("Driving Forward");
    drive_forward(20);
  }
}

void find_wall()
{
  while (wall == 1)
  {
    D_rection = !(D_rection);

    if (D_rection == false)
      right(count, 1);
    else
      left(count, 1);

    sense_wall(1);
    count = count + 1;
  }

  sense_wall(0);

  drive_reverse(5);

  if (D_rection == false) right(5 + count, 1);
  else left(5 + count, 1);

  count = 0;
}


void sense_edge(int mode)
{
  cm = filter(analogPin0, 50);
  //Serial.print("edge@");
  //Serial.println(cm);

  if (cm >= 10)
  {
    edge = 1;
    Serial.print("edge@");
    Serial.println(cm);
  }
  else
    edge = 0;

  if (mode == 0 || edge == 0) Right_Tilt.write(50);
  else if (mode == 1 || edge == 1) Right_Tilt.write(100);
  //delay(test4);
}

void find_edge()
{
  while (edge == 1)
  {
    D_rection = !(D_rection);
    //debugMyRobot.write(D_rection);
    if (D_rection == false)
      right(count, 1);
    else
      left(count, 1);

    sense_edge(1);
    count = count + 1;
  }

  sense_edge(0);

  drive_reverse(5);

  if (D_rection == false) right(5 + count, 1);
  else left(5 + count, 1);

  count = 0;
}

float IR_Distance(int sensorNum)
{
  float dist = 0;
  float voltages = analogRead(sensorNum) * .004828125;
  dist = (5 * pow(voltages, -1));
  delay(1);
  return dist;
}

float IR_Distance_Long(int sensorNum)
{
  float dist = 0;
  float voltages = analogRead(sensorNum) * .004828125;
  dist = (13 * pow(voltages, -1));
  delay(1);
  return dist;
}

float filter_long(int sensorNum, int window)
{
  float dist = 0;

  for (int i = 0; i < window; i++)
  {
    dist = dist + IR_Distance_Long(sensorNum);
  }
  dist = dist / window;
  return dist;
}

float filter(int sensorNum, int window)
{
  float dist = 0;

  for (int i = 0; i < window; i++)
  {
    dist = dist + IR_Distance(sensorNum);
  }
  dist = dist / window;
  return dist;
}

void detectMag()
{
  float valRead = analogRead(analogPin7) * .004828125;
  Serial.println(valRead);
  if (valRead <= 2.00) {
    digitalWrite(digitalPin22, HIGH);
    Serial.println("Got here!! True");
  }
  else {
    digitalWrite(digitalPin22, LOW);
    Serial.println("Got here!! False");
  }
}

void testServoAdjust()
{
  Right_Tilt.write(50);
  Left_Tilt.write(100);
  Front_Tilt.write(45);
  //delay(500);
  //Right_Tilt.write(100);
  //Left_Tilt.write(45);
  //Front_Tilt.write(100);
  delay(500);

}

void testLineFollower()
{
  //int window[5] = NULL;

  Right_Tilt.write(100);
  find_line();
  forward(1, 1);
  /*
    cm = 0;
    for (int i = 0; i < 50; i++) cm = cm + filter();
    cm = cm / 50;

    if (cm > 4 && cm < 10)
    forward(1);
    else
  */
}

void acquire_line()
{
  int line_index = 0;
  left(15, 2);
  while (line == 0 && line_index < 30)
  {
    right(1, 2);
    sense_line();
    line_index++;
  }
}

//OPTO: tips make the robot move more smoothly left and right KON3!
void find_line()
{
  while (line == 0)
  {
    D_rection = !(D_rection);
    if (D_rection == false)
      for (int i = 0; i < count; i++)
      {
        right(1, 2);
        sense_line();
        if (line == 1) break;
      }
    else
      for (int i = 0; i < count; i++)
      {
        left(1, 2);
        sense_line();
        if (line == 1) break;
      }
    sense_line();
    count = count + 1;
  }
  //sense_line();
  count = 0;
}

void sense_line()
{
  cm = filter(analogPin0, 50);

  if ((cm > 6) && (cm < 15)) //WAS 4
  {
    line = 1;
    //Serial.print("line@");
    //Serial.println(cm);
  }
  else {
    line = 0;
    //Serial.print("no@");
    //Serial.println(cm);
  }
}
/*void comp()
  {
  unsigned long current = millis();
  double timeDelta = (double)(current - last_time);

  if (timeDelta >= sampleTime)
  {
    double error = setPoint - input;
    errSum += error;
    double dErr = (error - lastErr);

    output = kP * error  + kD * dErr + kI * errSum;

    lastErr = error;
    last_time = current;
  }
  }
  void setTunings(double kp, double ki, double kd)
  {
  kP = kp;
  kI = ki;
  kD = kd;
  }
*/
