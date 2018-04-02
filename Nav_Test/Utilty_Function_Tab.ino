void sense_wall(int mode)
{
  cm = filter(analogPin0, 50);
  if (cm < 5)
  {
    wall = 1;
    //Serial.print("wall@");
    //Serial.println(cm);
  }
  else
    wall = 0;

  if (mode == 0 || wall == 0)
  {
    sonar.write(100);
    //sonar1.write(45);
    //sonar2.write(100);
  }
  else if (mode == 1 || wall == 1)
  {
    sonar.write(50);
    //sonar1.write(100);
    //sonar2.write(45);
  }
  //delay(test4);
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

void inBetweenWalls()
{
  int thresh = 2; //We can make this small if needed...

  cm_right = filter(analogPin2, 50);
  cm_left = filter(analogPin1, 50);

  if ((cm_right > cm_left) && abs(cm_right - cm_left) >= thresh)
  {
    drive_right(5);
    drive_reverse(10);
    drive_left(5);
  }
  else if ((cm_left > cm_right) && abs(cm_left - cm_right) >= thresh)
  {
    drive_left(5);
    drive_reverse(10);
    drive_right(5);
  }
  else
  {
    drive_forward(10);
  }
  if (cm_right > 100 && cm_left > 100)
  {
    //Do something with the front servo to detect front wall or lines.......
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
  //sonar.write(115);
  /*delay(test4);

    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);

    cm = microsecondsToCentimeters(duration);
  */
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

  if (mode == 0 || edge == 0) sonar.write(50);
  else if (mode == 1 || edge == 1) sonar.write(100);
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

/*filter(int sensorNum, int window)
  {
    int dist = 0;

    for(int i = 0; i < window; i++)
    {
        dist = dist + IR_Distance(sensorNum);
    }
    dist = dist / window;
    return dist;
  }*/

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
  sonar.write(50);
  sonar1.write(100);
  sonar2.write(45);
  //delay(500);
  //sonar.write(100);
  //sonar1.write(45);
  //sonar2.write(100);
  delay(500);

}

void testLineFollower()
{
  //int window[5] = NULL;

  sonar.write(100);
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

  sense_line();
  count = 0;
}

void sense_line()
{
  cm = filter(analogPin0, 50);

  if ((cm > 6) && (cm < 10)) //WAS 4
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

void comp()
{
  unsigned long current = micros();
  int timeDelta = (current - last_time);

  if (timeDelta >= sampleTime)
  {
    double error = setPoint - input;
    errSum += error;
    double dErr = (error - lastErr);

    output = kP * error + kI * errSum + kD * dErr;

    lastErr = error;
    last_time = current;
  }
}

void setTunings(double kp, double ki, double kd)
{
  double SampleTimeInSecs = ((double)sampleTime) / 1000;
  kP = kp;
  kI = ki * SampleTimeInSecs;
  kD = kd / SampleTimeInSecs;
}

void SetSampleTime(int newSampleTime)
{
  if (newSampleTime > 0)
  {
    double ratio = (double) newSampleTime / (double) sampleTime;
    kI *= ratio;
    kD /= ratio;
    sampleTime = (unsigned long) newSampleTime;
  }
}
