void motorToggle()
{
  digitalWrite(digitalPin52, HIGH);
  delay(3000);
  digitalWrite(digitalPin52, LOW);
  delay(3000);
}

void right(int repeat , int denom)
{
  for (int i = 0; i < repeat; i = i + 1)
  {
    //fix wiring
    M3->run(BACKWARD);
    M4->run(BACKWARD);

    M3->setSpeed(test2);
    M4->setSpeed(test2);

    delay(test1 / denom);

    M3->setSpeed(0);
    M4->setSpeed(0);

    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
  }
}

void drive_right(int repeat)
{
  M3->run(BACKWARD);
  M4->run(BACKWARD);

  M4->setSpeed(test2);
  M3->setSpeed(test2);


  delay(test1 * repeat);

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

void left(int repeat , int denom)
{
  for (int i = 0; i < repeat; i = i + 1)
  {
    //fix wiring
    M3->run(FORWARD);
    M4->run(FORWARD);

    M3->setSpeed(test2);
    M4->setSpeed(test2);

    delay(test1 / denom);

    M3->setSpeed(0);
    M4->setSpeed(0);

    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
  }
}

void drive_left(int repeat)
{
  M3->run(FORWARD);
  M4->run(FORWARD);

  M4->setSpeed(test2);
  M3->setSpeed(test2);


  delay(test1 * repeat);

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

void reverse(int repeat, int denom)
{
  for (int i = 0; i < repeat; i = i + 1)
  {
    //fix wiring
    M3->run(FORWARD);
    M4->run(BACKWARD);

    M3->setSpeed(test2);
    M4->setSpeed(test2);

    delay(test1 / denom);

    M3->setSpeed(0);
    M4->setSpeed(0);

    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
  }
}

void drive_reverse(int repeat)
{
  M3->run(FORWARD);
  M4->run(BACKWARD);

  M4->setSpeed(test2);
  M3->setSpeed(test2);


  delay(test1 * repeat);

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

void forward(int repeat, int denom)
{
  for (int i = 0; i < repeat; i = i + 1)
  {
    //fix wiring
    M3->run(BACKWARD);
    M4->run(FORWARD);

    M4->setSpeed(test2);
    M3->setSpeed(test2);


    delay(test1 / denom);

    M4->setSpeed(0);
    M3->setSpeed(0);

    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
  }
}

void drive_forward(int repeat)
{
  M3->run(BACKWARD);
  M4->run(FORWARD);

  M4->setSpeed(test2);
  M3->setSpeed(test2);


  delay(test1 * repeat);

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

/*void Drive_Straight(double cm)
  {
    int mSpeedLeft = 100;
    int mSpeedRight = 100;
    M3->run(BACKWARD);
    M4->run(FORWARD);

    M4->setSpeed(mSpeedRight);                                   //Right
    M3->setSpeed(mSpeedLeft);                                  //Left

    while(Pos_left <= cm || Pos_right <= cm)
    {
      left_counts = leftEnc.read() * 2 * pi / GearRatio;         // rads turned;
      right_counts =  rightEnc.read() * 2 * pi / GearRatio;     // rads turned;
      //Serial.print(right_counts + '\t' + left_counts);
      Pos_left = left_counts * .762;
      Pos_right = right_counts * .762;
      Serial.print(Pos_left);
      Serial.print("\t");
      Serial.println(Pos_right);

      if(Pos_left > Pos_right)
      {
        mSpeedRight = mSpeedRight + 1;
        M4->setSpeed(mSpeedRight);                                   //Right
      }else if( Pos_right < Pos_left)
      {
        mSpeedRight = mSpeedRight - 4;
        M4->setSpeed(mSpeedRight);                                   //Right
      }

    }

    M4->setSpeed(0);
    M3->setSpeed(0);
    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
    exit(0);
  }
*/

void Drive_Circle(double radius)
{
  double w = 7.5 * 2.54;
  double inner_distance = 2 *pi * radius ;
  double outer_distance = 2 * pi * (radius + w);
  //Serial.println(inner_distance);
  //Serial.println(outer_distance);
  //Serial.println("-------------");
  int inner_speed = 100;
  int outer_speed = 0;
  double ratio = (radius + w) / radius;
  //Serial.print("");
  outer_speed = ratio * inner_speed;

  M3->run(BACKWARD);
  M4->run(FORWARD);

  M4->setSpeed(inner_speed);
  M3->setSpeed(outer_speed);

  while (Pos_left < (inner_distance))
  {
    //Odometry_Helper();
    left_counts = leftEnc.read() * 2 * pi / GearRatio;         // rads turned;
    right_counts =  rightEnc.read() * 2 * pi / GearRatio;     // rads turned;
    //Serial.print(right_counts + '\t' + left_counts);
    Pos_left = left_counts;
    Pos_right = right_counts;
    //Serial.print(Pos_left);
    //Serial.print("\t");
    Serial.println(Pos_left);
  }

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  M3->run(RELEASE);
  M4->run(RELEASE);
  exit(0);
}

void Odometry_Helper()
{
  //double t = micros()/1000000.0;  //current time

  left_counts = leftEnc.read(); // countsPerRev_motor;
  right_counts =  rightEnc.read(); // countsPerRev_motor;

  /*Serial.print(left_counts);
    Serial.print("---");
    Serial.print(right_counts);
    Serial.print("\r\n");
  */

  Pos_left = ((left_counts) / GearRatio) * 2.54 * 2 * pi;
  Pos_right = ((right_counts) / GearRatio) * 2.54 * 2 * pi;

  //Vel_left = (Pos_left - Pos_left_old)/(t - t_old);
  //Vel_right = (Pos_right - Pos_right_old)/(t - t_old);

  Pos_left_old = Pos_left;
  Pos_right_old = Pos_right;

  //t_old = t;

  //Serial.print(t);
  //Serial.print("\t");
  Serial.print(Pos_left);
  Serial.print("\t");
  Serial.print(Pos_right);
  Serial.print("\r\n");

}
