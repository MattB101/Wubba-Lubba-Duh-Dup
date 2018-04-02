void motorToggle()
{
  digitalWrite(digitalPin43,HIGH);
  delay(100);
  digitalWrite(digitalPin44,LOW);
  digitalWrite(digitalPin45,HIGH);
  delay(100);
  digitalWrite(digitalPin44,LOW);
  digitalWrite(digitalPin45,LOW);
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

void Drive_Straight(double cm)
  {
    int mSpeedLeft = 100;
    int mSpeedRight = 100;

    setPoint = 1;
    
    M3->run(BACKWARD);
    M4->run(FORWARD);

    M4->setSpeed(mSpeedRight);                                   //Right
    M3->setSpeed(mSpeedLeft);                                    //Left
    delay(250);
    bool motor = true;
    
    while(Pos_left < cm && Pos_right < cm)
    {
      encoders();
      if (motor == true)
      {
        input = Pos_left / Pos_right;

        //Summing junc for PID
        comp();
        
        mSpeedRight = mSpeedLeft + (mSpeedLeft * output); //!!!!!!!!!!!!!!!!!!! WAS WORKING
        M3->setSpeed(mSpeedRight);
      }
      else if (motor == false)
      {
        input = Pos_right / Pos_left;
        //Summing junc for PID
        comp();
        mSpeedLeft = mSpeedRight + (mSpeedRight * output);
        M4->setSpeed(mSpeedLeft);
      }
      motor = !motor;
    }
    M4->setSpeed(0);
    M3->setSpeed(0);
    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
    exit(0);
  }

void Drive_Circle(double radius)
{
  radius = radius + (2.3 *.5);
  double w = 7.5 * 2.54;
  double inner_distance = 2 *pi * radius ;
  double outer_distance = 2 * pi * (radius + w);
  int inner_speed = 100;
  int outer_speed = 0;
  double ratio = (radius + w) / radius;
  outer_speed = ratio * inner_speed;

  M3->run(BACKWARD);
  M4->run(FORWARD);

  M4->setSpeed(inner_speed);
  M3->setSpeed(outer_speed);

  while (Pos_left < (inner_distance) && Pos_right < (outer_distance)) encoders();

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  
  M3->run(RELEASE);
  M4->run(RELEASE);
}

void encoders()
{
  Pos_left = (rightEnc.read() / encoder_scale) * wheel_circumference; //right and left switched AGAIN
  Pos_right = (leftEnc.read() / encoder_scale) * wheel_circumference;

  //Vel_left = (Pos_left - Pos_left_old)/(t - t_old);
  //Vel_right = (Pos_right - Pos_right_old)/(t - t_old);
  //Pos_left_old = Pos_left;
  //Pos_right_old = Pos_right;
  //t_old = t;
}
