void motorToggle(int turnOn)
{
  if (turnOn==1)
  {
    digitalWrite(digitalPin44, HIGH);
    digitalWrite(digitalPin43, LOW);
    digitalWrite(digitalPin45, HIGH);
  } else if (turnOn == 0)
  {
    digitalWrite(digitalPin44, LOW);
  }
}

void right(int repeat , int denom)
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

void drive_right(int repeat)
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

void left(int repeat , int denom)
{
  for (int i = 0; i < repeat; i = i + 1)
  {
    //fix wiring
    M3->run(BACKWARD);
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

void reverse(int repeat, int denom)
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

void drive_reverse(int repeat)
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

void forward(int repeat, int denom)
{
  for (int i = 0; i < repeat; i = i + 1)
  {
    //fix wiring
    M3->run(FORWARD);
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
void set_Ratio(int ratioNum)
{
  switch (ratioNum)
  {
    case 1:
      left_speed = 250;
      right_speed = 245;
      break;
    case 2:
      left_speed = 200;
      right_speed = 196;
      break;
    case 3:
      left_speed = 150;
      right_speed = 147;
      break;
    case 4:
      left_speed = 100;
      right_speed = 98;
      break;
    default:
      break;
  }
}

void drive_forward(int repeat)
{

  M3->run(FORWARD);
  M4->run(FORWARD);


  M4->setSpeed(right_speed);
  M3->setSpeed(left_speed);


  delay(test1 * repeat);

  M4->setSpeed(0);
  M3->setSpeed(0);

  delay(test3);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

/*
   broken as fuck!!!
*/
void Drive_Straight(double cm)
{
  rightEnc.write(0);
  leftEnc.write(0);

  Pos_left = 0;
  Pos_right = 0;

  float mSpeedLeft = 100 + (cm * .15);
  float mSpeedRight = 100;

  M3->run(FORWARD);
  M4->run(FORWARD);

  M3->setSpeed(mSpeedLeft);
  M4->setSpeed(mSpeedRight);

  delay(200);

  while (abs(Pos_left) < cm && abs(Pos_right) < cm) //was &&, now ||
  {
    Pos_left = (leftEnc.read() / encoder_scale) * wheel_circumference;
    comp_left();
    Pos_right = (rightEnc.read() / encoder_scale) * wheel_circumference;
    comp_right();
  }
  M4->setSpeed(0);
  M3->setSpeed(0);
  delay(test3);
  M4->run(RELEASE);
  M3->run(RELEASE);
  Serial.println("Finished Driving!! Was I straight?!?!?");
}

void encoders()
{
  Pos_left = (leftEnc.read() / encoder_scale) * wheel_circumference;
  Pos_right = (rightEnc.read() / encoder_scale) * wheel_circumference;
}
