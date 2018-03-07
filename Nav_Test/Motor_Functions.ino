void motorToggle()
{
  digitalWrite(digitalPin52,HIGH);
  delay(3000);
  digitalWrite(digitalPin52,LOW);
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
    
    
    delay(test1*repeat);
    
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
    
    
    delay(test1*repeat);
    
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
    
    
    delay(test1*repeat);
    
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
    
    
    delay(test1*repeat);
    
    M4->setSpeed(0);
    M3->setSpeed(0);      

    delay(test3);
    M3->run(RELEASE);
    M4->run(RELEASE);
}
