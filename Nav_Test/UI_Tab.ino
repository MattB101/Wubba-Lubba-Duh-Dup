//UI for using uno to send commands to mega/robot
void UI(byte cmd_typed)
{
    switch(cmd_typed)
    {
      case 113:// q
          auto_mode = 1;
          line_follower = 0;
          f_wall = 0;
          break;
      case 47:// /
          M3->run(RELEASE);
          M4->run(RELEASE);
          myStepper->release();
          auto_mode = 0;
          line_follower= 0;
          f_wall = 0;
          break;
      case 108:// l
          myStepper->step(60, FORWARD, DOUBLE);
          myStepper->release();
          auto_mode = 0;
          line_follower = 0;
          f_wall = 0;
          break;
      case 107:// k
          myStepper->step(60, BACKWARD, DOUBLE);
          delay(1000);
          myStepper->release();
          auto_mode = 0;
          line_follower = 0;
          f_wall = 0;
          break;
      case 91://[
            digitalWrite(digitalPin12,HIGH);
            delay(100);
            auto_mode = 0;
            line_follower = 0;
            f_wall = 0;
            break;
      case 93://]
            digitalWrite(digitalPin12,LOW);
            delay(100);
            auto_mode = 0;
            line_follower = 0;
            f_wall = 0;
            break;
      case 106:// j
            line_follower  = 1;
            auto_mode = 0;
            f_wall = 0;
            break;
      case 104: // h
            f_wall = 1;
            line_follower = 0;
            auto_mode = 0;
            break;  
      default:
            //auto_mode = 0;
            //line_follower = 0;
            //f_wall = 0;
            break;      
    }
}

void auto_drive()
{
  sense_wall(0);
    //debugMyRobot.write(ir_distance);
     
    if (wall == 0)  
    {
        forward(1, 1);
    }
    else
    {     
        find_wall(); 
        auto_mode = 0;
        done = 0;
    }
    
    sense_edge(0);
     
    if (edge == 0)  
    {
        forward(1, 1);
    }
    else
    {
        find_edge();
    }
}
