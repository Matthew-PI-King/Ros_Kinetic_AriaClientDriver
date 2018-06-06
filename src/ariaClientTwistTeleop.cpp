#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "geometry_msgs/Twist.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopAria
{
public:
  TeleopAria();
  void keyLoop();

private:


  ros::NodeHandle nh_;
  double myTransRatio, myRotRatio, myLatRatio, myMaxVel;
  ros::Publisher myCommandDataPublisher;

};

TeleopAria::TeleopAria():
		myTransRatio(0.0),
		myRotRatio(0.0),
		myLatRatio(0.0),
		myMaxVel(50)
{
	myCommandDataPublisher = nh_.advertise<geometry_msgs::Twist>("pioneer2/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopAria teleopAria;

  signal(SIGINT,quit);

  teleopAria.keyLoop();

  return(0);
}


void TeleopAria::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the the Robot.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    myTransRatio=0.0, myRotRatio=0.0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        myRotRatio = 30;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        myRotRatio = -30;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        myTransRatio = 30;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        myTransRatio = -100;
        dirty = true;
        break;
      default:	
        myTransRatio = 0;
        myRotRatio = 0;
    }


    geometry_msgs::Twist msg;
    msg.linear.x=myTransRatio;
    msg.angular.z=myRotRatio;
    msg.linear.y=myLatRatio;

    
    myCommandDataPublisher.publish(msg);
    
    
  }


  return;
}

