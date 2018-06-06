/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "Aria.h"
#include <cmath>
#include <math.h>


#define PI 3.14159265

/** @example simpleMotionCommands.cpp example showing how to connect and send
 * basic motion commands to the robot
 *
 * ARIA provides two levels of robot motion control, direct motion commands, and
 * actions. This example shows direct motion commands. See actionExample.cpp,
 * actionGroupExample.cpp, and others for examples on how to use actions.
 * Actions provide a more modular way of performing more complex motion
 * behaviors than the simple imperitive style used here.  
 *
 * See the ArRobot class documentation, as well as the overview of robot motion,
 * for more information.
 *
 * WARNING: this program does no sensing or avoiding of obstacles, the robot WILL
 * collide with any objects in the way!   Make sure the robot has about 2-3
 * meters of free space around it before starting the program.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */

int main(int argc, char **argv)
{

  Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArLog::log(ArLog::Terse, "WARNING: this program does no sensing or avoiding of obstacles, the robot WILL collide with any objects in the way! Make sure the robot has approximately 3 meters of free space on all sides.");

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  

  // We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  // Make sure you unlock before any sleep() call or any other code that will
  // take some time; if the robot remains locked during that time, then
  // ArRobot's background thread will be blocked and unable to communicate with
  // the robot, call tasks, etc.

  robot.lock();
  robot.enableMotors();
  robot.setRotVel(10.0);
  robot.unlock();
  ArUtil::sleep(1000);
 
  robot.lock();
  robot.enableMotors();
  robot.setRotVel(0);
  robot.unlock();
  ArUtil::sleep(1000);
     

  //Target pose
  double travel_dist=1000;
  double X_target=robot.getX()-travel_dist*cos(robot.getTh()*PI/180.0);
  double Y_target=robot.getY()-travel_dist*sin(robot.getTh()*PI/180.0);
  double Th_target=robot.getTh();
  double error_r,control_signal_v;
  double error_alpha,error_beta,control_signal_omega,beta;
  double k_r=1;
  double k_alpha=0.01;
  double k_beta=0.01;
  
  ArLog::log(ArLog::Normal, "Starting control loop...");
 
  
  error_r=sqrt(pow(X_target-robot.getX(),2.0)+pow(Y_target-robot.getY(),2.0));
  
  beta=atan2 (Y_target-robot.getY(),X_target-robot.getX())*180.0/PI;
  error_alpha=beta-robot.getTh();
  error_beta=beta-Th_target;
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Error=(%.5f %.5f %.5f)",error_r,std::abs(error_alpha),std::abs(error_beta));
  while(std::abs(error_r)>1.0 || std::abs(error_alpha)>1.0 || std::abs(error_beta)>1.0){
     
     	error_r=sqrt(pow(X_target-robot.getX(),2.0)+pow(Y_target-robot.getY(),2.0));
        if((X_target-robot.getX())<0){
		error_r=-error_r;}
  	beta=atan2 (Y_target-robot.getY(),X_target-robot.getX())*180.0/PI;
  	error_alpha=beta-robot.getTh();
  	error_beta=beta-Th_target;

	control_signal_omega=k_alpha*error_alpha+k_beta*error_beta;
	control_signal_v=k_r*error_r;

     ArLog::log(ArLog::Normal, "simpleMotionCommands: Error=(%.2f,%.2f,%.2f) controlsig=(%.2f,%.2f)",error_r,error_alpha,error_beta,control_signal_v,control_signal_omega);
     robot.setVel(control_signal_v);
     //robot.setRotVel(control_signal_omega);
     ArUtil::sleep(100);
  }	

  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

   
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
