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
#include "ArNetworking.h"
#include <cmath>
#include <math.h>


#define PI 3.14159265

/** @example serverDemo.cpp Example ArNetworking server providing teleoperation,
 * sonar data, control the camera, etc.
 *
 * This is a basic ArNetworking server. It connects to a robot or simulator,
 * including, if available, IRs, gyro, and bumpers.  Give the option 
 * "-connectLaser" on the command line to enable the laser rangefinder,
 * if available.
 *
 * Run "./serverDemo -help" for a full list of command line options.
 *
 * Once running, connect to this server with a a client such as 
 * MobileEyes.
 *
 * This server provides the following services:
 *  - User login (optional)
 *  - Basic robot telemetry information 
 *  - Range sensor data values (not used by MobileEyes)
 *  - Graphics representing range sensor reading positions
 *  - Teleoperation modes (including safe/unsafe drive modes)
 *  - Wander mode
 *  - Various advanced "custom" commands to control logging, debugging, etc.
 *  - If an ACTS or SAV server is running, forward the video stream 
 *  - Camera control (pan/tilt/zoom) if cameras are available
 *
 * Note that this program requires a terminal to run -- i.e. you can't run
 * it in the background in Linux.  To modify it to allow that, remove the key
 * handler code in main().
 */
ArRobot robot;
//NEW LASER DATA REQUESt CAPABILITY
void laserRequest(ArServerClient *client, ArNetPacket *packet)
{ 
  robot.lock();
  ArNetPacket sending;
  sending.empty();
  ArLaser* laser = robot.findLaser(1);
  if(!laser){
      printf("Could not connect to Laser... exiting\n");
      Aria::exit(1);}	
  laser->lockDevice();
  const std::list<ArSensorReading*> *sensorReadings = laser->getRawReadings(); // see ArRangeDevice interface doc
  sending.byte4ToBuf((ArTypes::Byte4)(sensorReadings->size()));
  for (std::list<ArSensorReading*>::const_iterator it2= sensorReadings->begin(); it2 != sensorReadings->end(); ++it2){
	ArSensorReading* laserRead =*it2;
        sending.byte4ToBuf((ArTypes::Byte4)(laserRead->getRange()));
	//printf("%i,%i:",laserRead->getRange(),laserRead->getIgnoreThisReading());
  }
  laser->unlockDevice();
  robot.unlock();
  sending.finalizePacket();
  //sending.printHex();
  client->sendPacketTcp(&sending);
}

//NEW VELOCITY SETTING CAPABILITY  //TODO stop when client disconnects
void setVelRequest(ArServerClient *client, ArNetPacket *packet)
{ 
  robot.lock();
  double vel;
  double rotVel;
  vel=(double)packet->bufToByte2();
  rotVel=(double)packet->bufToByte2();
  printf("recieved data %lf,%lf\n", vel,rotVel);
  robot.setVel(vel);
  robot.setRotVel(rotVel);
  robot.unlock();
}


//NEW MOVE BOOK 
void moveStepRequest(ArServerClient *client, ArNetPacket *packet)
{	robot.lock();
  	robot.enableMotors();
  	robot.setRotVel(0);
	robot.setVel(0);
  	robot.unlock();
  	ArUtil::sleep(1000);
     

	  //Target pose
	  double travel_dist=(double)packet->bufToByte2();  // the distance to travel back in mm
	  double X_target=robot.getX()-travel_dist*cos(robot.getTh()*PI/180.0);
	  double Y_target=robot.getY()-travel_dist*sin(robot.getTh()*PI/180.0);
	  double Th_target=robot.getTh();
	  double error_r,control_signal_v;
	  double error_alpha,error_beta,control_signal_omega,beta;
	  double k_r=1;
	  double k_alpha=0.01;
	  double k_beta=0.01;
	  double counter=0;
  
  	  ArLog::log(ArLog::Normal, "Starting control loop...");
   	  error_r=sqrt(pow(X_target-robot.getX(),2.0)+pow(Y_target-robot.getY(),2.0));
          beta=atan2 (Y_target-robot.getY(),X_target-robot.getX())*180.0/PI;
  	  error_alpha=beta-robot.getTh();
  	  error_beta=beta-Th_target;
  	  ArLog::log(ArLog::Normal, "simpleMotionCommands: Error=(%.5f %.5f %.5f)",error_r,std::abs(error_alpha),std::abs(error_beta));
  	  while(std::abs(error_r)>1.0){
          	error_r=sqrt(pow(X_target-robot.getX(),2.0)+pow(Y_target-robot.getY(),2.0));
        	if((X_target-robot.getX())<2.0){
			error_r=-error_r;}
  		beta=atan2 (Y_target-robot.getY(),X_target-robot.getX())*180.0/PI;
  		error_alpha=beta-robot.getTh();
  		error_beta=beta-Th_target;
		control_signal_omega=k_alpha*error_alpha+k_beta*error_beta;
		control_signal_v=k_r*error_r;
     		//ArLog::log(ArLog::Normal, "simpleMotionCommands: Error=(%.2f,%.2f,%.2f) controlsig=(%.2f,%.2f)",error_r,error_alpha,error_beta,control_signal_v,control_signal_omega);
		robot.lock();     		
		robot.setVel(control_signal_v);
     		//robot.setRotVel(control_signal_omega);
		robot.unlock();
     		ArUtil::sleep(100);
		counter++;
		if(counter>150) {
		ArLog::log(ArLog::Normal,"control loop exit code 1");		
		break;
		}
  	}
	ArLog::log(ArLog::Normal, "Move step Positioning Error=(%.2f)",error_r);
	robot.lock();	
	robot.setRotVel(0);
	robot.setVel(0);
  	robot.unlock();
}


void laserRequest_and_odom(ArServerClient *client, ArNetPacket *packet)
{ 
  robot.lock();
  ArNetPacket sending;
  sending.empty();
  ArLaser* laser = robot.findLaser(1);
  if(!laser){
      printf("Could not connect to Laser... exiting\n");
      Aria::exit(1);}	
  laser->lockDevice();
  const std::list<ArSensorReading*> *sensorReadings = laser->getRawReadings(); // see ArRangeDevice interface doc
  sending.byte4ToBuf((ArTypes::Byte4)(sensorReadings->size()));
  for (std::list<ArSensorReading*>::const_iterator it2= sensorReadings->begin(); it2 != sensorReadings->end(); ++it2){
	ArSensorReading* laserRead =*it2;
        sending.byte4ToBuf((ArTypes::Byte4)(laserRead->getRange()));
	//printf("%i,%i:",laserRead->getRange(),laserRead->getIgnoreThisReading());
  }
  sending.byte4ToBuf((ArTypes::Byte4)(robot.getX()));
  sending.byte4ToBuf((ArTypes::Byte4)(robot.getY()));
  sending.byte4ToBuf((ArTypes::Byte4)(robot.getTh()));
  sending.byte4ToBuf((ArTypes::Byte4)(robot.getVel()));
  sending.byte4ToBuf((ArTypes::Byte4)(robot.getRotVel()));
  //printf("%1f,%1f,%1f\n",robot.getX(),robot.getY(),robot.getTh());
  laser->unlockDevice();
  robot.unlock();
  sending.finalizePacket();
  //sending.printHex();
  client->sendPacketTcp(&sending);
}

int main(int argc, char **argv)
{
  // mandatory init
  Aria::init();

  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // set up our parser
  ArArgumentParser parser(&argc, argv);

  // load the default arguments 
  parser.loadDefaultArguments();

  // robot
  //ArRobot robot;
  ArGlobalFunctor2<ArServerClient *, ArNetPacket *> laserRequestCB(&laserRequest); //this is used for laserdata requests
  ArGlobalFunctor2<ArServerClient *, ArNetPacket *> setVelRequestCB(&setVelRequest); 
  ArGlobalFunctor2<ArServerClient *, ArNetPacket *> moveStepRequestCB(&moveStepRequest); 
  ArGlobalFunctor2<ArServerClient *, ArNetPacket *> laserRequestCB2(&laserRequest_and_odom); //this is used for laserdata requests
  // set up our simple connector
  ArRobotConnector robotConnector(&parser, &robot);


  // add a gyro, it'll see if it should attach to the robot or not
  ArAnalogGyro gyro(&robot);


  // set up the robot for connecting
  if (!robotConnector.connectRobot())
  {
    printf("Could not connect to robot... exiting\n");
    Aria::exit(1);
  }

  // our base server object
  ArServerBase server;
  server.addData("LaserRequest", "custom command for server to enable laser data", &laserRequestCB, "none", "none"); //laser data request
  server.addData("SetVelRequest", "custom command for server to enable simple motion commands", &setVelRequestCB, "Byte2(velocity in mm/s)Byte2(rot velocity in deg/s)", "none"); //laser data request
  server.addData("MoveStepRequest", "custom command for server to enable simple motion commands", &moveStepRequestCB, "Byte2(move back distance in mm)", "none"); 
server.addData("LaserRequest_odom", "custom command for server to enable laser data", &laserRequestCB2, "none", "none"); //laser data request
  ArServerSimpleOpener simpleOpener(&parser);

  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
 
  // Tell the laser connector to always connect the first laser since
  // this program always requires a laser.
  parser.addDefaultArgument("-connectLaser");
  
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  ArClientSwitchManager clientSwitchManager(&server, &parser);

  // parse the command line... fail and print the help if the parsing fails
  // or if the help was requested
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
    Aria::logOptions();
    Aria::exit(1);
  }

  // Set up where we'll look for files such as user/password 
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "ArNetworking/examples");

  // first open the server up
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    if (simpleOpener.wasUserFileBad())
      printf("Bad user/password/permissions file\n");
    else
      printf("Could not open server port\n");
    exit(1);
  }

  // Range devices:
  
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);

  ArIRs irs;
  robot.addRangeDevice(&irs);

  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);

  // attach services to the server
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoDrawings drawings(&server);

  // modes for controlling robot movement
  ArServerModeStop modeStop(&server, &robot);
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  
  ArServerModeWander modeWander(&server, &robot);
  modeStop.addAsDefaultMode();
  modeStop.activate();

  // set up the simple commands
  ArServerHandlerCommands commands(&server);
  ArServerSimpleComUC uCCommands(&commands, &robot);  // send commands directly to microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // control debug logging
  ArServerSimpleComGyro gyroCommands(&commands, &robot, &gyro); // configure gyro
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot); // control more debug logging
  ArServerSimpleServerCommands serverCommands(&commands, &server); // control ArNetworking debug logging
  ArServerSimpleLogRobotDebugPackets logRobotDebugPackets(&commands, &robot, ".");  // debugging tool

  // This is an older drive mode. ArServerModeDrive is newer and generally performs better,
  // but you can use this for old clients if neccesary.
  //ArServerModeDrive modeDrive(&server, &robot);
  //modeDrive.addControlCommands(&commands); // configure the drive modes (e.g. enable/disable safe drive)

  ArServerHandlerConfig serverHandlerConfig(&server, Aria::getConfig()); // make a config handler
  ArLog::addToConfig(Aria::getConfig()); // let people configure logging

  // Forward video if either ACTS or SAV server are running.
  // You can find out more about SAV and ACTS on our website
  // http://robots.activmedia.com. ACTS is for color tracking and is
  // a separate product. SAV just does software A/V transmitting and is
  // free to all our customers. Just run ACTS or SAV server before you
  // start this program and this class here will forward video from the
  // server to the client.
  ArHybridForwarderVideo videoForwarder(&server, "localhost", 7070);
  
  // Control a pan/tilt/zoom camera, if one is installed, and the video
  // forwarder was enabled above.
  ArPTZ *camera = NULL;
  ArServerHandlerCamera *handlerCamera = NULL;
  ArCameraCollection *cameraCollection = NULL;
  if (videoForwarder.isForwardingVideo())
  {
    bool invertedCamera = false;
    camera = new ArVCC4(&robot,	invertedCamera, 
			ArVCC4::COMM_UNKNOWN, true, true);
    camera->init();

    cameraCollection = new ArCameraCollection();
    cameraCollection->addCamera("Cam1", "VCC4", "Camera", "VCC4");
    handlerCamera = new ArServerHandlerCamera("Cam1", 
		                              &server, 
					      &robot,
					      camera, 
					      cameraCollection);
  }

  // You can use this class to send a set of arbitrary strings 
  // for MobileEyes to display, this is just a small example
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));
  /*
  Aria::getInfoGroup()->addStringInt(
	  "Laser Packet Count", 10, 
	  new ArRetFunctorC<int, ArSick>(&sick, 
					 &ArSick::getSickPacCount));
  */
  
  // start the robot running, true means that if we lose connection the run thread stops
  robot.enableMotors();
  robot.runAsync(true);


  // connect the laser(s) if it was requested
  if (!laserConnector.connectLasers())
  {
    printf("Could not connect to lasers... exiting\n");
    Aria::exit(2);
  }
  

  drawings.addRobotsRangeDevices(&robot);

  // log whatever we wanted to before the runAsync
  simpleOpener.checkAndLog();
  // now let it spin off in its own thread
  server.runAsync();

  printf("Server is now running...\n");

  // Add a key handler so that you can exit by pressing
  // escape. Note that a key handler prevents you from running
  // a program in the background on Linux, since it expects an 
  // active terminal to read keys from; remove this if you want
  // to run it in the background.
  ArKeyHandler *keyHandler;
  if ((keyHandler = Aria::getKeyHandler()) == NULL)
  {
    keyHandler = new ArKeyHandler;
    Aria::setKeyHandler(keyHandler);
    robot.lock();
    robot.attachKeyHandler(keyHandler);
    robot.unlock();
    printf("To exit, press escape.\n");
  }
  
  //user code   



  clientSwitchManager.runAsync();

  robot.waitForRunExit();
  Aria::exit(0);
}


