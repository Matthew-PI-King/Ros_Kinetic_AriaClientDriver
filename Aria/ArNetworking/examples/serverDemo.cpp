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
ArRobot robot; //to use it globally
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
  ArGlobalFunctor2<ArServerClient *, ArNetPacket *> laserRequestCB(&laserRequest); //this is used for laserdata requests  

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
  server.addData("LaseRequest", "custom command for server to enable laser data", &laserRequestCB, "none", "none"); //laser data request

  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  ArServerSimpleOpener simpleOpener(&parser);


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

  clientSwitchManager.runAsync();
  std::map<int, ArLaser*> *lasers = robot.getLaserMap();
  // Print out some data from each connected laser.
  while(robot.isConnected())
  {
	int numLasers = 0;

 	  // Get a pointer to ArRobot's list of connected lasers. We will lock the robot while using it to prevent changes by tasks in the robot's background task thread or any other threads. Each laser has an index. You can also store the laser's index or name (laser->getName()) and use that to get a reference (pointer) to the laser object using ArRobot::findLaser().
	  robot.lock();
	  std::map<int, ArLaser*> *lasers = robot.getLaserMap();

	  for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
	  {
		int laserIndex = (*i).first;
		ArLaser* laser = (*i).second;
		if(!laser)
			continue;
		++numLasers;
		laser->lockDevice();

				

		ArRangeBuffer* rangeBuffer=laser->getCumulativeRangeBuffer();
		ArPose readingPose=rangeBuffer->getPoseTaken();
		// The current readings are a set of obstacle readings (with X,Y positions as well as other attributes) that are the most recent set from teh laser.
		std::list<ArPoseWithTime*> *currentReadings = laser->getCurrentBuffer(); // see ArRangeDevice interface doc
		for (std::list<ArPoseWithTime*>::iterator it= currentReadings->begin(); it != currentReadings->end(); ++it){
			ArPoseWithTime* laserRead =*it;
    			//printf("%d,",laserRead->findAngleTo(readingPose));
		}

		const std::list<ArSensorReading*> *sensorReadings = laser->getRawReadings(); // see ArRangeDevice interface doc
		for (std::list<ArSensorReading*>::const_iterator it2= sensorReadings->begin(); it2 != sensorReadings->end(); ++it2){
			ArSensorReading* laserRead =*it2;
    			//printf("%i,%i:",laserRead->getRange(),laserRead->getIgnoreThisReading());
		}
		

		// There is a utility to find the closest reading wthin a range of degrees around the laser, here we use this laser's full field of view (start to end)
		// If there are no valid closest readings within the given range, dist will be greater than laser->getMaxRange().
		double angle = 0;
		double dist = laser->currentReadingPolar(laser->getStartDegrees(), laser->getEndDegrees(), &angle);

		ArLog::log(ArLog::Normal, "Laser #%d (%s): %s. Have %d 'current' readings. Closest reading is at %3.0f degrees and is %2.4f meters away.", laserIndex, laser->getName(), (laser->isConnected() ? "connected" : "NOT CONNECTED"), currentReadings->size(), angle, dist/1000.0);
		//print laser configuration
		ArLog::log(ArLog::Normal,"(%s).%d.%d.%d",laser->getName(),laser->getStartDegrees(),laser->getEndDegrees(),laser->getIncrement());
		printf("start degrees :%d\nEnd degrees:%d\nIncrement:%s\n",laser->getStartDegrees(),laser->getEndDegrees(),laser->getName());

                laser->unlockDevice();
	    }
	if(numLasers == 0)
		ArLog::log(ArLog::Normal, "No lasers.");
	else
		ArLog::log(ArLog::Normal, "");

        // Unlock robot and sleep for 5 seconds before next loop.
	robot.unlock();
  	ArUtil::sleep(1000);
   }
   
 
  robot.waitForRunExit();
  Aria::exit(0);
}


