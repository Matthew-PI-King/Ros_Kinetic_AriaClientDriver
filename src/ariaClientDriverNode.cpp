/*
 * ariaClientDriverNode.cpp
 *
 *
 *  Created on: Mar 2, 2013
 *      Author: islab2
 */


#include "Aria.h"
#include "ArNetworking.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

//Ros masseges
//#include "../msg_gen/cpp/include/ariaClientDriver/AriaNavData.h"
//#include "../msg_gen/cpp/include/ariaClientDriver/AriaCommandData.h"



#include <sstream>
//#include <stdio.h>
//#include <stdlib.h>
#include <string>
#include <iostream>

//The driver class def
#include "ariaClientDriver.cpp"


void escape(void)
{
	printf("esc pressed, shutting down aria\n");
	Aria::shutdown();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ariaClientDriverNode");	//ROS Initialization


	Aria::init();										//Aria Initialization
	ArClientBase client;								//setup client
	ArArgumentParser parser(&argc, argv);				//command line argument handler
	ArClientSimpleConnector clientConnector(&parser);	//connect to Arserver

	parser.loadDefaultArguments();
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		exit(0);
	}

	if (!clientConnector.connectClient(&client))
	{
		if (client.wasRejected())
			printf("Server '%s' rejected connection, exiting\n", client.getHost());
		else
			printf("Could not connect to server '%s', exiting\n", client.getHost());
		exit(1);
	}
	printf("Connected to server.\n");

	client.setRobotName(client.getHost()); // include server name in log messages
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	ArGlobalFunctor escapeCB(&escape);
	keyHandler.addKeyHandler(ArKeyHandler::ESCAPE, &escapeCB);
	client.runAsync();

	if(!client.dataExists("ratioDrive") )
		printf("Warning: server does not have ratioDrive command, can not use drive commands!\n");
	else
		printf("Keys are:\nUP: Forward\nDOWN: Backward\nLEFT: Turn Left\nRIGHT: Turn Right\n");
	printf("s: Enable safe drive mode (if supported).\nu: Disable safe drive mode (if supported).\nl: list all data requests on server\n\nDrive commands use 'ratioDrive'.\nt: logs the network tracking tersely\nv: logs the network tracking verbosely\nr: resets the network tracking\n\n");


	AriaClientDriver ariaClientDriver(&client,&keyHandler,"");

	//while (ros::ok() && client.getRunningWithLock()) //the main loop
	while (client.getRunningWithLock()) //the main loop
	{
		keyHandler.checkKeys();  //addthis if teleop from node required
		ariaClientDriver.controlloop();
		//Input output handling callback threads implemented in ariaClientDriver Class
		//ArUtil::sleep(100);	//noneed

	}

	client.disconnect();
	Aria::shutdown();
	return 0;
}



