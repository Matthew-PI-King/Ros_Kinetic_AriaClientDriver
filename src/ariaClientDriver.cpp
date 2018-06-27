/*
 * ariaClientDriver.cpp
 *
 *  Created on: Mar 2, 2013
 *      Author: islab2
 *
 * 
 * Change log
 * v1.1 - working bear minimum command line robot name parsing
 * v1.2 - tf prefix compatible no robot name parsing required (ros launch groups prefered instead)my
 * v1.4 - direct motion commands (unsafe) - server modified 
 * v1.5 - raw+pointcloud data for debug -- laser scan to point cloud (no need) --> synced_odom+laser request (server demo 2 modified) (/pioneer/Ariacmdvel for ration drive /pioneer/cmdvel for direct   motion (need server demo 3))
 *
 *  Updated on: April 31st, 2018
 *	Maintainer: Matthew King mpik32@mun.ca
 *
 * Change log
 * v2.0 - Updated for ROS kinetic. Removed old and unused code. Added Automatic Laser FOV detection. Fixed issue where scan of >180 points causing crash.
 *
 *
 */

#include <std_msgs/Bool.h>
#include "ariaClientDriver.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/NavSatFix.h>
#include <ariaclientdriver/wEncoder.h>
#include <ariaclientdriver/AriaNavData.h>
#include <ariaclientdriver/AriaCommandData.h>


//#include "ros/ros.h"

using namespace ariaclientdriver;

AriaClientDriver::AriaClientDriver(ArClientBase *client, ArKeyHandler *keyHandler, std::string robotName) :

  myClient(client), myKeyHandler(keyHandler),
  myTransRatio(0.0), myRotRatio(0.0), myLatRatio(0.0), myMaxVel(50),
  myPrinting(false),   myNeedToPrintHeader(false),   myGotBatteryInfo(false), myNeedToInitialize(true), myDirectMotionEnabler(false), myXbias(0.0),myYbias(0.0),myThbias(0.0),
  minLaserAngle(-M_PI/2), maxLaserAngle(M_PI/2), //default FOV 180 degrees centered forward.
  /* Initialize functor objects with pointers to our handler methods: */
  myUpCB(this, &AriaClientDriver::up),
  myDownCB(this, &AriaClientDriver::down),
  myLeftCB(this, &AriaClientDriver::left),
  myRightCB(this, &AriaClientDriver::right),
  myLateralLeftCB(this, &AriaClientDriver::lateralLeft),
  myLateralRightCB(this, &AriaClientDriver::lateralRight),
  mySafeDriveCB(this, &AriaClientDriver::safeDrive),
  myUnsafeDriveCB(this, &AriaClientDriver::unsafeDrive),
  myStepBackCB(this, &AriaClientDriver::stepBack),
  myListDataCB(this, &AriaClientDriver::listData),
  myLogTrackingTerseCB(this, &AriaClientDriver::logTrackingTerse),
  myLogTrackingVerboseCB(this, &AriaClientDriver::logTrackingVerbose),
  myResetTrackingCB(this, &AriaClientDriver::resetTracking),
  myHandleBatteryInfoCB(this, &AriaClientDriver::handleBatteryInfo),
  myHandlePhysicalInfoCB(this, &AriaClientDriver::handlePhysicalInfo),
  myHandleTemperatureInfoCB(this, &AriaClientDriver::handleTemperatureInfo),
  myHandleSensorInfoCB(this, &AriaClientDriver::handleSensorInfo),
  myHandleRangeDataCB(this, &AriaClientDriver::handleRangeData),
  mygetLaserMetaDataCB(this, &AriaClientDriver::handleLaserMetaData),
  mygetGpsDataCB(this, &AriaClientDriver::handleGpsData),
  mygetwEncoderDataCB(this, &AriaClientDriver::handleEncoderData)
{
   printf("Initializing \n");
   //workaround for tf_prefix issue for laser
   myRosNodeHandle.getParam("tf_prefix", myRobotName);

   //KDL Tree and state publisher
   std::string robot_desc_string;
   myRosNodeHandle.getParam("/robot_description", robot_desc_string);
   if (!kdl_parser::treeFromString(robot_desc_string, myKdlTree)){

      ROS_ERROR("Failed to construct kdl tree\n please load urdf to parameter server:robot_description_pioneer1");
   }
   else{
	printf("KDL parser::OK\n");}
   
  	//TODO: FIX BELOW LINE!
   myRobotStatePublisher= new robot_state_publisher::RobotStatePublisher(myKdlTree);	

   //critical thread performing command updates
   //this is for direct motion command
   myTwistSubscribe = myRosNodeHandle.subscribe("cmd_vel", 1, &AriaClientDriver::topicCallBack2,this);
   myTwistSubscribe2 = myRosNodeHandle.subscribe("cmd_vel_ratio", 1, &AriaClientDriver::topicCallBack3,this);
   myDirectEnablerSubscribe = myRosNodeHandle.subscribe("direct_enable", 1, &AriaClientDriver::topicCallBack4,this);
   myLaserPublish = myRosNodeHandle.advertise<sensor_msgs::LaserScan>("scan", 50);
   myLaserPublish2 = myRosNodeHandle.advertise<sensor_msgs::PointCloud>("scan2", 50);
   myOdomPublish = myRosNodeHandle.advertise<nav_msgs::Odometry>("odom", 50);
   myGpsPublish= myRosNodeHandle.advertise<sensor_msgs::NavSatFix>("gps", 50);
   myEncoderPublish = myRosNodeHandle.advertise<ariaclientdriver::wEncoder>("wheelEncoder", 50);

   // Critical thread performing sensor data acquisition

     //row laser + synced odom request
     myClient->addHandler("LaserRequest_odom", &myHandleRangeDataCB);
     myClient->request("LaserRequest_odom", 100);
   
     myClient->addHandler("getGPS", &mygetGpsDataCB);
     myClient->request("getGPS", 100);

     myClient->addHandler("getwEncoder", &mygetwEncoderDataCB);
     //myClient->request("getwEncoder", 100);
     myClient->requestOnce("getwEncoder");

  //myCmdVelSubscribe = myRosNodeHandle.subscribe<std_msgs::String>("AriaCmdVel", 1, Foo());

  // Handlers(Threads) for Keyboard input
  myKeyHandler->addKeyHandler(ArKeyHandler::UP, &myUpCB); //calls a method of an object pointer
  myKeyHandler->addKeyHandler(ArKeyHandler::DOWN, &myDownCB);//double colon used for namespace
  myKeyHandler->addKeyHandler(ArKeyHandler::LEFT, &myLeftCB);
  myKeyHandler->addKeyHandler(ArKeyHandler::RIGHT, &myRightCB);
  myKeyHandler->addKeyHandler('q', &myLateralLeftCB);
  myKeyHandler->addKeyHandler('e', &myLateralRightCB);
  myKeyHandler->addKeyHandler('s', &mySafeDriveCB);
  myKeyHandler->addKeyHandler('u', &myUnsafeDriveCB);
  myKeyHandler->addKeyHandler('l', &myListDataCB);
  myKeyHandler->addKeyHandler('t', &myLogTrackingTerseCB);
  myKeyHandler->addKeyHandler('v', &myLogTrackingVerboseCB);
  myKeyHandler->addKeyHandler('r', &myResetTrackingCB);
  myKeyHandler->addKeyHandler('o', &myStepBackCB);



  // Handlers(Threads) for Service calls
  
  myClient->addHandler("physicalInfo", &myHandlePhysicalInfoCB);
  myClient->requestOnce("physicalInfo");
  myClient->addHandler("batteryInfo", &myHandleBatteryInfoCB);
  myClient->requestOnce("batteryInfo");
  if (myClient->dataExists("temperatureInfo"))
  {
	  myClient->addHandler("temperatureInfo", &myHandleTemperatureInfoCB);
	  myClient->requestOnce("temperatureInfo");
  }
  if (myClient->dataExists("getSensorList"))
  {
	  myClient->addHandler("getSensorList", &myHandleSensorInfoCB);
	  myClient->requestOnce("getSensorList");
  }

  if (myClient->dataExists("getLaserMetaData"))
  {
	  myClient->addHandler("getLaserMetaData", &mygetLaserMetaDataCB);
	  myClient->requestOnce("getLaserMetaData");
  }

  unsafeDrive();
  printf("Aria Client Driver node started...\n");
//printf("Debugging...\n");
//myClient->logDataList();
}

AriaClientDriver::~AriaClientDriver(void)
{
	myClient->requestStop("update");
}

void AriaClientDriver::up(void)
{
  if (myPrinting)
    printf("Forwards\n");
  myTransRatio = 100;
}

void AriaClientDriver::down(void)
{
  if (myPrinting)
    printf("Backwards\n");
  myTransRatio = -100;
}

void AriaClientDriver::left(void)
{
  if (myPrinting)
    printf("Left\n");
  myRotRatio = 100;
}

void AriaClientDriver::right(void)
{
  if (myPrinting)
    printf("Right\n");
  myRotRatio = -100;
}

void AriaClientDriver::lateralLeft(void)
{
  if (myPrinting)
    printf("Lateral left\n");
  myLatRatio = 100;
}

void AriaClientDriver::lateralRight(void)
{
  if (myPrinting)
    printf("Lateral right\n");
  myLatRatio = -100;
}

void AriaClientDriver::safeDrive()
{
  /* Construct a request packet. The data is a single byte, with value
   * 1 to enable safe drive, 0 to disable. */
  ArNetPacket p;
  p.byteToBuf(1);

  /* Send the packet as a single request: */
  if(myPrinting)
    printf("Sending setSafeDrive 1.\n");
  myClient->requestOnce("setSafeDrive",&p);
  if(myPrinting)
    printf("\nSent enable safe drive.\n");
}

void AriaClientDriver::unsafeDrive()
{
  /* Construct a request packet. The data is a single byte, with value
   * 1 to enable safe drive, 0 to disable. */
  ArNetPacket p;
  p.byteToBuf(0);

  /* Send the packet as a single request: */
  if(myPrinting)
    printf("Sending setSafeDrive 0.\n");
  myClient->requestOnce("setSafeDrive",&p);
  if(myPrinting)
    printf("\nSent disable safe drive command. Your robot WILL run over things if you're not careful.\n");
}


void AriaClientDriver::stepBack(){
  ArNetPacket request;
  request.empty();
  printf("Requesting step back %lf\n", 1000.0);
  request.byte2ToBuf((ArTypes::Byte2)(1000.0));
  myClient->requestOnce("MoveStepRequest",&request);
}

void AriaClientDriver::listData()
{
  myClient->logDataList();
}

void AriaClientDriver::logTrackingTerse()
{
  myClient->logTracking(true);
}

void AriaClientDriver::logTrackingVerbose()
{
  myClient->logTracking(false);
}

void AriaClientDriver::resetTracking()
{
  myClient->resetTracking();
}

void AriaClientDriver::topicCallBack2(const geometry_msgs::Twist &msg) //direct motion command (need server demo 3)
{
    if(myDirectMotionEnabler){
	//should modify arnl server to accept velocities
	//myTransRatio		=msg.linear.x*10.0;
	//myRotRatio		=msg.angular.z*10.0;
	//myLatRatio		=msg.linear.y*10.0;
	//myMaxVel		=100.0;
	double vel=msg.linear.x*1000.0;
 	double rotVel=msg.angular.z/M_PI*180.0;

	ArNetPacket request;
  	request.empty();
  	//printf("Requesting set vel %lf\n", vel);
  	request.byte2ToBuf((ArTypes::Byte2)(vel));
	request.byte2ToBuf((ArTypes::Byte2)(rotVel));
  	myClient->requestOnce("SetVelRequest",&request);}
}

void AriaClientDriver::topicCallBack3(const geometry_msgs::Twist &msg) //ratio motion command
{
	//should modify arnl server to accept velocities
	myTransRatio		=msg.linear.x*10.0;
	myRotRatio		=msg.angular.z*10.0;
	myLatRatio		=msg.linear.y*10.0;
	myMaxVel		=100.0;
}

void AriaClientDriver::topicCallBack4(const std_msgs::Bool &msg){
        myDirectMotionEnabler = msg.data;
}

void AriaClientDriver::sendInput() //generic motion command
{
  /* This method is called by the main function to send a ratioDrive
   * request with our current velocity values. If the server does
   * not support the ratioDrive request, then we abort now: */
  if(!myClient->dataExists("ratioDrive")) return;

  /* Construct a ratioDrive request packet.  It consists
   * of three doubles: translation ratio, rotation ratio, and an overall scaling
   * factor. */



  ArNetPacket packet;
  packet.doubleToBuf(myTransRatio);
  packet.doubleToBuf(myRotRatio);
  packet.doubleToBuf(myMaxVel); // use half of the robot's maximum.
  packet.doubleToBuf(myLatRatio);
  if (myPrinting)
    printf("Sending\n");
  myClient->requestOnce("ratioDrive", &packet);
  myTransRatio = 0;
  myRotRatio = 0;
  myLatRatio = 0;
}


void AriaClientDriver::handleBatteryInfo(ArNetPacket *packet)
{
  /* Get battery configuration parameters: when the robot will begin beeping and
   * warning about low battery, and when it will automatically disconnect and
   * shutdown. */
  double lowBattery = packet->bufToDouble();
  double shutdown = packet->bufToDouble();
  printf("Low battery voltage: %6g       Shutdown battery voltage: %6g\n", lowBattery, shutdown);
  fflush(stdout);
  myNeedToPrintHeader = true;
  myGotBatteryInfo = true;

  if (packet->getDataReadLength() == packet->getDataLength())
  {
    printf("Packet is too small so its an old server, though you could just get to the bufToUByte anyways, since it'd be 0 anyhow\n");
    myVoltageIsStateOfCharge = false;
  }
  else
    myVoltageIsStateOfCharge = (packet->bufToUByte() == 1);

}


void AriaClientDriver::handlePhysicalInfo(ArNetPacket *packet)
{
  /* Get phyiscal configuration parameters: */
  char robotType[512];
  char robotSubtype[512];
  int width;
  int lengthFront;
  int lengthRear;

  packet->bufToStr(robotType, sizeof(robotType));
  packet->bufToStr(robotSubtype, sizeof(robotSubtype));
  width = packet->bufToByte2();
  lengthFront = packet->bufToByte2();
  lengthRear = packet->bufToByte2();

  printf("Type: %s Subtype: %s Width %d: LengthFront: %d LengthRear: %d\n",
	 robotType, robotSubtype, width, lengthFront, lengthRear);
  fflush(stdout);
}

void AriaClientDriver::handleTemperatureInfo(ArNetPacket *packet)
{
  char warning = packet->bufToByte();
  char shutdown = packet->bufToByte();
  printf("High temperature warning: %4d       High temperature shutdown: %4d\n", warning, shutdown);
  fflush(stdout);
  myNeedToPrintHeader = true;
}

void AriaClientDriver::handleSensorInfo(ArNetPacket *packet)
{
  int numberOfSensors = packet->bufToByte2();
  printf("I have %d Range sensors \n", numberOfSensors);
  for(int i=0;i<numberOfSensors;i++){
	  memset(mySensors, 0, sizeof(mySensors));
	  packet->bufToStr(mySensors, sizeof(mySensors));
	  printf("%d    %s\n",i+1, mySensors);
  }
  fflush(stdout);
}

void AriaClientDriver::handleRangeData(ArNetPacket *packet)
{
   //printf("Recieved Packet \n");
   //Decode packet (num_of readings, readings[0..180], myX, myY, myTh, myVel, myRotvel)
   int NumberOfReadings;//=541;
   NumberOfReadings = (int)(double) packet->bufToByte4();
   for (int i=0; i<NumberOfReadings; i++){
    	myLaserReading[i]=packet->bufToByte4();
        //printf("%1.0f, ",myLaserReadin();
	}
    myX = (double) packet->bufToByte4();
    myY = (double) packet->bufToByte4();
    myTh = (double) packet->bufToByte4();
    myVel = (double) packet->bufToByte4();
    myRotVel = (double) packet->bufToByte4();
    //myLatVel = 0;
    /* printf("\n%6s|%6s|%6s|%6s|%6s|%6s|%4s|%6s|%15s|%20s|\n",
	   "x","y","theta", "vel", "rotVel", "latVel", "temp", myVoltageIsStateOfCharge ? "charge" : "volts", "mode","status");
	    printf("%6.0f|%6.0f|%6.1f|%6.0f|%6.0f|%6.0f|%4.0d|%6.1f|%15s|%20s|\r",
	   myX, myY, myTh, myVel, myRotVel, myLatVel, myTemperature, myVoltage, myMode, myStatus);
    fflush(stdout);*/
   
  
    //Assemble and publish laser scan massege
      sensor_msgs::LaserScan msg;
      msg.header.stamp=ros::Time::now();
      msg.header.frame_id=(std::string("/")+myRobotName+std::string("/base_laser")).c_str();
      msg.angle_min=-3*M_PI/4;        			// start angle of the scan [rad]
      //msg.angle_min=minLaserAngle;
      msg.angle_max=3*M_PI/4;       		// end angle of the scan [rad]
      //msg.angle_max=maxLaserAngle;
      //msg.angle_increment=(maxLaserAngle-minLaserAngle)/(NumberOfReadings-1);  	// angular distance between measurements [rad] 
      msg.angle_increment=(1.5*M_PI)/(NumberOfReadings-1);
      msg.time_increment=0.00024;//0.01333/180;   // time between measurements [seconds] ()75Hz motor
      msg.scan_time=0.03;//0.3;        		// time between scans [seconds]
      msg.range_min=0;        		// minimum range value [m]
      msg.range_max=30;        			// maximum range value [m]
      msg.ranges.resize(NumberOfReadings);
      msg.intensities.resize(NumberOfReadings);
      for (int i=0; i < NumberOfReadings; i++) {   	// range data [m] (Note: values < range_min or > range_max should be discarded)
    	  //printf("%f\n",myLaserReading[i]);
    	  msg.ranges[i]=(float)myLaserReading[i]/1000.0;
    	  msg.intensities[i]=1;
      }
      myLaserPublish.publish(msg);
       
	if(myNeedToInitialize){
	 	printf("Initializing odometer...\n ");
		
		myXbias=myX;
		myYbias=myY;
		myThbias=myTh;
		printf("Bias X:%f \tBias Y:%f \tBias Th:%f \t\n",myXbias,myYbias,myThbias);
	 	myNeedToInitialize=false;
	}

	//corected pose
        double x_corr=cos((myThbias)/180*M_PI)*(myX-myXbias)+sin((myThbias)/180*M_PI)*(myY-myYbias);
        double y_corr=-sin((myThbias)/180*M_PI)*(myX-myXbias)+cos((myThbias)/180*M_PI)*(myY-myYbias);
    

    //Assemble and publish odometry massege  
    	ros::Time current_time=ros::Time::now();
  	nav_msgs::Odometry odom;
  	odom.header.stamp = current_time;
  	odom.header.frame_id ="odom";
  	odom.pose.pose.position.x = (x_corr)/1000.0;
  	odom.pose.pose.position.y = (y_corr)/1000.0;
  	odom.pose.pose.position.z = 0.0;
  	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw((myTh-myThbias)/180*M_PI);
  	odom.child_frame_id = "pioneer1/base_link";
  	odom.twist.twist.linear.x = myVel/1000.0;
  	odom.twist.twist.linear.y = myLatVel/1000.0;
  	odom.twist.twist.angular.z = myRotVel/180*M_PI;
  	myOdomPublish.publish(odom);
    	myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,(myTh-myThbias)/180*M_PI), tf::Vector3((x_corr)/1000.0, (y_corr)/1000.0, 271/1000.0)),current_time,"odom","pioneer1/base_link"));
	//myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,0), tf::Vector3(0.0,0.0,0.0)),current_time,"odom","pioneer1/base_link"));
  	//publish robot transforms

  		//TODO: FIX BELOW LINE!
  	myRobotStatePublisher->publishFixedTransforms(myRobotName); 
	//myRobotStatePublisher->publishFixedTransforms(); 
        //printf("Initializing pose...\n ");

	
}

void  AriaClientDriver::handleLaserMetaData(ArNetPacket *packet){

    minLaserAngle = M_PI*((double) packet->bufToByte4())/180; //convert degrees to radians.
    maxLaserAngle = M_PI*((double) packet->bufToByte4())/180;

}


//Callback function which recieved GPS data from the server and formats it for ROS.
void AriaClientDriver::handleGpsData(ArNetPacket *packet){

  //printf("Recieved GPS data");
  //Packet Format [Lat, Lon, Altitude, FixType, numSattellitesTracked, PDOP, HDOP, VDOP]

  float myLat = ((float) packet->bufToByte4())/1048576; //Previously there were sig figs which were being lost during conversion. To fix this, we multiply by 2^20 before sending and divide after. 
  float myLon = ((float) packet->bufToByte4())/1048576;
  float myAlt = ((float) packet->bufToByte4())/1048576;

  //double myLat = packet->bufToByte8();
  //double myLon = packet->bufToByte8();
  //double myAlt = packet->bufToByte8();

  int fixType = (int) packet->bufToByte4(); //different encoding.
  int numSats = (int) packet->bufToByte4();
  float PDOP = (float) packet->bufToByte4();
  float HDOP = (float) packet->bufToByte4();
  float VDOP = (float) packet->bufToByte4();
  
  //printf("lat:%f lon:%f alt:%f fixType:%i nSats:%d PDOP:%f HDOP:%f VDOP:%f", myLat, myLon, myAlt, fixType, numSats, PDOP, HDOP, VDOP);

  sensor_msgs::NavSatFix msg;

  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=(std::string("/")+myRobotName+std::string("/base_link")).c_str();

  msg.status.status = 0; //FIXME
  msg.status.service = 1; //FIXME these should not be hardcoded.

  msg.latitude = myLat;
  msg.longitude = myLon;
  msg.altitude = myAlt;

  //msg.position_covariance = []; 3x3 covariance matrix?
  msg.position_covariance_type = 0; //FIXME 0 means unknown but it should be calculated based on PDOP HDOP and VDOP.
  
  myGpsPublish.publish(msg);
}

//Callback functionwhich recieves wheel encoder data from the server and formats it for ROS.
void AriaClientDriver::handleEncoderData(ArNetPacket *packet){
  /*
  double x =((double) packet->bufToByte8())*0.001; 	//X velocity?
  double y =((double) packet->bufToByte8())*0.001; 	//Y velocity?
  double theta =((double) packet->bufToByte8())*0.001534; //Ask Patrick about these constants.
  double left =((double) packet->bufToByte8())*0.001;
  double right =((double) packet->bufToByte8())*0.001;
  */

  //packet->bufToByte2(); //discard the header

  float x =((float) packet->bufToByte4()); 	
  float y =((float) packet->bufToByte4()); 	
  float theta =((float) packet->bufToByte4());
  float left =((float) packet->bufToByte4());
  float right =((float) packet->bufToByte4());

  ariaclientdriver::wEncoder msg;
  msg.X=x;
  msg.Y=y;
  msg.Th=theta;
  msg.VelLeft=left;
  msg.VelRight=right;

  myEncoderPublish.publish(msg);
}

void AriaClientDriver::controlloop(){
	ros::Rate myLoopRate(10);
	sendInput();
	ros::spinOnce();
	myLoopRate.sleep();
}



