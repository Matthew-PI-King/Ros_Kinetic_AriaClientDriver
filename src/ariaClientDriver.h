/*
 * ariaClientDriver.h
 *
 *  Created on: Mar 2, 2013
 *      Author: islab2
 */
//#include "../Aria/include/Aria.h"
//#include "ros/ros.h"
//#include "../Aria/ArNetworking/include/ArNetworking.h"
//#include "Aria.h"

class AriaClientDriver
{
public:

  //Constructor and Destructor
  AriaClientDriver(ArClientBase *client, ArKeyHandler *keyHandler,std::string robotName);
  virtual ~AriaClientDriver(void);

  //Teleoperation
  void up(void);			/// Up arrow key handler: drive the robot forward
  void down(void);			/// Down arrow key handler: drive the robot backward
  void left(void);			/// Left arrow key handler: turn the robot left
  void right(void);			/// Right arrow key handler: turn the robot right
  void lateralLeft(void);	/// Move the robot laterally right  (q key)
  void lateralRight(void);	/// Move the robot laterally right  (e key)

  //Robot Command Requests   use as a call back for a subscriber
  //use sendinput itself as the call back for constant control (i.e. sustained control inputs available)
  //The two methods below makes the robot stop when theres no control inputs
  void topicCallBack2(const geometry_msgs::Twist &msg);		/// Decode incoming Twist masseges
  void topicCallBack3(const geometry_msgs::Twist &msg);		/// ratio command
  void topicCallBack4(const std_msgs::Bool &msg);		/// direct motion enabler
  void sendInput();												/// Send drive request to the server with stored values

  


  //Robot Service Requests
  void safeDrive();			/// Send a request to enable "safe drive" mode on the server
  void unsafeDrive();		/// Send a request to disable "safe drive" mode on the server
  void stepBack();
  void listData();
  void logTrackingTerse();
  void logTrackingVerbose();
  void resetTracking();

  void handleBatteryInfo(ArNetPacket *packet);		/// This callback is called when an update on the battery configuration changes
  void handlePhysicalInfo(ArNetPacket *packet);		/// This is called when the physical robot information comes back
  void handleTemperatureInfo(ArNetPacket *packet);	/// This callback is called when an update on the temperature information changes
  void handleSensorInfo(ArNetPacket *packet);		/// This callback is called when an update on the temperature information changes
  void handleGpsData(ArNetPacket *packet);


  void controlloop();
  //ros objects
  ros::NodeHandle myRosNodeHandle;

  ros::Subscriber myCmdVelSubscribe; 			// this used a custom massege
  ros::Subscriber myTwistSubscribe;	//should subscribe to  standard massesge twist (nav stack prerequisit)
  ros::Subscriber myTwistSubscribe2;
  ros::Subscriber myDirectEnablerSubscribe;

  ros::Publisher myNavdataPublish;
  ros::Publisher myLaserPublish;				//Publish standard massesge laser data (nav stack prerequisit)
  ros::Publisher myOdomPublish;					//Publish standard massesge Odometry (nav stack prerequisit)
  ros::Publisher myLaserPublish2;
  ros::Publisher myGpsPublish;
  ros::Publisher myEncoderPublish;  //semi-processed encoder data (for "processing" see serverDemo3 or equivalent).

  ros::ServiceServer myLaserLocalizationService;
  ros::ServiceServer mySafeDriveService;

  tf::TransformBroadcaster myTfBroadcaster;		//Broadcast standard massesge transform (nav stack prerequisit)
  
  KDL::Tree myKdlTree;		
			//used for visualization
  robot_state_publisher::RobotStatePublisher* myRobotStatePublisher;		//used to easily handle kdl transformations
  //ros::Rate myLoopRate();

protected:
  ArClientBase *myClient;
  ArKeyHandler *myKeyHandler;


  //// The command velocities
  double myTransRatio;	/// Current translation value (a percentage of the  maximum speed)
  double myRotRatio;	/// Current rotation ration value (a percentage of the maximum rotational velocity)
  double myLatRatio;	/// Current rotation ration value (a percentage of the maximum rotational velocity)
  double myMaxVel;

  //// The robots sensor readings
  double myX;
  double myY;
  double myTh;
  double myVel;
  double myRotVel;
  double myLatVel;
  bool myVoltageIsStateOfCharge;
  char myTemperature;
  double myVoltage;
  std::string myRobotName;
  char myStatus[256];
  char myMode[256];
  char mySensors[256];
  double myLaserReadingX[181];
  double myLaserReadingY[181];
  double myLaserReading[1024];

  // Other member variables
  bool myPrinting;
  bool myNeedToPrintHeader;
  bool myGotBatteryInfo;
  bool myNeedToInitialize;
  bool myDirectMotionEnabler;

  double myXbias;
  double myYbias;
  double myThbias;

  double minLaserAngle; //radians
  double maxLaserAngle; //radians

  //Robot Data Requests      These are call back for data requests p->publish inside
  void handleRangeData(ArNetPacket *packet);		/// This callback is called when an update on the temperature information changes
  void handleLaserMetaData(ArNetPacket *packet);
  void handleEncoderData(ArNetPacket *packet);



  /// Functors for each method of the class
  ArFunctorC<AriaClientDriver> myUpCB;  //creating a functor(similar to class) named myUpCB from template
  ArFunctorC<AriaClientDriver> myDownCB;
  ArFunctorC<AriaClientDriver> myLeftCB;
  ArFunctorC<AriaClientDriver> myRightCB;
  ArFunctorC<AriaClientDriver> myLateralLeftCB;
  ArFunctorC<AriaClientDriver> myLateralRightCB;
  ArFunctorC<AriaClientDriver> mySafeDriveCB;
  ArFunctorC<AriaClientDriver> myUnsafeDriveCB;
  ArFunctorC<AriaClientDriver> myStepBackCB;
  ArFunctorC<AriaClientDriver> myListDataCB;
  ArFunctorC<AriaClientDriver> myLogTrackingTerseCB;
  ArFunctorC<AriaClientDriver> myLogTrackingVerboseCB;
  ArFunctorC<AriaClientDriver> myResetTrackingCB;
  ArFunctorC<AriaClientDriver> mySquareDriveCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> myHandleBatteryInfoCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> myHandlePhysicalInfoCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> myHandleTemperatureInfoCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> myHandleSensorInfoCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> myHandleRangeDataCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> mygetLaserMetaDataCB; //laser angles.
  ArFunctor1C<AriaClientDriver, ArNetPacket *> mygetGpsDataCB;
  ArFunctor1C<AriaClientDriver, ArNetPacket *> mygetwEncoderDataCB;

};



