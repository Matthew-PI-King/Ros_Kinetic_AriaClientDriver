# ariaClientDriver

![alt text](https://cloud.githubusercontent.com/assets/10774875/9448012/cbaaf228-4a74-11e5-9531-0b07e5f585ba.png "islabexp")

This is a Driver node for pioneer robots (or any robot) running using ARIA libraries of MoblieRobots Inc. This driver does not need a ROS installation on the robots host computer. Therefore this package is suitable for pioneer robots equipped with older computers which makes it difficult to install ROS. If you prefer to install ROS on the robots`s host computer, ROSARIA package is recomended (http://wiki.ros.org/ROSARIA).

The package is a client software with a ROS wrapper which connects with ARIA SDK based servers running on robots. The user simply runs the server on the robot`s host computer and runs the ROS client on a remote machine. This allows a quick and easy way to integrate robots running ARIA to a ROS based multi-robot framework.

## Installation
1. Install the following from the mobile robots site (The instructions assumes a 32 bit Ubuntu system. For other systems please find the corresponding files in these links):

	* Aria SDK: Navigate to  http://robots.mobilerobots.com/wiki/ARIA and install ARIA 2.9.0 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture. 
	
	* Mobilesim Simulator: Navigate to http://robots.mobilerobots.com/wiki/MobileSim and install MobileSim 0.7.3 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture

	* ARNL Libraries (Optional- Not required for simulation): Navigate to http://robots.mobilerobots.com/wiki/ARNL,_SONARNL_and_MOGS and install Base ARNL Library 1.9.0 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture  and ARNL Laser Localization Library 1.9.0 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture.
	
2. The package is tested on ROS fuerte using rosbuild system on 32-bit Ubuntu 12.04.
To install and configure ROS fuerte please use the steps detailed in the following link: http://wiki.ros.org/fuerte/Installation/Ubuntu 

3. Use the following steps to configure a ros_workspace directory (if not already configured):
 ```bash
$ cd ~
$ mkdir ros_workspace
$ echo "export ROS_PACKAGE_PATH=/opt/ros/fuerte/share:/opt/ros/fuerte/stacks:~/ros_workspace" >> ~/.bashrc
```
 
4. Download the zip package and extract in the ros_workspace directory. Alternatively type the following commands in the terminal. 
 ```bash 
 $ cd ~/ros_workspace
 $ git clone https://github.com/sendtooscar/ariaClientDriver.git
 ```

5. Navigate to the package directory and type make 
 ```bash 
 $ cd ariaClientDriver
 $ make
 ```
6. If the compiler complains about CMake Cache files,  navigate to the `ariaClient
Driver/build` directory and remove `CMakeCache.txt`. Aria libraries included in the zip file are 32-bit libraries, so depending on your system edit the library paths in `CMakeLists.txt` file to 64-bit library locations installed on your machine.

7. The current version of the driver provides raw laser data access and direct velocity motion commands.  For this to work, the robot or the simulating machine should compile and run `serverDemo3.cpp` modified server. The `serverDemo3.cpp` file is located in  `/ariaClientDriver/Aria/ArNetworking/examples/`  directory. Copy this to `/usr/local/Aria/ArNetworking/examples/` directory. Open a terminal and type the following to compile the modified server.
 ```bash
$ cd /usr/local/Aria/ArNetworking/examples/
$ make serverDemo3
```

This completes the package installation. The test examples can be used to familiarize with basic functionality of the ariaClientDriver node.

## Topic List

|Type|Description|
|------|------|
|odom 			|Type: ``nav_msgs/Odometry``. Publishes odometry reading of the robot.|
|scan			|Type: ``sensor_msgs/LaserScan``. Publishes raw laser readings of the robot. (Configured to 1 degree resolution and 180 degree field of view. ToDo: Set these using ROS parameters)|
|cmd_vel_ratio 		|Type: ``geometry_msgs/Twist``. This is the default topic used for robot motion control. The robot will assume the set translational and rotation speeds while avoiding obstacles. The speeds are accepted as a ratio of the set maximum velocity. (ToDo: ROS parameters to set maximum velocity)|
|cmd_vel 		|Type: ``geometry_msgs/Twist``. This topic is used to send direct motion commands. I.e., set the translational and rotation speeds in metric units without obstacle avoidance behaviours.|
|direct_enable 		|Type: `std_msgs/Bool`. This topic is used to enable or disable direct motion commands. Publish `true` to enable and latch direct motion commands.|	
|**Deprecated topics:**||
|scan2 		         |Publish laser scans as a ROS point cloud message|
|AriaCmdVel 		|Custom topic used to control the robot|
|AriaNavData 		|Custom message type: ``ariaClientDriver/AriaNavData``. Publishes all robot data. (ToDo: ROS parameter to enable this topic)|


##Examples

###A. Single robot SLAM
This example shows how to use the ROS navigation stack with the driver node. 

![alt text](https://cloud.githubusercontent.com/assets/10774875/9443583/5f516458-4a5c-11e5-999a-0b680f8898e3.png "Example1")


1. Initiate one robot in the Mobile Sim simulator: 
 ```bash
  $ MobileSim -m /usr/local/Aria/maps/columbia.map -r p3at
 ```
2. Run the server on the robot: 
 ```bash
 $ /usr/local/Aria/ArNetworking/examples/./serverDemo3 -connectlaser -rh localhost -rrtp 8101 -ris -sp 7272 
 ```

3. Run the ROS driver client, gmapping for SLAM, and RVIZ for visualization using the following launch file: 
 ```bash
 $ roslaunch ariaClientDriver test1_1robotslam.launch 
 ```
4. Go to terminal and use arrow keys to move the robot OR use the teleop panel to move the robot and build a map of the environment.  In the terminal press 'u' to disable obstacle avoidance if needed.

5. Once complete, run the following commands in a new terminal to save the map. 
 ```bash
 $ roscd ariaClientDriver
 $ rosrun map_server map_saver -f maps/columbia
 ```
 
###B. Multi-Robot Localization 

![alt text](https://cloud.githubusercontent.com/assets/10774875/9443703/26159f28-4a5d-11e5-9ed9-58499819cd3d.png "Example2")

1. Close all programs and initialize two robots in the simulator.
 ```bash
 $ MobileSim -m /usr/local/Aria/maps/columbia.map -r p3at -r p3at
 ```
 The port corresponding to the robot is indicated in the terminal once the simulator is initiated.


2. Run servers on each robot using two terminals:
 ```bash
 $ /usr/local/Aria/ArNetworking/examples/./serverDemo3 -connectlaser -rh localhost -rrtp 8101 -ris -sp 7272
 $ /usr/local/Aria/ArNetworking/examples/./serverDemo3 -connectlaser -rh localhost -rrtp 8102 -ris -sp 7273
 ```
 The port used to connect to the simulated robot is specified using the -rrtp option flag, and the port used to initiate the  robot server is specified using the -sp option flag.


3. Run the ROS driver client, gmapping for SLAM, and RVIZ for visualization for both robots using the following launch file: 
 ```bash
 $ roslaunch ariaClientDriver test2_2robotamcl.launch 
 ```
 Both robots can be moved togeher using arrow keys in the terminal window OR you can break the launch file to two launch files one for each robot to realize separate control in seperate terminals. Alternatively, you can use the teleop panels to move the robots. The particle distribution should converge to the correct location of the map as the robots collect scans of the environment. Also you can uncomment the move base node in the launch file which allows to set waypoints for each robot. The move base node allows the robot to find the optimum path to maneuver through the enviroment.

###C. Running on actual robots
This example explains the procedure to realize the above functionality using actual robots.

![alt text](https://cloud.githubusercontent.com/assets/10774875/9443710/3444cb0a-4a5d-11e5-9cfe-0da3ab03a3a3.png "Example3")

1. Connect all robots to the same Wifi network and identify the IP addresses of each robot.

2. Telnet or ssh each robot and copy the `serverDemo3.cpp` file to `/usr/local/Arnl/examples` folder of the robot's computer. 

3. Compile the file using the command:
 ```bash
  $ sudo make serverDemo3
 ```
 
4. Do the same for both robots and run the server on each robot.
 ```bash
 $ sudo ./serverDemo3 -connectlaser 
 ```

5. On your local machine run  example 1  and example 2 using the same procedure explained above. The only change is that you should use the robots ip address. The launch files `test3_1robotslam.launch` and`test3_2robotamcl.launch` can be used as reference to implement the examples using actual robots.
