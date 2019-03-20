# RoboND-Term1-P3-Where-Am-I
Project 3 of Udacity Robotics Software Engineer Nanodegree Program
![Overview](/videos/Term1-Project2-Go-Chase-It-Demo.gif)  
## Overview  
[TODO]
In this project you'll create two ROS packages inside your `catkin_ws/src`: the `drive_bot` and the `ball_chaser` which will be used in Gazebo for all your upcoming projects in the [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209). Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:  
1. `drive_bot`:  
* Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
* Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
* House your robot inside the world you built in the **Build My World** project.
* Add a white-colored ball to your Gazebo world and save a new copy of this world.
* The `world.launch` file should launch your world with the white-colored ball and your robot.
2. `ball_chaser`:
* Create a `ball_chaser` ROS package to hold your C++ nodes.
* Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
* Write a `process_image` C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
* The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.  
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  
## Project Description  
[TODO]
Directory Structure  
```
.Go-Chase-It                                   # Go Chase It Project
├── catkin_ws                                  # Catkin workspace
│   ├── src
│   │   ├── ball_chaser                        # ball_chaser package        
│   │   │   ├── launch                         # launch folder for launch files
│   │   │   │   ├── ball_chaser.launch
│   │   │   ├── src                            # source folder for C++ scripts
│   │   │   │   ├── drive_bot.cpp
│   │   │   │   ├── process_images.cpp
│   │   │   ├── srv                            # service folder for ROS services
│   │   │   │   ├── DriveToTarget.srv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_gokart                          # my_gokart package        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── gokart_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_gokart.gazebo
│   │   │   │   ├── my_gokart.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── myoffice.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_robot                           # my_robot package        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── meshes                         # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── myoffice.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
├── my_ball                                    # Model files 
│   ├── model.config
│   ├── model.sdf
├── videos                                     # Video files
│   ├── Term1-Project2-Go-Chase-It-Demo.gif    # Demo video
```
- [Term1-Project2-Go-Chase-It-Demo.gif](/videos/Term1-Project2-Go-Chase-It-Demo.gif): A demo video for successful run.  
- [drive_bot.cpp](/catkin_ws/src/ball_chaser/src/drive_bot.cpp): ROS service C++ script, command the robot with specify speeds.  
- [process_images.cpp](/catkin_ws/src/ball_chaser/src/process_images.cpp): ROS service C++ script, process the camera image and return requested speeds.  
- [gokart_description.launch](/catkin_ws/src/my_gokart/launch/gokart_description.launch): Create gokart model in Gazebo world.  
- [world.launch](/catkin_ws/src/my_gokart/launch/world.launch): Launch my_gokart mode in Gazebo world with building and plugins.  
- [my_gokart.gazebo](/catkin_ws/src/my_gokart/urdf/my_gokart.gazebo): Define my_gokart URDF model plugins.  
- [my_gokart.xacro](/catkin_ws/src/my_gokart/urdf/my_gokart.xacro): Define my_gokart URDF model.  
- [empty.world](/catkin_ws/src/my_gokart/worlds/empty.world): Gazebo world file that includes nothing.  
- [myoffice.world](/catkin_ws/src/my_gokart/worlds/myoffice.world): Gazebo world file that includes the models.  
- [CMakeLists.txt](/catkin_ws/src/my_gokart/CMakeLists.txt): File to link the C++ code to libraries.  
- [robot_description.launch](/catkin_ws/src/my_robot/launch/robot_description.launch): Create robot model in Gazebo world.  
- [hokuyo.dae](/catkin_ws/src/my_robot/meshes/hokuyo.dae): Hokuyo LiDAR sensor mesh model.  
- [my_robot.gazebo](/catkin_ws/src/my_robot/urdf/my_robot.gazebo): Define my_robot URDF model plugins.  
- [my_robot.xacro](/catkin_ws/src/my_robot/urdf/my_robot.xacro): Define my_robot URDF model.  

## Run the project  
* Clone this repository
```
git clone https://github.com/jinchaolu/RoboND-Term1-P3-Where-Am-I.git
```
* Open the repository and make  
```
cd /home/workspace/RoboND-Term1-P3-Where-Am-I/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* Launch amcl node  
```
roslaunch my_robot amcl.launch
```  
* Testing  
You have two options to control your robot while it localize itself here:  
- Send navigation goal via RViz  
- Send move command via teleop package.  
Navigate your robot, observe its performance and tune your parameters for AMCL.  

**Option 1: Send `2D Navigation Goal`**  
Your first option would be sending a `2D Nav Goal` from RViz. The `move_base` will try to navigate your robot based on the localization. Based on the new observation and the odometry, the robot to further perform the localization.  
Click the `2D Nav Goal` button in the toolbar, then click and drag on the map to send the goal to the robot. It will start moving and localize itself in the process. If you would like to give `amcl` node a nudge, you could give the robot an initial position estimate on the map using `2D Pose Estimate`.  
**Option 2: Use `teleop` Node**  
You could also use teleop node to control your robot and observe it localize itself in the environment.  
Open another terminal and launch the `teleop` script:  
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
You could control your robot by keyboard commands now.  

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. When you see this error:  
[request_publisher-2] process has died [pid 7531, exit code -6, cmd /home/nvidia/Documents/github/RoboND-Term1-P3-Where-Am-I/catkin_ws/devel/lib/pgm_map_creator/request_publisher (-30,30)(30,30)(30,-30)(-30,-30) 5 0.01 /home/nvidia/Documents/github/RoboND-Term1-P3-Where-Am-I/catkin_ws/src/pgm_map_creator/maps/map __name:=request_publisher __log:=/home/nvidia/.ros/log/21ee11ca-411f-11e9-9258-00044bc5f185/request_publisher-2.log].
log file: /home/nvidia/.ros/log/21ee11ca-411f-11e9-9258-00044bc5f185/request_publisher-2*.log

Please refer to this link to fix it,  
http://answers.gazebosim.org/question/8928/protobuf-error-for-custom-messages-transport-tutorial/  

Then catkin_make  
Then source  
3. You might need to generate the map again because of the size.  
4. Got an error when launching amcl.launch  
check the amcl.launch file that you have correctly mapped the topics to the correct published ones  
<remap to="scan" from="my_robot/laser/scan"/>  
Figure out amcl node is subscribing which topic? Then do the correct remapping.  

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubric  
### 1. Basic Requirements  
#### 1.1 Did the student submit all required files?  
Yes, he did.   
### 2. Simulation Setup
#### 2.1 Did the student set up the simulation environment properly?  
Yes, he did.  
#### 2.2 Is the student's simulation setup suitable for the localization task?  
Yes, it is.  
### 3. Localization Setup  
#### 3.1 Did the student correctly build the launch files for localization?  
Yes, he did.  
#### 3.2 Did the student properly set the parameters for localization?  
Yes, he did.  
### 4. Localization Performance  
#### 4.1 Is the student's robot able to localize itself?  
Yes, it is.  
