
# Overview
This repository contains the work done during the TTK4551-Specialization project. Here we aim to control a Quadcopter drone using both the functionallity from the flight control and mission planing software Qgroundcontrol, using the MAVLink protocol. Aswell as using the offboar control mode where we can control the PX4 flightstack uning software outside of the autopilot for autonomus flight. Using the Offboard control is very dangerous due to the risk of bugs/errors in the offboard control script. Therefore this project aims to create a safe simulation environment to simulate control scripts on a drone using PX4 autopilot in mind, before protentialy implementing it into a companion computer on a real drone. 

This repo is a derivative of Braden Wagstaff's Offboard example
https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example

I have taken form his example and added some more functionality such as:
* connection between Qgroundcontrol 
* Mission control terminal to launch offboard mode, while giving status updates
* How to log data from flight and delivering data 


## YouTube Tutorial
Braden Wagstaff published a walkthrough tutorial on YouTube to demonstrate the example and to help beginners set up their enviornment. The video was super helpful, but be always sure to defer to this Readme file for instructions. Some changes have been made since the video was posted, meaning that even though it is helpful, it is not 100% accurate.
You can watch the video [here](https://www.youtube.com/watch?v=8gKIP0OqHdQ).

The tutorial explains at a basic level how to use ROS2 and PX4 in order to control a simulated UAV's velocity with keyboard controls. The goal is to create a simple example that a complete beginner can follow and understand, even with no ROS2 or PX4 experience.


### ROS2_PX4_Offboard_Example
The content of this tutorial was migrated to Custom Modes ad written in C++, you can find it below:
[ros2_px4_teleop_example 🎛️](https://github.com/ARK-Electronics/ros2_px4_teleop_example/tree/main)

## Prerequisites to use the simulator
* ROS2 Humble
* Gazebo Fortress
* QGroundControl
* PX4 Autopilot
* Micro XRCE-DDS Agent
* px4_msgs
* Ubuntu 22.04
* Python 3.10

### Installing ROS2 Humble on your computer
To install ROS2 Humble follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Installing Gazebo fortress on your computer
To install Gazebo fortress follow the steps [here](https://gazebosim.org/docs/fortress/install_ubuntu/)

### Installing QGroundControl on your computer
To install QGroundControll in ubuntu follow the steps here [here](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html)

## Setup Steps

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

You will now need to restart your computer before continuing.




### Installing Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with the following commands

```
pip3 install --user -U pyros-genmsg
```

I also found that without these packages installed Gazebo has issues with loading

```
pip3 install --user kconfiglib jsonschema jinja2
```

### Build Micro DDS
As in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client) we will need to run this code in order to build MicroDDS on your machine

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Setup Workspace
This git repo is intended to be a ROS2 package that is cloned into a ROS2 workspace.

We're going to create a workspace in our home directory, and then clone in this repo and also the px4_msgs repo. 

For more information on creating workspaces, see [here](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html)

Run this code to create a workspace in your home directory

```
mkdir -p ~/ros2_px4_offboard_ws/src
cd ~/ros2_px4_offboard_ws/src
```

*ros2_px4_offboard_ws* is just a name I chose for the workspace. You can name it whatever you want. But after we run *colcon build* you might have issues changing your workspace name so choose wisely.

We are now in the src directory of our workspace. This is where ROS2 packages go, and is where we will clone in our two repos.

### Clone in Packages
We first will need the px4_msgs package. Our ROS2 nodes will rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/main/en/ros/ros2_comm.html#overview:~:text=ROS%202%20applications,different%20PX4%20releases) for more information.

Be sure you're in the src directory of your workspace and then run this code to clone in the px4_msgs repo

```
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
```

Once again be sure you are still in the src directory of your workspace. Run this code to clone in our example package

```
git clone https://github.com/ErikFougnerArnesen/TK4551---Eng-Cybernetics-Specialization-Project
```

Run this code to clone the repo



### Building the Workspace
The two packages in this workspace are px4_msgs and px4_offboard. px4_offboard is a ROS2 package that contains the code for the offboard control node that we will implement. It lives inside the ROS2_PX4_Offboard_Example directory.

Before we build these two packages, we need to source our ROS2 installation. Run this code to do that

```
source /opt/ros/humble/setup.bash
```

This must be run every terminal that wants to run ROS2 commands. To get around this, add it to your .bashrc file. Then the command every time you open a new terminal window.

To build these two packages, you must be in workspace directory not in src, run this code to change directory from src to one step back i.e. root of your workspace and build the packages

```
cd ..
colcon build
```
As mentioned in Jaeyoung Lim's [example](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md) you will get some warnings about setup.py but as long as there are no errors, you should be good to go.


After this runs, we should never need to build px4_msgs again. However, we will need to build px4_offboard every time we make changes to the code. To do this, and save time, we can run
```
colcon build --packages-select px4_offboard
```

If you tried to run our code now, it would not work. This is because we need to source our current workspace. This is always done after a build. To do this, be sure you are in the src directory, and then run this code

```
source install/setup.bash
```

We will run this every time we build. It will also need to be run in every terminal that we want to run ROS2 commands in.


### Running Drone with keyboard commands
This example has been designed to run from one launch file that will start all the necessary nodes. The launch file will run a python script that uses gnome terminal to open a new terminal window for MicroDDS and Gazebo.

Run this code to start it

```
ros2 launch px4_offboard offboard_velocity_control.launch.py
```
Once everything is running, you should be able to focus into the control.py terminal window, arm, and takeoff. The controls mimic Mode 2 RC Transmitter controls with WASD being the left joystick and the arrow keys being the right joystick.
* W: Up
* S: Down
* A: Yaw Left
* D: Yaw Right
* Up Arrow: Pitch Forward
* Down Arrow: Pitch Backward
* Left Arrow: Roll Left
* Right Arrow: Roll Right
* Space: Arm/Disarm

Pressing *Space* will arm the drone. Wait a moment and it will takeoff and switch into offboard mode. You can now control it using the keyboard keys. landing the drone, it will disarm and to takeoff again you will need to toggle the arm switch off and back on with the space bar. 


### Runnin the waypoint mission Developed
This will launch the waypoint mission developed during the projet.
```
ros2 launch px4_offboard waypoint_mission.launch.py
```
Once everything is up and running, press space. Wait a moment and the drone will takeoff -> increase altitude to 100m -> Travel to the predefined latitude and longitude -> hover for 5 seconds -> Return to home location -> Land -> Mission complete

Watch the Mission_control_terminal, it will show a status update of the drone during the mission. It will further provide information such as Altitude and distance from target coordinates.  

## Closing Simulation *IMPORTANT*
When closing the simulation, it is very very important to click *Ctrl+C* on all terminals. THis will close Gazebo and all it's child processes. 
If you just close the terminal windows, it will cause to leaving gazebo and child processes to run in the background.
This can cause issues when you want to run Gazebo in the future     

## Connecting Qgroundcontrol to simulator
To connect the ground controll software to the simulator, you will need to go in Qgroundcontrol into:
Application Settings -> Comm Links

Here make sure to check the UDP checkbox. After that Press 'Add new link' under Links.

Give it then a name you can remember, such as PX4 SITL and give the type of UDP on port 14550. It will then connect automatically to the simulator when you launch. 

## Logging simulator flights
To log simulator flights got to Application setting again and enter PX4 Log Transfere. You will then need to check 'Enable automatic loggin', then enter you email address you want to recive the link to the log files from PX4 Log review site. I recommend to make the logs publick so you can share them og add them to your rapports. I would also recommend to check 'Enable automatic log uploads', so you do not need to do it manually     


## Known Issues
If the vehicle does not arm check to ensure the parameter NAV_DLL_ACT is set to 0.
