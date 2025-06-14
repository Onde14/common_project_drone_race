# Installation

Install ROS2 Galactic

```
https://docs.ros.org/ with the `ros-galactic-desktop` option.
```

Make sure you have gazebo
```
sudo apt install gazebo11 libgazebo11 libgazebo11-dev
```
Add the following
```
sudo apt install libasio-dev
sudo apt install ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers 
sudo apt install libignition-rendering3 
pip3 install transformations
```
Build this package
```
mkdir -p ~/tello_letter_project
cd ~/tello_letter_project
git clone https://github.com/Onde14/common_project_drone_race.git -b tuomas
cd common_project_drone_race
source /opt/ros/galactic/setup.bash
colcon build
```
Launching tello letter simulation
```
cd ~/tello_letter_project
source install/setup.bash
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh
ros2 launch open_project simple_launch.py
```


## Giving drones letter commands

Wait till all the drones have took off the ground.

Run ./letter_publisher.py on another shell. Then write a letter or a word you want drones to showcase. Write exit to exit from letter publisher.

Alternatively publish the target character to `/char`: `ros2 topic pub /char std_msgs/msg/String "{data: 'X'}"`

or word `ros2 topic pub /char std_msgs/msg/String "{data: 'Fly'}"`

## Changing amount of drones

If you want to change the amount of drones spawned you have change `NUM_DRONES` value in simple_launch.py, letter_server.py and tello_controller.py.

## Fixes for problems
If you run into the No namespace found error re-set GAZEBO_MODEL_PATH:
```
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh
```

If enough drones isn't spawning to gazebo

To spawn N drones add:  

`~/CycloneDDS/my-config.xml`:

```
<CycloneDDS>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
  </Discovery>
</CycloneDDS>
```

add `CYCLONEDDS_URI="file://$HOME/CycloneDDS/my-config.xml"` to `~/.bashrc`

# Design

Project consist of launcher file called simple_launch.py and three script files called letter_publisher.py, letter_server.py and tello_controller.py

## Simple_launch

Simple_launch calls other scripts in order to start and generate the gazebo world, drones in it and nodes related to them. 

## Letter_publisher

Letter_publisher takes inputted text from the user and publishes every character of the text one by one to the char topic.

## Letter_server

Letter_server holds LetterServer node class and classes Line and PartialCircle and methods for drawing and measuring positions for drones to take. The LetterServer class creates individual target topics for every drone to subscribe to, listens for incoming character request in char topic, calls get_character_points method to draw the points of the requested letter and publishes the individual target positions of every drone in the gazebo world.

## Tello_controller

Tello_controller holds the TelloController node class that listens for incoming target position publishes in the drone's target topic and navigates the drone in the navigate method to that target position. It does this by taking into consideration the drone's current x, y, z position, comparing them to the drone's target position x, y, z and calculating these positions difference as error which then can used to command the drone to move in right direction in these axis. In navigation drones constantly calculate their distance to other drones and make sure that any drones don't come too close to it by moving to opposite direction from them. This way drones are aware of each other and cooperate together to not cause collisions.

# Findings

We initially ran into some issues with spawning enough drones required to implement the system, solutions to the issues are described above. Once we could manage to spawn enough drones, the system turned out quite stable.

# Future extensions

Future extensions could be implementing the system on actual drones, like a droneshow. Localization of the drones would probably require some more work.

# AI usage

ChatGPT was used to generate code for letter_server.py and letter_publisher.py.
