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


### Giving drones letter commands

Wait till all the drones have took off the ground.

Run ./letter_publisher.py on another shell. Then write a letter or a word you want drones to showcase. Write exit to exit from letter publisher.

Alternatively publish the target character to `/char`: `ros2 topic pub /char std_msgs/msg/String "{data: 'X'}"`

or word `ros2 topic pub /char std_msgs/msg/String "{data: 'Fly'}"`

### Fixes for problems
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

# Findings


