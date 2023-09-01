# emlid_reach_ros
ROS package supporting [EMLID](https://emlid.com/) Reach series GPS devices.

*Disclaimer: This is not an official ROS package and the author of this ROS package is not affiliated with EMLID.*

# Dependancies
- ROS melodic
- serial

# Install

### Dependancy
```
sudo apt update
sudo apt install ros-melodic-serial
```

### Build from source
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/tmxkn1/emlid_reach_ros.git
git clone https://github.com/tmxkn1/nmea_msgs.git
cd ..
catkin_make
```

# Configure Reach GPS device
- Complete the initial device setup using either the mobile phone App or the web interface.
- Log into the device.
- Select Position streaming 1 or 2.
- Change "Channel" to "serial".

# Quick start
- Connect the device to your PC using a USB cable
- Find the device name, e.g. /dev/ttyACM0
```
roslaunch emlid_reach_ros reach_ros.launch port:=<your_device_name>
```

**Note** you may need to add read and write permission to the device:
```
sudo chmod +666 <your_device_name>
```