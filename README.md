# emlid_reach_ros
ROS package supporting [EMLID](https://emlid.com/) Reach series GPS devices.

It interfaces with EMLID Reach devices, parses received NMEA sentences and publishes topics including NMEA messages, NavSatFix, TimeReference and Twist.

*Disclaimer: This is not an official ROS package and the author of this ROS package is not affiliated with EMLID.*

# Install

### Dependancy
```
sudo apt update
sudo apt install ros-${ROS_DISTRO}-serial
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

# reach_node
### Published Topics

```
reach/nmea/gpgga (nmea_msgs/Gpgga.h)
    Translated from NMEA GGA sentences which contains position fix data.

reach/nmea/gpgsa (nmea_msgs/Gpgsa.h)
    Translated from NMEA GSA sentences which contains DOP and active satellite data.

reach/nmea/gpgst (nmea_msgs/Gpgst.h)
    Translated from NMEA GST sentences which contains GPS Pseudorange Noise Statistics.

reach/nmea/gpgsv (nmea_msgs/Gpgsv.h)
    Translated from NMEA GSV sentences which contains satellite data.

reach/nmea/gprmc (nmea_msgs/Gprmc.h)
    Translated from NMEA RMC sentences which contains recommended minimum specific GPS/Transit data.

reach/nmea/gpvtg (nmea_msgs/Gpvtg.h)
    Translated from NMEA VTG sentences which contains track made good and ground speed.

reach/nmea/gpzda (nmea_msgs/Gpzda.h)
    Translated from NMEA ZDA sentences which contains date and time.

reach/nmea/ignored_sentences (nmea_msgs/Sentence.h)
    Raw NMEA sentences that are not parsed by the driver.

reach/fix (sensor_msgs/NavSatFix)
    GPS postion fix in Lat/Lon/Alt. Requires GGA or RMC sentences to be parsed.
        Note that altitude is only reported by GGA.
    GPS fix quality is also reported in this topic.
        RMC only reports fix or no fix
        GGA also reports RTK fixes. 
    Covariance is calculated if GPGST sentences are also parsed.

reach/vel (geometry_msgs/TwistStamped)
    Linear velocity calculated from GPS data. Requires RMC or VTG sentences to be parsed.

reach/time_ref (sensor_msgs/TimeReference)
    The timestamp from the GPS device. Requires ZDA or RMC sentences to be parsed.
```

### Parameters
```
comm_type (string, default: serial)
    The communication type. Currently only serial is supported. TCP/IP support is planned.

port (string, default: /dev/ttyACM0)
     The device patha.

baudrate (int, default: 115200)
    The baud rate to receive NMEA data.

timeout (int, default: 1000)
    The timeout in milliseconds to wait for NMEA data.

polling_rate (int, default: 1)
    The polling rate in Hz. This value should be equal or less than the device's output rate.

sentences (string, default: GGA,GST,ZDA,VTG)
    The NMEA sentences to be parsed. The driver will only parse the sentences specified here.
    The sentences should be separated by comma.

pub_ignored (bool, default: true)
    Toggles publishing of raw NMEA sentences that are not parsed by the driver.

parser_debug (bool, default: false)
    Toggles debug output from the NMEA parser.
```
