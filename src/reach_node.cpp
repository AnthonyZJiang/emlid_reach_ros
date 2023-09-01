#include <ros/ros.h>

#include <nmea_msgs/Sentence.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgsa.h>
#include <nmea_msgs/Gpgst.h>
#include <nmea_msgs/Gpgsv.h>
#include <nmea_msgs/GpgsvSatellite.h>
#include <nmea_msgs/Gprmc.h>

#include "nmea/nmea_parser.h"
#include "nmea/nmea_sentence.h"
#include "reach_driver/reach_driver.h"


using namespace reach_driver;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "reach_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");


    std::string commType;
    float polling_rate;
    ros::param::param<std::string>("comm_type", commType, "serial");
    ros::param::param<float>("polling_rate", polling_rate, 1);

    bool notPolling = true;

    if (commType == "serial")
    {
        ReachSerialDriver serial_driver(node, private_nh);
        while (ros::ok() && serial_driver.ok())
        {
            bool polled = serial_driver.poll();
            if (!polled)
            {
                ROS_WARN_THROTTLE(1.0, "[REACH] Failed to poll device. Waiting for data...");
                notPolling = true;
            }
            else if (notPolling)
            {
                ROS_INFO_STREAM("[REACH] Polling successful. Reach is now streaming data.");
                notPolling = false;
            }
            ros::Duration(1.0/polling_rate).sleep();
            ros::spinOnce();
        }
    }
}