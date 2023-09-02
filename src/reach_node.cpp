/* BSD 3-Clause License

Copyright (c) 2023, Zhengyi Jiang, The University of Manchester, Ice Nine Robotics Solutions Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

int main(int argc, char *argv[])
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
            ros::Duration(1.0 / polling_rate).sleep();
            ros::spinOnce();
        }
    }
}
