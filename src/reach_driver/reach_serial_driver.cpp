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

#include <ros/console.h>

#include "reach_driver/reach_driver.h"

using namespace reach_driver;

ReachSerialDriver::ReachSerialDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
    : initialised(false), ReachDriver(node, private_nh)
{
    private_nh.param<std::string>("port", port, "/dev/ttyACM0");
    private_nh.param<int>("baudrate", baudrate, 115200);
    private_nh.param<int>("timeout", timeout, 1000);
    ROS_INFO_STREAM("[REACH] Connecting to Reach...");
    ROS_INFO_STREAM("[REACH] Port: " << port << " | Baudrate: " << baudrate << " | Timeout: " << timeout);
    initialise();
}

ReachSerialDriver::~ReachSerialDriver()
{
    ser.close();
}

bool ReachSerialDriver::ok()
{
    return initialised && ser.isOpen();
}

void ReachSerialDriver::initialise()
{
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
    ser.setTimeout(to);

    if (ser.isOpen())
    {
        ROS_WARN_STREAM("[REACH] Port \"" << port << "\" is already open.");
        return;
    }
    while (ros::ok() && !ser.isOpen())
    {
        try
        {
            ser.open();
        }
        catch (serial::SerialException &e)
        {
            ROS_ERROR_STREAM("[REACH] Unable to open " << port << ". Error: \n"
                                                              << e.what());
            ros::Duration(1.0).sleep();
        }
        catch (serial::IOException &e)
        {
            if (e.getErrorNumber() == 13)
            {
                ROS_WARN_STREAM("[REACH] Do not have read access for " << port << ".");
            }
            else if (e.getErrorNumber() == 2)
            {
                ROS_WARN_STREAM("[REACH] Waiting for port \"" << port << "\" to appear. Reconnecting...");
            }
            else 
            {
                ROS_WARN_STREAM("[REACH] Can't open " << port << ".\n" 
                                                             << e.what());
            }
            ros::Duration(1.0).sleep();
        }
    }
    initialised = ser.isOpen();
    if (ok())
    {
        ROS_INFO_STREAM("[REACH] Reach connected!");
    }
}

void ReachSerialDriver::disconnect()
{
    ser.close();
    initialised = false;
}

void ReachSerialDriver::reconnect()
{
    disconnect();
    initialise();
}

string ReachSerialDriver::readFromDevice()
{
    try
    {
        return ser.read(ser.available());
    }
    catch (serial::PortNotOpenedException &e)
    {
        ROS_ERROR_STREAM("[REACH] Port \"" << port << "\" not open. Reconnecting...");
        reconnect();
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("[REACH] Serial Exception. Error: \n"
                         << e.what() << ".\nReconnecting...");
        reconnect();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[REACH] Unable to read from port. Error: \n"
                         << e.what() << ".\nReconnecting...");
        reconnect();
    }
}

bool ReachSerialDriver::available()
{
    try
    {
        return ser.available();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[REACH] Unable to read from port. Error: \n"
                         << e.what() << ".\nReconnecting...");
        reconnect();
    }
}
