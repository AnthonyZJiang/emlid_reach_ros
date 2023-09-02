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
            ROS_ERROR_STREAM("[REACH] Unable to open port \"" << port << "\". Error: \n"
                                                              << e.what());
            ros::Duration(1.0).sleep();
        }
        catch (serial::IOException &e)
        {
            ROS_WARN_STREAM("[REACH] Waiting for port \"" << port << "\" to appear. Reconnecting...");
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
