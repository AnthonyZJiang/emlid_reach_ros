#include <ros/console.h>

#include "reach_driver/reach_driver.h"

using namespace reach_driver;

ReachSerialDriver::ReachSerialDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
:initialised(false), ReachDriver(node, private_nh)
{
    private_nh.param<std::string>("port", port, "/dev/ttyACM0");
    private_nh.param<int>("baudrate", baudrate, 115200);
    private_nh.param<int>("timeout", timeout, 1000);
    ROS_INFO_STREAM("[REACH] Connecting to Reach via port: " << port << ", baudrate: " << baudrate << ", with timeout: " << timeout);
    initialise();
    if (ok())
    {
        ROS_INFO_STREAM("[REACH] Reach connected.");
    }
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
    try
    {
        ser.open();
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("[REACH] Unable to open port \"" << port << "\". Reconnecting...");
        reconnect(5000);
    }
    
    initialised = ser.isOpen();
}

void ReachSerialDriver::disconnect()
{
    ser.close();
    initialised = false;
}

void ReachSerialDriver::reconnect(int interval)
{
    disconnect();
    if (interval > 0) {
        ros::Duration(interval).sleep();
    }
    initialise();
}

string ReachSerialDriver::readFromDevice()
{
    try{
        return ser.read(ser.available());
    }
    catch (serial::PortNotOpenedException &e)
    {
        ROS_ERROR_STREAM("[REACH] Port \"" << port << "\" not open. Reconnecting...");
        reconnect(0);
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("[REACH] Serial Exception. Error: \"" << e.what() << "\". Reconnecting...");
        reconnect();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[REACH] Unable to read from port. Error: \"" << e.what() << "\". Reconnecting...");
        reconnect();
        
    }
}

bool ReachSerialDriver::available()
{
    return ser.available();
}