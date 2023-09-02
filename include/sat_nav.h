#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>

#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgst.h>
#include <nmea_msgs/Gprmc.h>
#include <nmea_msgs/Gpzda.h>
#include <nmea_msgs/Gpvtg.h>

using namespace std;

namespace sat_nav
{
    class SatNav
    {
    private:
        float speed = 0;
        double track = 0;
        bool isSpeedTrackSet = false;

        double latitude = 0;
        double longitude = 0;
        float altitude = 0;
        bool isLatLonAltSet = false;

        float hdop = 0;
        bool isHdopSet = false;

        float latDev = 0;
        float lonDev = 0;
        float altDev = 0;
        bool isDevSet = false;

        uint32_t gpsQual = 0;
        bool isGpsQualSet = false;

        std::string date = "";
        double utcSeconds = 0;
        bool isUtcSecondsSet = false;
        uint16_t year = 0;
        uint8_t month = 0;
        uint8_t day = 0;

    public:
        SatNav();
        ~SatNav();

        void addData(nmea_msgs::Gpgga &gga);
        void addData(nmea_msgs::Gpgst &gst);
        void addData(nmea_msgs::Gprmc &rmc);
        void addData(nmea_msgs::Gpzda &zda);
        void addData(nmea_msgs::Gpvtg &vtg);

        void setTwist(geometry_msgs::Twist &twist);
        void setNavSatFix(sensor_msgs::NavSatFix &fix);
        void setTimeReference(sensor_msgs::TimeReference &tref);
    };
}
