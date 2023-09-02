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
