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

#include <cmath>

#include "sat_nav.h"
#include "nmea/conversion.h"

using namespace sat_nav;

SatNav::SatNav()
{
}
SatNav::~SatNav()
{
}

/*
 * GPGGA - lat, lon, alt, gps_qual, hdop, utc_seconds
 * GPRMC - lat, lon, speed, track, date, utc_seconds, position_status
 * GPGST - lat_dev, lon_dev, alt_dev, utc_seconds
 * GPZDA - utc_seconds, day, month, year
 * GPVTG - speed_k, track_t
 *
 * Twist
 *  linear      : available from GPVTG or GPRMC.
 *
 * NavSatFix:
 *  status      : available from GPGGA.gps_qual or GPRMC.position_status. GGA provides more details.
 *  latitude    : available from GPGGA.lat or GPRMC.lat.
 *  longitude   : available from GPGGA.lon or GPRMC.lon.
 *  altitude    : only available from GPGGA.alt.
 *  covariance  : need both GPGGA.hdop and GPGST.xxx_dev to compute.
 *
 * TimeReference:
 *  time_ref    : utc_seconds available from GPGGA, GPRMC, GPGST and GPZDA. GPZDA does not provide ms.
 */

void SatNav::addData(nmea_msgs::Gpgga &gga)
{
    if (utcSecondsSetState != SET_STATE_OPTIMUM)
    {
        utcSeconds = gga.utc_seconds;
        utcSecondsSetState = SET_STATE_OPTIMUM;
    }
    if (latLonAltSetState != SET_STATE_OPTIMUM)
    {
        latitude = nmea::latLonToDeg(gga.lat, gga.lat_dir);
        longitude = nmea::latLonToDeg(gga.lon, gga.lon_dir);
        latLonAltSetState = SET_STATE_OPTIMUM;
    }
    altitude = gga.alt;

    gpsQual = gga.gps_qual;
    gpsQualSetState = SET_STATE_OPTIMUM;

    hdop = gga.hdop;
    hdopSetState = SET_STATE_OPTIMUM;
}

void SatNav::addData(nmea_msgs::Gpgst &gst)
{
    if (utcSecondsSetState != SET_STATE_OPTIMUM)
    {
        utcSeconds = gst.utc_seconds;
        utcSecondsSetState = SET_STATE_OPTIMUM;
    }
    latDev = gst.lat_dev;
    lonDev = gst.lon_dev;
    altDev = gst.alt_dev;
    devSetState = SET_STATE_OPTIMUM;
}

void SatNav::addData(nmea_msgs::Gprmc &rmc)
{
    if (utcSecondsSetState != SET_STATE_OPTIMUM)
    {
        utcSeconds = rmc.utc_seconds;
        utcSecondsSetState = SET_STATE_OPTIMUM;
    }
    if (latLonAltSetState != SET_STATE_OPTIMUM)
    {
        latitude = nmea::latLonToDeg(rmc.lat, rmc.lat_dir);
        longitude = nmea::latLonToDeg(rmc.lon, rmc.lon_dir);
        latLonAltSetState = SET_STATE_OPTIMUM;
    }
    if (speedTrackSetState != SET_STATE_OPTIMUM)
    {
        speed = nmea::knotsToKilometersPerHour(rmc.speed);
        track = rmc.track;
        speedTrackSetState = SET_STATE_OPTIMUM;
    }
    if (gpsQualSetState == SET_STATE_NONE)
    {
        gpsQual = rmc.position_status == "A" ? 1 : 0;
        gpsQualSetState = SET_STATE_NORMAL;
    }
    date = rmc.date;
}

void SatNav::addData(nmea_msgs::Gpzda &zda)
{
    if (utcSecondsSetState == SET_STATE_NONE)
    {
        utcSeconds = zda.utc_seconds;
        utcSecondsSetState = SET_STATE_NORMAL;
    }
    day = zda.day;
    month = zda.month;
    year = zda.year;
}

void SatNav::addData(nmea_msgs::Gpvtg &vtg)
{
    if (speedTrackSetState != SET_STATE_OPTIMUM)
    {
        speed = vtg.speed_k;
        track = vtg.track_t;
        speedTrackSetState = SET_STATE_OPTIMUM;
    }
}

bool SatNav::setTwist(geometry_msgs::Twist &twist)
{
    if (speedTrackSetState == SET_STATE_NONE)
    {
        return false;
    }

    twist.linear.x = speed * sin(track);
    twist.linear.y = speed * cos(track);
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    return true;
}

bool SatNav::setNavSatFix(sensor_msgs::NavSatFix &fix)
{
    if (latLonAltSetState == SET_STATE_NONE)
    {
        return false;
    }
    sensor_msgs::NavSatStatus status;
    switch (gpsQual)
    {
    case 1:
        status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
    case 9:
    case 2:
        status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        break;
    case 4:
    case 5:
        status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        break;
    default:
        status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
    }
    status.service = 1;

    fix.status = status;
    fix.latitude = latitude;
    fix.longitude = longitude;
    fix.altitude = altitude;

    if (devSetState == SET_STATE_NONE || hdopSetState == SET_STATE_NONE)
    {
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
    else
    {
        fix.position_covariance[0] = pow(hdop * latDev, 2);
        fix.position_covariance[4] = pow(hdop * lonDev, 2);
        fix.position_covariance[8] = pow(hdop * altDev, 2);
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    }
    return true;
}

bool SatNav::setTimeReference(sensor_msgs::TimeReference &tref)
{
    if (year > 0)
    {
        double unixTs = nmea::toUnixTimestamp(year, month, day, utcSeconds);
        tref.time_ref = ros::Time(unixTs);
    }
    else if (!date.empty())
    {
        double unixTs = nmea::toUnixTimestamp(date, utcSeconds);
        tref.time_ref = ros::Time(unixTs);
    }
    else
    {
        return false;
    }
    tref.source = "gps";
    return true;
}
