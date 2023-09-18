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

#include <sstream>
#include <cmath>
#include <ros/console.h>

#include "reach_driver/reach_driver.h"
#include "nmea/nmea_sentence.h"
#include "nmea/conversion.h"

using namespace reach_driver;
using namespace nmea;

ReachDriver::ReachDriver(ros::NodeHandle node,
                         ros::NodeHandle private_nh)
{
    bool parser_debug;
    private_nh.param<bool>("parser_debug", parser_debug, false);
    parser.log = parser_debug;
    ROS_INFO_STREAM("[REACH] NMEA Parser debug: " << (parser_debug ? "on" : "off"));
    private_nh.param<string>("frame_id", frame_id, "gps");

    setSentencePubs(private_nh, node);

    twist_pub = node.advertise<geometry_msgs::Twist>("reach/vel", 100);
    fix_pub = node.advertise<sensor_msgs::NavSatFix>("reach/fix", 100);
    timeref_pub = node.advertise<sensor_msgs::TimeReference>("reach/time_ref", 100);
}

ReachDriver::~ReachDriver()
{
}

void ReachDriver::setSentencePubs(ros::NodeHandle private_nh, ros::NodeHandle node)
{
    ROS_INFO_STREAM("[REACH] Publish ignored sentences: " << (publish_ignored ? "on" : "off"));
    private_nh.param<bool>("pub_ignored", publish_ignored, true);
    if (publish_ignored)
    {
        sentence_pub = node.advertise<nmea_msgs::Sentence>("reach/nmea/ignored_sentence", 100);
    }

    std::string sentences;
    private_nh.param<std::string>("sentences", sentences, "GGA,GSA,GST,GSV,RMC,VTG,ZDA");
    istringstream f(sentences);
    string s;
    std::stringstream ss;
    while (getline(f, s, ','))
    {
        if (s == "GGA")
        {
            publish_gpgga = true;
            gpgga_pub = node.advertise<nmea_msgs::Gpgga>("reach/nmea/gpgga", 100);
            ss << s << ",";
        }
        else if (s == "GSA")
        {
            publish_gpgsa = true;
            gpgsa_pub = node.advertise<nmea_msgs::Gpgsa>("reach/nmea/gpgsa", 100);
            ss << s << ",";
        }
        else if (s == "GST")
        {
            publish_gpgst = true;
            gpgst_pub = node.advertise<nmea_msgs::Gpgst>("reach/nmea/gpgst", 100);
            ss << s << ",";
        }
        else if (s == "GSV")
        {
            publish_gpgsv = true;
            gpgsv_pub = node.advertise<nmea_msgs::Gpgsv>("reach/nmea/gpgsv", 100);
            ss << s << ",";
        }
        else if (s == "RMC")
        {
            publish_gprmc = true;
            gprmc_pub = node.advertise<nmea_msgs::Gprmc>("reach/nmea/gprmc", 100);
            ss << s << ",";
        }
        else if (s == "VTG")
        {
            publish_gpvtg = true;
            gpvtg_pub = node.advertise<nmea_msgs::Gpvtg>("reach/nmea/gpvtg", 100);
            ss << s << ",";
        }
        else if (s == "ZDA")
        {
            publish_gpzda = true;
            gpzda_pub = node.advertise<nmea_msgs::Gpzda>("reach/nmea/gpzda", 100);
            ss << s << ",";
        }
        else
        {
            ROS_WARN_STREAM("Unknown sentence type \"" << s << "\". Ignoring.");
        }
    }
    std::string sss = ss.str();
    ROS_INFO_STREAM("[REACH] Sentences to publish: " << (sss.empty() ? "none" : sss.substr(0, sss.size() - 1)));
}

bool ReachDriver::available() {}

string ReachDriver::readFromDevice() {}

bool ReachDriver::poll()
{
    if (available())
    {
        geometry_msgs::Twist twist;
        sensor_msgs::NavSatFix fix;
        sensor_msgs::NavSatStatus status;
        sensor_msgs::TimeReference timeref;
        sat_nav::SatNav satNav;

        ros::Time now = ros::Time::now();
        std::string text = readFromDevice();
        std::vector<NMEASentence> sentences = parser.getSentencesFromRawText(text);
        for (size_t i = 0; i < sentences.size(); i++)
        {
            if (!sentences[i].valid())
            {
                ROS_INFO_STREAM("[REACH] Invalid sentence: " + sentences[i].text);
                continue;
            }
            if (sentences[i].name == "GGA" && publish_gpgga)
            {
                nmea_msgs::Gpgga gpgga;
                gpgga.header.stamp = now;
                gpgga.header.frame_id = frame_id;
                parser.parseParameters(gpgga, sentences[i]);
                gpgga_pub.publish(gpgga);

                satNav.addData(gpgga);
            }
            else if (sentences[i].name == "GSA" && publish_gpgsa)
            {
                nmea_msgs::Gpgsa gpgsa;
                gpgsa.header.stamp = now;
                gpgsa.header.frame_id = frame_id;
                parser.parseParameters(gpgsa, sentences[i]);
                gpgsa_pub.publish(gpgsa);
            }
            else if (sentences[i].name == "GST" && publish_gpgst)
            {
                nmea_msgs::Gpgst gpgst;
                gpgst.header.stamp = now;
                gpgst.header.frame_id = frame_id;
                parser.parseParameters(gpgst, sentences[i]);
                gpgst_pub.publish(gpgst);
                satNav.addData(gpgst);
            }
            else if (sentences[i].name == "GSV" && publish_gpgsv)
            {
                nmea_msgs::Gpgsv gpgsv;
                gpgsv.header.stamp = now;
                gpgsv.header.frame_id = frame_id;
                parser.parseParameters(gpgsv, sentences[i]);
                gpgsv_pub.publish(gpgsv);
            }
            else if (sentences[i].name == "RMC" && publish_gprmc)
            {
                nmea_msgs::Gprmc gprmc;
                gprmc.header.stamp = now;
                gprmc.header.frame_id = frame_id;
                parser.parseParameters(gprmc, sentences[i]);
                gprmc_pub.publish(gprmc);
                satNav.addData(gprmc);
            }
            else if (sentences[i].name == "VTG" && publish_gpvtg)
            {
                nmea_msgs::Gpvtg gpvtg;
                gpvtg.header.stamp = now;
                gpvtg.header.frame_id = frame_id;
                parser.parseParameters(gpvtg, sentences[i]);
                gpvtg_pub.publish(gpvtg);
                satNav.addData(gpvtg);
            }
            else if (sentences[i].name == "ZDA" && publish_gpzda)
            {
                nmea_msgs::Gpzda gpzda;
                gpzda.header.stamp = now;
                gpzda.header.frame_id = frame_id;
                parser.parseParameters(gpzda, sentences[i]);
                gpzda_pub.publish(gpzda);
                satNav.addData(gpzda);
            }
            else if (publish_ignored)
            {
                nmea_msgs::Sentence sentence;
                sentence.header.stamp = now;
                sentence.header.frame_id = frame_id;
                parser.parseParameters(sentence, sentences[i]);
                sentence_pub.publish(sentence);
            }
        }

        if (satNav.setTwist(twist))
        {
            twist_pub.publish(twist);
        }
        if (satNav.setNavSatFix(fix))
        {
            fix.header.stamp = now;
            fix.header.frame_id = frame_id;
            fix_pub.publish(fix);
        }
        if (satNav.setTimeReference(timeref))
        {
            timeref.header.stamp = now;
            timeref.header.frame_id = frame_id;
            timeref_pub.publish(timeref);
        }
        
        return true;
    }
    return false;
}
