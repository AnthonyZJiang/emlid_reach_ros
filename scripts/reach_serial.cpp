#include <ros/ros.h>
#include <serial/serial.h>

#include <nmea_msgs/Sentence.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgsa.h>
#include <nmea_msgs/Gpgst.h>
#include <nmea_msgs/Gpgsv.h>
#include <nmea_msgs/GpgsvSatellite.h>
#include <nmea_msgs/Gprmc.h>

#include <nmea/nmea_parser.h>
#include <nmea/nmea_sentence.h>

serial::Serial ser;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Publisher sentence_pub = nh.advertise<nmea_msgs::Sentence>("nmea/ignored_sentence", 100);
    ros::Publisher gpgga_pub = nh.advertise<nmea_msgs::Gpgga>("nmea/gpgga", 100);
    ros::Publisher gpgsa_pub = nh.advertise<nmea_msgs::Gpgsa>("nmea/gpgsa", 100);
    ros::Publisher gpgst_pub = nh.advertise<nmea_msgs::Gpgst>("nmea/gpgst", 100);
    ros::Publisher gpgsv_pub = nh.advertise<nmea_msgs::Gpgsv>("nmea/gpgsv", 100);
    ros::Publisher gprmc_pub = nh.advertise<nmea_msgs::Gprmc>("nmea/gprmc", 100);
    ros::Publisher gpvtg_pub = nh.advertise<nmea_msgs::Gpvtg>("nmea/gpvtg", 100);
    ros::Publisher gpzda_pub = nh.advertise<nmea_msgs::Gpzda>("nmea/gpzda", 100);

    nmea::NMEAParser parser;
    nmea::NMEASentence nmea;
    parser.log = true;
    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(5);
    while (ros::ok())
    {

        ros::spinOnce();

        if (ser.available())
        {
            ROS_INFO_STREAM("Reading from serial port");

            ros::Time now = ros::Time::now();
            std::string text = ser.read(ser.available());
            // ROS_INFO_STREAM("Read: " << text);
            //             text = R"($GNRMC,231938.60,A,5330.0158984,N,00216.5258340,W,0.31,271.54,300823,,,A*5F
            // $GNGGA,231938.60,5330.0158984,N,00216.5258340,W,1,06,6.5,52.140,M,51.454,M,0.0,*78
            // $GPGSA,A,3,11,12,,,,,,,,,,,8.7,6.5,5.7*3F
            // $GLGSA,A,3,75,,,,,,,,,,,,8.7,6.5,5.7*22
            // $GAGSA,A,3,13,,,,,,,,,,,,8.7,6.5,5.7*2F
            // $GBGSA,A,3,12,34,,,,,,,,,,,8.7,6.5,5.7*2A
            // $GPGSV,1,1,02,11,22,088,36,12,51,075,40,,,,,,,,*7F
            // $GLGSV,1,1,01,75,71,050,36,,,,,,,,,,,,*50
            // $GAGSV,1,1,01,13,28,076,37,,,,,,,,,,,,*54
            // $GBGSV,1,1,02,12,24,041,35,34,42,071,36,,,,,,,,*6D
            // $GNVTG,271.54,T,,M,0.31,N,0.58,K,A*29
            // $GNGST,231938.60,11.000,,,,4.700,6.400,5.900*5E
            // $GNZDA,231938.60,30,08,2023,00,00*74
            // $GNEBP,,,,,,M*13
            // $PTNL,AVR,212405.20,+52.1531,Yaw,-0.0806,Tilt,,,12.575,3,1.4,16*39
            // $*-->The,following,should,fail
            // $GS
            // %GS,,*1E
            // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*55
            // $GPGGA,205630.945,3346.1070,N,08423.6687,W,0,03,,30.8,M,-30.8,M,*73)";


            std::vector<nmea::NMEASentence> nmea = parser.getSentencesFromRawText(text);
            for (size_t i = 0; i < nmea.size(); i++)
            {
                if (!nmea[i].valid())
                {
                    ROS_INFO_STREAM("Invalid sentence: " + nmea[i].text);
                    continue;
                }
                if (nmea[i].name == "GGA")
                {
                    nmea_msgs::Gpgga gpgga;
                    gpgga.header.stamp = now;
                    parser.parseParameters(gpgga, nmea[i]);
                    gpgga_pub.publish(gpgga);
                }
                else if (nmea[i].name == "GSA")
                {
                    nmea_msgs::Gpgsa gpgsa;
                    gpgsa.header.stamp = now;
                    parser.parseParameters(gpgsa, nmea[i]);
                    gpgsa_pub.publish(gpgsa);
                }
                else if (nmea[i].name == "GST")
                {
                    nmea_msgs::Gpgst gpgst;
                    gpgst.header.stamp = now;
                    parser.parseParameters(gpgst, nmea[i]);
                    gpgst_pub.publish(gpgst);
                }
                else if (nmea[i].name == "GSV")
                {
                    nmea_msgs::Gpgsv gpgsv;
                    gpgsv.header.stamp = now;
                    parser.parseParameters(gpgsv, nmea[i]);
                    gpgsv_pub.publish(gpgsv);
                }
                else if (nmea[i].name == "RMC")
                {
                    nmea_msgs::Gprmc gprmc;
                    gprmc.header.stamp = now;
                    parser.parseParameters(gprmc, nmea[i]);
                    gprmc_pub.publish(gprmc);
                }
                else if (nmea[i].name == "VTG")
                {
                    nmea_msgs::Gpvtg gpvtg;
                    gpvtg.header.stamp = now;
                    parser.parseParameters(gpvtg, nmea[i]);
                    gpvtg_pub.publish(gpvtg);
                }
                else if (nmea[i].name == "ZDA")
                {
                    nmea_msgs::Gpzda gpzda;
                    gpzda.header.stamp = now;
                    parser.parseParameters(gpzda, nmea[i]);
                    gpzda_pub.publish(gpzda);
                }
                else
                {
                    nmea_msgs::Sentence sentence;
                    sentence.header.stamp = now;
                    parser.parseParameters(sentence, nmea[i]);
                    sentence_pub.publish(sentence);
                }
            }
            ros::Time end = ros::Time::now();
            ROS_INFO_STREAM("Time: " << (end - now).toSec());
        }
        loop_rate.sleep();
    }
}