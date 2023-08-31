/*
 * NMEAParser.h
 *
 *  Created on: Aug 12, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <exception>

#include <nmea_msgs/Sentence.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgsa.h>
#include <nmea_msgs/Gpgst.h>
#include <nmea_msgs/Gpgsv.h>
#include <nmea_msgs/GpgsvSatellite.h>
#include <nmea_msgs/Gprmc.h>
#include <nmea_msgs/Gpvtg.h>
#include <nmea_msgs/Gpzda.h>

#include <nmea/nmea_sentence.h>


namespace nmea
{
	class NMEAParser;

	class NMEAParseError : public std::exception
	{
	public:
		std::string message;
		NMEASentence nmea;

		NMEAParseError(std::string msg);
		virtual ~NMEAParseError();

		std::string what();
	};

	class NMEAParser
	{
	private:
		void parseText(NMEASentence &nmea, std::string s);

		void logInfo(std::string s);
		void logWarning(std::string s);
		void logError(std::string s);

		void cleanUpSentence(std::string &txt);
		void trimSentenceStart(std::string &txt);
		void checkChecksum(std::string &txt);
		void parseName(NMEASentence &nmea, std::string &txt);

		static uint8_t calculateXORChecksum(std::string);

	public:
		NMEAParser();
		virtual ~NMEAParser();

		bool log;
		bool ignoreEmptyChecksum;
		std::vector<NMEASentence> getSentencesFromRawText(std::string text);

		void parseParameters(NMEASentence &nmea);
		void parseParameters(nmea_msgs::Sentence &sentence, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gprmc &gprmc, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gpgga &gpgga, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gpgsa &gpgsa, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gpgsv &gpgsv, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gpgst &gpgst, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gpvtg &gpvtg, NMEASentence &nmea);
		void parseParameters(nmea_msgs::Gpzda &gpzda, NMEASentence &nmea);
	};

}
