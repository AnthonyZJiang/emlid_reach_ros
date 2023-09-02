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


----------------------------- 3RD PARTY LICENSES -----------------------------


NemaTode is released under the ZLib license.

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software
    in a product, an acknowledgment in the product documentation would be
    appreciated but is not required.
    2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.
    3. This notice may not be removed or altered from any source
    distribution.
 */

#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <ros/console.h>

#include "nmea/nmea_parser.h"
#include "nmea/conversion.h"

using namespace std;
using namespace nmea;

// --------- PARSER ERROR ----------

NMEAParseError::NMEAParseError(std::string msg)
	: message(msg)
{
}

NMEAParseError::~NMEAParseError()
{
}

std::string NMEAParseError::what()
{
	return message;
}

// ------------ HELPERS ------------

// true if the text contains a non-alpha numeric value
bool hasNonAlphaNum(string txt)
{
	for (const char i : txt)
	{
		if (!isalnum(i))
		{
			return true;
		}
	}
	return false;
}

// true if the name contains only alpha characters or ','
bool isValidName(string txt)
{
	for (const char i : txt)
	{
		if (!isalpha(i) && i != ',')
		{
			return false;
		}
	}
	return true;
}

// true if alphanumeric or '+' or '-' or '.'
bool isValidParamChars(string txt)
{
	for (const char i : txt)
	{
		if (!isalnum(i) && i != '+' & i != '-' && i != '.')
		{
			return false;
		}
	}
	return true;
}

// remove carriage return from end of string
void removeCR(string &txt)
{
	if (*(txt.end() - 1) != '\r')
	{
		return;
	}
	// if (*(txt.end() - 2) == '\r')
	// {
	// 	txt = txt.substr(0, txt.size() - 2);
	// 	return;
	// }
	txt = txt.substr(0, txt.size() - 1);
}

// remove all whitespace
void squish(string &str)
{

	char chars[] = {'\t', ' '};
	for (const char i : chars)
	{
		// needs include <algorithm>
		str.erase(std::remove(str.begin(), str.end(), i), str.end());
	}
}

// remove side whitespace
void trim(string &str)
{
	stringstream trimmer;
	trimmer << str;
	str.clear();
	trimmer >> str;
}

// ---------- NMEA PARSER ----------

NMEAParser::NMEAParser()
	: log(false), ignoreEmptyChecksum(true)
{
}

NMEAParser::~NMEAParser()
{
}

// Loggers
void NMEAParser::logInfo(string txt)
{
	if (log)
	{
		cout << "[Info]    " << txt << endl;
	}
}
void NMEAParser::logWarning(string txt)
{
	if (log)
	{
		cout << "[Warning] " << txt << endl;
	}
}
void NMEAParser::logError(string txt)
{
	ROS_ERROR_STREAM("[REACH]::NMEA " << txt);
}

std::vector<NMEASentence> NMEAParser::getSentencesFromRawText(std::string text)
{
	logInfo("NEW data received.");
	istringstream f(text);
	string sentence;
	std::vector<NMEASentence> sentences;
	while (getline(f, sentence, '\n'))
	{
		logInfo("Processing NEW sentence...");
		if (sentence.empty())
		{
			logWarning("Blank string -- Skipped processing.");
			continue;
		}
		NMEASentence nmea;
		nmea.isValid = false;
		try
		{
			cleanUpSentence(sentence);
			nmea.text = sentence;
			trimSentenceStart(sentence);
			checkChecksum(sentence);
			parseName(nmea, sentence);
			parseParameters(nmea);
			nmea.isValid = true;
			sentences.push_back(nmea);
		}
		catch (NMEAParseError &e)
		{
			logError(e.what());
			continue;
		}
		catch (std::exception &e)
		{
			string s = " >> NMEA Parser Internal Error: Indexing error?... ";
			throw std::runtime_error(s + e.what());
		}
	}
	return sentences;
}

void NMEAParser::cleanUpSentence(std::string &txt)
{
	removeCR(txt);
	squish(txt);
	logInfo(string("NMEA string: (\"") + txt + "\")");
}

void NMEAParser::trimSentenceStart(std::string &txt)
{
	size_t startbyte = 0;
	size_t dollar = txt.find_last_of('$');
	if (dollar == string::npos)
	{
		throw NMEAParseError("No '$' found in string. Sentence:" + txt);
	}
	else
	{
		startbyte = dollar;
	}
	txt = txt.substr(startbyte + 1);
}

uint8_t NMEAParser::calculateXORChecksum(string s)
{
	uint8_t checksum = 0;
	for (const char i : s)
	{
		checksum = checksum ^ i;
	}
	return checksum;
}

void NMEAParser::checkChecksum(std::string &txt)
{
	size_t lastStarI = txt.find_last_of('*');

	bool haschecksum = lastStarI != string::npos;
	if (haschecksum)
	{
		std::string checksumText = txt.substr(lastStarI + 1, txt.size() - lastStarI);
		uint8_t checksumParsed;
		try
		{
			checksumParsed = (uint8_t)parseInt(checksumText, 16);
		}
		catch (ConversionError &)
		{
			throw NMEAParseError("parseInt() error. Parsed checksum string [" + checksumText + "] was not readable as hex.");
		}

		// remove checksum from text
		txt = txt.substr(0, lastStarI);
		if (checksumParsed == calculateXORChecksum(txt))
		{
			logInfo("Checksum is valid.");
			return;
		}
		throw NMEAParseError("Checksum is invalid! Sentence:" + txt);
	}
	if (ignoreEmptyChecksum)
	{
		logWarning("No checksum information provided. Could not find '*'. Empty checksum is set to be ignored. Sentence:" + txt);
		return;
	}
	throw NMEAParseError("No checksum information provided. Could not find '*'. Sentence:" + txt);
}

void NMEAParser::parseName(NMEASentence &nmea, std::string &txt)
{
	size_t comma = txt.find(',');

	// "$," case - no name
	if (comma == 0)
	{
		throw NMEAParseError("No name is provided or name is incomplete. Unable to parse the sentence:" + nmea.text);
	}

	if (comma == string::npos)
	{ // comma not found, but there is a name...
		if (txt.empty())
		{
			throw NMEAParseError("This is an empty sentence.");
		}
		if (hasNonAlphaNum(txt))
		{
			throw NMEAParseError("Name [" + txt + "]contains non-alpha-numeric characters. Unable to parse the sentence.");
		}
		if (txt.size() < 4)
		{
			throw NMEAParseError("Name [" + txt + "] is too short. Unable to parse the sentence.");
		}
		nmea.name = txt.substr(2, txt.size() - 2);
		txt = "";
		return;
	}

	if (comma < 4)
	{
		throw NMEAParseError("Name [" + txt + "] is too short. Unable to parse the sentence.");
	}

	// recognised talker list: GA, GB, GI, GL, GN, GP, GQ
	std::string talker = txt.substr(0, 2);
	if (talker == "GA" || talker == "GB" || talker == "GI" || talker == "GL" || talker == "GN" || talker == "GP" || talker == "GQ")
	{
		nmea.name = txt.substr(2, comma - 2);
	}
	else
	{
		nmea.name = txt.substr(0, comma);
		if (nmea.name == "PTNL")
		{
			comma = txt.find(',', comma + 1);
			nmea.name = txt.substr(0, comma);
		}
	}

	if (!isValidName(nmea.name))
	{
		nmea.name = "";
		throw NMEAParseError("Name [" + nmea.name + "]contains non-alpha-numeric characters. Unable to parse the sentence.");
	}

	// extract parameter texts
	nmea.paramText = txt.substr(comma + 1);
}

void NMEAParser::parseParameters(NMEASentence &nmea)
{
	if (nmea.paramText.empty())
	{
		nmea.parameters.push_back("");
		return;
	}
	istringstream f(nmea.paramText);
	string s;
	while (getline(f, s, ','))
	{
		if (!isValidParamChars(s))
		{
			throw NMEAParseError("Invalid character (non-alpha-num) in parameter [" + s + "].");
		}
		nmea.parameters.push_back(s);
	}

	// above line parsing does not add a blank parameter if there is a comma at the end...
	//  so do it here.
	if (*(nmea.paramText.end() - 1) == ',')
	{
		nmea.parameters.push_back("");
	}

	logInfo("Found " + to_string(nmea.parameters.size()) + " parameters.");
}

void NMEAParser::parseParameters(nmea_msgs::Sentence &sentence, NMEASentence &nmea)
{
	sentence.sentence = nmea.text;
	logInfo("ROS Sentence parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gprmc &gprmc, NMEASentence &nmea)
{
	// float64 utc_seconds
	// string position_status
	// float64 lat
	// float64 lon
	// string lat_dir
	// string lon_dir
	// float32 speed
	// float32 track
	// string date
	// float32 mag_var
	// string mag_var_direction
	// string mode_indicator
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 12)
		{
			throw ConversionError("Expected 12 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gprmc.utc_seconds = toUtcSeconds(nmea.parameters[0]);
		gprmc.position_status = nmea.parameters[1];
		gprmc.lat = parseDouble(nmea.parameters[2]);
		gprmc.lon = parseDouble(nmea.parameters[4]);
		gprmc.lat_dir = nmea.parameters[3];
		gprmc.lon_dir = nmea.parameters[5];
		gprmc.speed = parseFloat(nmea.parameters[6]);
		gprmc.track = parseFloat(nmea.parameters[7]);
		gprmc.date = nmea.parameters[8];
		gprmc.mag_var = parseFloat(nmea.parameters[9]);
		gprmc.mag_var_direction = nmea.parameters[10];
		gprmc.mode_indicator = nmea.parameters[11];
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS RMC parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gpgga &gpgga, NMEASentence &nmea)
{
	// float64 utc_seconds
	// float64 lat
	// float64 lon
	// string lat_dir
	// string lon_dir
	// uint32 gps_qual
	// uint32 num_sats
	// float32 hdop
	// float32 alt
	// string altitude_units
	// float32 undulation
	// string undulation_units
	// uint32 diff_age
	// string station_id
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 14)
		{
			throw ConversionError("Expected 14 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gpgga.utc_seconds = toUtcSeconds(nmea.parameters[0]);
		gpgga.lat = parseDouble(nmea.parameters[1]);
		gpgga.lat_dir = nmea.parameters[2];
		gpgga.lon = parseDouble(nmea.parameters[3]);
		gpgga.lon_dir = nmea.parameters[4];
		gpgga.gps_qual = (uint32_t)parseInt(nmea.parameters[5]);
		gpgga.num_sats = (uint32_t)parseInt(nmea.parameters[6]);
		gpgga.hdop = parseFloat(nmea.parameters[7]);
		gpgga.alt = parseFloat(nmea.parameters[8]);
		gpgga.altitude_units = nmea.parameters[9];
		gpgga.undulation = parseFloat(nmea.parameters[10]);
		gpgga.undulation_units = nmea.parameters[11];
		gpgga.diff_age = (uint32_t)round(parseDouble(nmea.parameters[12]));
		gpgga.station_id = nmea.parameters[13];
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS GGA parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gpgsa &gpgsa, NMEASentence &nmea)
{
	// string auto_manual_mode
	// uint8 fix_mode
	// uint8[] sv_ids
	// float32 pdop
	// float32 hdop
	// float32 vdop
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 17)
		{
			throw ConversionError("Expected 17 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gpgsa.auto_manual_mode = nmea.parameters[0];
		gpgsa.fix_mode = (uint8_t)parseInt(nmea.parameters[1]);
		for (size_t i = 2; i < 14; i++)
		{
			gpgsa.sv_ids.push_back((uint8_t)parseInt(nmea.parameters[i]));
		}
		gpgsa.pdop = parseFloat(nmea.parameters[14]);
		gpgsa.hdop = parseFloat(nmea.parameters[15]);
		gpgsa.vdop = parseFloat(nmea.parameters[16]);
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS GSA parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gpgsv &gpgsv, NMEASentence &nmea)
{
	// uint8 n_msgs
	// uint8 msg_number
	// uint8 n_satellites
	// GpgsvSatellite[] satellites
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 19)
		{
			throw ConversionError("Expected 19 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gpgsv.n_msgs = (uint8_t)parseInt(nmea.parameters[0]);
		gpgsv.msg_number = (uint8_t)parseInt(nmea.parameters[1]);
		gpgsv.n_satellites = (uint8_t)parseInt(nmea.parameters[2]);
		for (size_t i = 3; i < nmea.parameters.size(); i += 4)
		{
			nmea_msgs::GpgsvSatellite sat;
			// uint8 prn
			// uint8 elevation
			// uint16 azimuth
			// int8 snr
			sat.prn = (uint8_t)parseInt(nmea.parameters[i + 1]);
			sat.elevation = (uint8_t)parseInt(nmea.parameters[i + 2]);
			sat.azimuth = (uint16_t)parseInt(nmea.parameters[i + 3]);
			sat.snr = (int8_t)parseFloat(nmea.parameters[i]);
			gpgsv.satellites.push_back(sat);
		}
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS GSV parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gpgst &gpgst, NMEASentence &nmea)
{
	// float64 utc_seconds
	// float32 rms
	// float32 semi_major_dev
	// float32 semi_minor_dev
	// float32 orientation
	// float32 lat_dev
	// float32 lon_dev
	// float32 alt_dev
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 8)
		{
			throw ConversionError("Expected 8 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gpgst.utc_seconds = toUtcSeconds(nmea.parameters[0]);
		gpgst.rms = parseFloat(nmea.parameters[1]);
		gpgst.semi_major_dev = parseFloat(nmea.parameters[2]);
		gpgst.semi_minor_dev = parseFloat(nmea.parameters[3]);
		gpgst.orientation = parseFloat(nmea.parameters[4]);
		gpgst.lat_dev = parseFloat(nmea.parameters[5]);
		gpgst.lon_dev = parseFloat(nmea.parameters[6]);
		gpgst.alt_dev = parseFloat(nmea.parameters[7]);
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS GST parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gpvtg &gpvtg, NMEASentence &nmea)
{
	// float64 tmg_a
	// float64 track_t
	// string track_t_ref
	// float64 track_m
	// string track_m_ref
	// float32 speed_n
	// string speed_n_unit
	// float32 speed_k
	// string speed_k_unit
	// string mode_indicator
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 9)
		{
			throw ConversionError("Expected 9 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gpvtg.track_t = parseDouble(nmea.parameters[0]);
		gpvtg.track_t_ref = nmea.parameters[1];
		gpvtg.track_m = parseDouble(nmea.parameters[2]);
		gpvtg.track_m_ref = nmea.parameters[3];
		gpvtg.speed_n = parseFloat(nmea.parameters[4]);
		gpvtg.speed_n_unit = nmea.parameters[5];
		gpvtg.speed_k = parseFloat(nmea.parameters[6]);
		gpvtg.speed_k_unit = nmea.parameters[7];
		gpvtg.mode_indicator = nmea.parameters[8];
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS VTG parsed.");
}

void NMEAParser::parseParameters(nmea_msgs::Gpzda &gpzda, NMEASentence &nmea)
{
	// uint32 utc_seconds
	// uint8 day
	// uint8 month
	// uint16 year
	// int8 hour_offset_gmt
	// uint8 minute_offset_gmt
	try
	{
		if (nmea.parameters.empty())
		{
			parseParameters(nmea);
		}
		if (nmea.parameters.size() != 6)
		{
			throw ConversionError("Expected 6 parameters, got " + to_string(nmea.parameters.size()) + ".");
		}
		gpzda.utc_seconds = parseDouble(nmea.parameters[0]);
		gpzda.day = (uint8_t)parseInt(nmea.parameters[1]);
		gpzda.month = (uint8_t)parseInt(nmea.parameters[2]);
		gpzda.year = (uint16_t)parseInt(nmea.parameters[3]);
		gpzda.hour_offset_gmt = (int8_t)parseInt(nmea.parameters[4]);
		gpzda.minute_offset_gmt = (uint8_t)parseInt(nmea.parameters[5]);
	}
	catch (ConversionError &e)
	{
		logError("The following error occurred when parsing [" + nmea.text + "]:\n" + e.what());
	}
	catch (std::exception &e)
	{
		string s = " >> NMEA Parser Internal Error: Indexing error?... ";
		throw std::runtime_error(s + e.what());
	}
	logInfo("ROS ZDA parsed.");
}
