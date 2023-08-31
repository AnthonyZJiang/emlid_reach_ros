/*
 * NumberConversion.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#include <nmea/conversion.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>

namespace nmea
{
	// Returns 0 with "" input
	double parseDouble(std::string s)
	{

		char *p;
		double d = ::strtod(s.c_str(), &p);
		if (*p != 0)
		{
			std::stringstream ss;
			ss << "NumberConversionError: parseDouble() error in argument \"" << s << "\", '"
			   << *p << "' is not a number.";
			throw ConversionError(ss.str());
		}
		return d;
	}

	// Returns 0 with "" input
	float parseFloat(std::string s)
	{

		char *p;
		double d = ::strtof(s.c_str(), &p);
		if (*p != 0)
		{
			std::stringstream ss;
			ss << "NumberConversionError: parseFloat() error in argument \"" << s << "\", '"
			   << *p << "' is not a number.";
			throw ConversionError(ss.str());
		}
		return d;
	}

	// Returns 0 with "" input
	int64_t parseInt(std::string s, int radix)
	{
		char *p;

		int64_t d = ::strtoll(s.c_str(), &p, radix);

		if (*p != 0)
		{
			std::stringstream ss;
			ss << "NumberConversionError: parseInt() error in argument \"" << s << "\", '"
			   << *p << "' is not a number.";
			throw ConversionError(ss.str());
		}
		return d;
	}

	double latLongToDeg(std::string llstr, std::string dir)
	{
		double pd = parseDouble(llstr);
		double deg = trunc(pd / 100); // get ddd from dddmm.mmmm
		double mins = pd - deg * 100;

		deg = deg + mins / 60.0;

		char hdg = 'x';
		if (!dir.empty())
		{
			hdg = dir[0];
		}

		// everything should be N/E, so flip S,W
		if (hdg == 'S' || hdg == 'W')
		{
			deg *= -1.0;
		}

		return deg;
	}

	double knotsToKilometersPerHour(double knots)
	{
		return knots * 1.852;
	}

	double toUtcSeconds(std::string raw_ts)
	{
		double ts = parseDouble(raw_ts);
		int32_t hour = (int32_t)trunc(ts / 10000.0);
		int32_t min = (int32_t)trunc((ts - hour * 10000) / 100.0);
		double sec = ts - min * 100 - hour * 10000;
		return hour * 3600 + min * 60 + sec;
	}

}