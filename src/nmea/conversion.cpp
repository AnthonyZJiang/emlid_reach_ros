/*
 * NumberConversion.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>

#include "nmea/conversion.h"

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

	double latLonToDeg(double pd, std::string dir)
	{
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

	double toUnixTimestamp(uint16_t year, uint8_t month, uint8_t day, double utcSec)
	{
		// http://en.wikipedia.org/wiki/Julian_day
		int32_t a = (14 - month) / 12;
		int32_t y = year + 4800 - a;
		int32_t m = month + 12 * a - 3;
		int32_t jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
		double utc_day = jdn - 2440587.5;
		return utc_day * 86400.0 + utcSec;
	}

	double toUnixTimestamp(std::string raw_date, double utcSec)
	{
		double date = parseDouble(raw_date);
		uint16_t year = (uint16_t)trunc(date / 10000.0);
		uint8_t month = (uint8_t)trunc((date - year * 10000) / 100.0);
		uint8_t day = (uint8_t)trunc(date - month * 100 - year * 10000);

		return toUnixTimestamp(year, month, day, utcSec);
	}
}
