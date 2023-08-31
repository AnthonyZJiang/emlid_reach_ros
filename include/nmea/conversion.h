/*
 * NumberConversion.h
 *
 *  Created on: Aug 14, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#pragma once

#include <cstdint>
#include <string>
#include <exception>

namespace nmea
{
	class ConversionError : public std::exception
	{
	public:
		std::string message;
		ConversionError(std::string msg)
			: message(msg){};

		virtual ~ConversionError(){};

		std::string what()
		{
			return message;
		}
	};

	double parseDouble(std::string s);

	float parseFloat(std::string s);

	int64_t parseInt(std::string s, int radix = 10);

	double latLongToDeg(std::string llstr, std::string dir);

	double knotsToKilometersPerHour(double knots);

	double toUtcSeconds(std::string raw_ts);
}
