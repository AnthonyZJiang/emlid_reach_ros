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
