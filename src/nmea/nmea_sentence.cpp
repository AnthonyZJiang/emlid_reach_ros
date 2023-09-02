#include "nmea/nmea_sentence.h"

using namespace nmea;

NMEASentence::NMEASentence()
{
}

NMEASentence::~NMEASentence()
{
}

bool NMEASentence::valid() const
{
	return isValid;
}
