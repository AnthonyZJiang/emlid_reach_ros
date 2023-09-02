#pragma once

#include <string>
#include <vector>

namespace nmea
{
    class NMEASentence
    {
        friend class NMEAParser;

    private:
        std::string paramText;
        bool isValid;

    public:
        std::string text;                    // whole plaintext of the received command
        std::string name;                    // name of the command
        std::vector<std::string> parameters; // list of parameters from the command

        NMEASentence();
        virtual ~NMEASentence();
        bool valid() const;
    };
}
