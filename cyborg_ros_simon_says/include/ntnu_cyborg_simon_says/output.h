#ifndef NTNU_CYBORG_SIMON_SAYS_OUTPUT_H
#define NTNU_CYBORG_SIMON_SAYS_OUTPUT_H

#include <string>

namespace ntnu_cyborg_simon_says
{
    class Output
    {
        public:
            virtual ~Output() {}
            virtual void say(const std::string&) = 0;
    };
}

#endif // NTNU_CYBORG_SIMON_SAYS_OUTPUT_H

