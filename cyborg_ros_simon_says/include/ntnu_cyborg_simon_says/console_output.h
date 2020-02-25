#ifndef NTNU_CYBORG_SIMON_SAYS_CONSOLE_OUTPUT_H
#define NTNU_CYBORG_SIMON_SAYS_CONSOLE_OUTPUT_H

#include "ros/ros.h"
#include "ntnu_cyborg_simon_says/output.h"

namespace ntnu_cyborg_simon_says
{
    class ConsoleOutput : public Output
    {
        public:
            ConsoleOutput();
            virtual void say(const std::string&);
    };
}

#endif // NTNU_CYBORG_SIMON_SAYS_CONSOLE_OUTPUT_H

