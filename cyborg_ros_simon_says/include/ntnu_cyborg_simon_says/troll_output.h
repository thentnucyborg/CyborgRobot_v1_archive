#ifndef NTNU_CYBORG_SIMON_SAYS_TROLL_OUTPUT_H
#define NTNU_CYBORG_SIMON_SAYS_TROLL_OUTPUT_H

#include "ntnu_cyborg_simon_says/output.h"
#include "ros/ros.h"

#include <string>

namespace ntnu_cyborg_simon_says
{
    class TrollOutput : public Output
    {
        public:
            TrollOutput(ros::NodeHandle);
            virtual void say(const std::string&);

        private:
            ros::Publisher publisher_;
            std::string    expression_;
    };
}

#endif // NTNU_CYBORG_SIMON_SAYS_TROLL_OUTPUT_H

