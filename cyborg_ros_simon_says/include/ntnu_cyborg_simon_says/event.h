#ifndef NTNU_CYBORG_SIMON_SAYS_EVENT_H
#define NTNU_CYBORG_SIMON_SAYS_EVENT_H

#include <string>

namespace ntnu_cyborg_simon_says
{
    class Event
    {
        public:
            virtual ~Event() {}
            virtual std::string announcement() = 0;
            virtual bool        completed()    = 0;
            virtual void        start()        {};
            virtual void        stop()         {};
            virtual double      timeout()      = 0;
    };
}

#endif // NTNU_CYBORG_SIMON_SAYS_EVENT_H

