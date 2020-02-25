#ifndef NTNU_CYBORG_SIMON_SAYS_GESTURES_H
#define NTNU_CYBORG_SIMON_SAYS_GESTURES_H

#include "ntnu_cyborg_simon_says/event.h"

#include <k2_client/BodyArray.h>

namespace ntnu_cyborg_simon_says
{
    enum class Gesture : int
    {
        wave            = 0,
        handInFront     = 1,
        liftRightFoot   = 2,
        liftLeftFoot    = 3,
        flexnes         = 4,
        handsStraightUp = 5,
        touchRightFoot  = 6
    };

    class GestureEvent : public Event
    {
        public:
            GestureEvent(Gesture);
            virtual std::string announcement();
            virtual bool        completed();
            virtual void        start();
            virtual double      timeout();
        private:
            Gesture gesture_;
    };

    namespace gestures
    {
        void bodyArrayMessageHandler(const k2_client::BodyArray);
    }
}

#endif // NTNU_CYBORG_SIMON_SAYS_GESTURES_H

