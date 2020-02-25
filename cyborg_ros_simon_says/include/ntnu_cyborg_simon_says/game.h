#ifndef NTNU_CYBORG_SIMON_SAYS_GAME_H
#define NTNU_CYBORG_SIMON_SAYS_GAME_H

#include "ntnu_cyborg_simon_says/event.h"
#include "ntnu_cyborg_simon_says/output.h"

#include "ros/ros.h"

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace ntnu_cyborg_simon_says
{
    class Game
    {
        public:
            typedef std::chrono::monotonic_clock   Clock;
            typedef std::chrono::time_point<Clock> TimePoint;

            enum class State
            {
                none,
                initialize,
                awaitStartEvent,
                announceEventStart,
                awaitEventCompleted,
                gameCompleted,
                gameOver
            };

            Game(ros::NodeHandle                     node,
                 std::shared_ptr<Event>              startEvent,
                 std::vector<std::shared_ptr<Event>> eventLibrary,
                 std::shared_ptr<Output>             output);

            void process();
            void reset();

        private:
            bool hasStateTimeoutTimerExpired(TimePoint now) const;
            bool hasStateTransitionTimerExpired(TimePoint now) const;
            void releaseControl();
            bool requestControl();

            template <typename T>
            T parameter(const std::string&);

            void setState(State);

            void setState(State,
                          TimePoint now,
                          double stateTransitionDelay = 0.0,
                          double stateTimeoutDelay    = 0.0);

            ros::NodeHandle                     node_;

            bool                                use_coordinator_;
            ros::ServiceClient                  releaseControlService_;
            ros::ServiceClient                  requestControlService_;

            State                               lastState_;
            State                               state_;
            std::shared_ptr<Event>              startEvent_;
            std::vector<std::shared_ptr<Event>> eventLibrary_;
            std::shared_ptr<Output>             output_;

            std::deque<std::shared_ptr<Event>>  events_;
            std::shared_ptr<Event>              currentEvent_;

            bool                                isStateTransitionTimerSet_;
            bool                                isStateTimeoutTimerSet_;
            bool                                event_positive_;

            TimePoint                           stateTransitionTimerValue_;
            TimePoint                           stateTimeoutTimerValue_;

            std::default_random_engine          rng_;
    };
}

#endif // NTNU_CYBORG_SIMON_SAYS_GAME_H

