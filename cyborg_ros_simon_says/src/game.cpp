#include "ntnu_cyborg_simon_says/game.h"

#include "ntnu_cyborg_coordinator/RequestControl.h"
#include "ntnu_cyborg_coordinator/ReleaseControl.h"
#include "ros/console.h"

#include <algorithm>

namespace
{
    namespace local
    {
        const char* getStateName(const ntnu_cyborg_simon_says::Game::State state)
        {
            switch (state)
            {
                case ntnu_cyborg_simon_says::Game::State::none:
                    return "none";
                case ntnu_cyborg_simon_says::Game::State::initialize:
                    return "initialize";
                case ntnu_cyborg_simon_says::Game::State::awaitStartEvent:
                    return "awaitStartEvent";
                case ntnu_cyborg_simon_says::Game::State::announceEventStart:
                    return "announceEventStart";
                case ntnu_cyborg_simon_says::Game::State::awaitEventCompleted:
                    return "awaitEventCompleted";
                case ntnu_cyborg_simon_says::Game::State::gameCompleted:
                    return "gameCompleted";
                case ntnu_cyborg_simon_says::Game::State::gameOver:
                    return "gameOver";
                default:
                    return "unknown";
            }
        }
    }
}

ntnu_cyborg_simon_says::Game::Game(
    ros::NodeHandle                     node,
    std::shared_ptr<Event>              startEvent,
    std::vector<std::shared_ptr<Event>> eventLibrary,
    std::shared_ptr<Output>             output)
    : node_(node),
      use_coordinator_(false),
      state_(State::initialize),
      startEvent_(startEvent),
      eventLibrary_(eventLibrary),
      output_(output)
{
    node.getParam("use_coordinator", use_coordinator_);

    if (use_coordinator_)
    {
        releaseControlService_ = node.serviceClient<ntnu_cyborg_coordinator::ReleaseControl>(
            "/ntnu_cyborg_coordinator/releaseControl");
        requestControlService_ = node.serviceClient<ntnu_cyborg_coordinator::RequestControl>(
            "/ntnu_cyborg_coordinator/requestControl");
    }

    ROS_INFO_NAMED("ntnu_cyborg_simon_says", "use_coordinator=%s", use_coordinator_ ? "true" : "false");

    reset();
}

bool
ntnu_cyborg_simon_says::Game::hasStateTimeoutTimerExpired(
    const TimePoint now) const
{
    if (isStateTimeoutTimerSet_)
    {
        return now >= stateTimeoutTimerValue_;
    }
    else
    {
        return true;
    }
}

bool
ntnu_cyborg_simon_says::Game::hasStateTransitionTimerExpired(
    const TimePoint now) const
{
    if (isStateTransitionTimerSet_)
    {
        return now >= stateTransitionTimerValue_;
    }
    else
    {
        return true;
    }
}

template <typename T>
inline
T
ntnu_cyborg_simon_says::Game::parameter(const std::string& key)
{
    T value;

    if (!node_.getParamCached(key, value))
    {
        ROS_INFO_NAMED("ntnu_cyborg_simon_says", "Failed to get parameter '%s' value!", key.c_str());
    }

    return value;
}

void
ntnu_cyborg_simon_says::Game::process()
{
    const auto now = Clock::now();

    if (state_ != lastState_)
    {
        ROS_INFO_NAMED("ntnu_cyborg_simon_says", "Entered state '%s'!", local::getStateName(state_));
        lastState_ = state_;
    }

    if (hasStateTransitionTimerExpired(now))
    {
        switch (state_)
        {
            case State::initialize:
            {
                releaseControl();
                reset();
                startEvent_->start();
                setState(State::awaitStartEvent);

                break;
            }
            case State::awaitStartEvent:
            {
                if (startEvent_->completed())
                {
                    if (requestControl())
                    {
                        output_->say(parameter<std::string>("message_game_start"));

                        std::shuffle(eventLibrary_.begin(),
                                     eventLibrary_.end(),
                                     rng_);

                        events_.clear();

                        const auto challengeCount = parameter<int>("challenge_count");

                        for (auto i = 0; i < challengeCount; i++)
                        {
                            events_.push_back(eventLibrary_[i]);
                        }

                        setState(State::announceEventStart, now, parameter<double>("delay_event_begin"));
                    }
                    else
                    {
                        startEvent_->start();
                    }
                }

                break;
            }
            case State::announceEventStart:
            {
                if (!events_.empty())
                {
                    currentEvent_ = events_.front();
                    events_.pop_front();

                    event_positive_ = std::bernoulli_distribution()(rng_);

                    currentEvent_->start();

                    const auto message = event_positive_
                        ? parameter<std::string>("message_command_preface") + ' ' + currentEvent_->announcement()
                        : currentEvent_->announcement();

                    output_->say(message);

                    const auto period = event_positive_
                        ? currentEvent_->timeout() : currentEvent_->timeout() / 2.0;

                    setState(State::awaitEventCompleted, now, 0.0, period);
                }
                else
                {
                    output_->say(parameter<std::string>("message_game_completed"));
                    setState(State::gameOver);
                }

                break;
            }
            case State::awaitEventCompleted:
            {
                if (hasStateTimeoutTimerExpired(now))
                {
                    if (event_positive_)
                    {
                        output_->say(parameter<std::string>("message_command_ignored"));
                        setState(State::gameOver);
                    }
                    else
                    {
                        output_->say(parameter<std::string>("message_non_command_ignored"));
                        setState(State::announceEventStart, now, parameter<double>("delay_event_begin"));
                    }

                    currentEvent_->stop();
                }
                else if (currentEvent_->completed())
                {
                    if (event_positive_)
                    {
                        output_->say(parameter<std::string>("message_command_followed"));
                        setState(State::announceEventStart, now, parameter<double>("delay_event_begin"));
                    }
                    else
                    {
                        output_->say(parameter<std::string>("message_non_command_followed"));
                        setState(State::gameOver);
                    }

                    currentEvent_->stop();
                }

                break;
            }
            case State::gameOver:
            {
                startEvent_->stop();
                setState(State::initialize, now, parameter<double>("delay_game_restart"));
                break;
            }
        }
    }
}

void
ntnu_cyborg_simon_says::Game::releaseControl()
{
    if (use_coordinator_)
    {
        ntnu_cyborg_coordinator::ReleaseControl call;
        call.request.id = ros::this_node::getName();
        releaseControlService_.call(call);
    }
}

bool
ntnu_cyborg_simon_says::Game::requestControl()
{
    if (use_coordinator_)
    {
        ntnu_cyborg_coordinator::RequestControl call;
        call.request.id = ros::this_node::getName();
        return requestControlService_.call(call) && call.response.controlReceived;
    }
    else
    {
        return true;
    }
}

void
ntnu_cyborg_simon_says::Game::reset()
{
    lastState_                 = State::none;
    state_                     = State::initialize;
    isStateTransitionTimerSet_ = false;
    isStateTimeoutTimerSet_    = false;

    rng_ = std::default_random_engine(std::random_device()());

    currentEvent_.reset();
    events_.clear();
}

void
ntnu_cyborg_simon_says::Game::setState(
    const State state)
{
    state_                     = state;
    isStateTransitionTimerSet_ = false;
    isStateTimeoutTimerSet_    = false;
}

void
ntnu_cyborg_simon_says::Game::setState(
    const State     state,
    const TimePoint now,
    const double    stateTransitionDelay,
    const double    stateTimeoutDelay)
{
    if (isStateTransitionTimerSet_ = stateTransitionDelay != 0.0)
    {
        stateTransitionTimerValue_ = now +
            std::chrono::milliseconds(
                static_cast<long>(1000.0 * stateTransitionDelay));
    }

    if (isStateTimeoutTimerSet_ = stateTimeoutDelay != 0.0)
    {
        stateTimeoutTimerValue_ = now +
            std::chrono::milliseconds(
                static_cast<long>(1000.0 * stateTimeoutDelay));
    }

    state_ = state;
}

