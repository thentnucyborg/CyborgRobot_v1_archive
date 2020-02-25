#include "ntnu_cyborg_simon_says/event.h"
#include "ntnu_cyborg_simon_says/game.h"
#include "ntnu_cyborg_simon_says/gestures.h"
#include "ntnu_cyborg_simon_says/troll_output.h"

#include "ros/ros.h"

#include <memory>
#include <string>
#include <vector>

std::shared_ptr<ntnu_cyborg_simon_says::Game>
setupGame(ros::NodeHandle& node)
{
    using namespace ntnu_cyborg_simon_says;

    auto startEvent = std::make_shared<GestureEvent>(Gesture::wave);

    std::vector<std::shared_ptr<Event>> eventLibrary
    {
        std::make_shared<GestureEvent>(Gesture::handInFront),
        std::make_shared<GestureEvent>(Gesture::liftRightFoot),
        std::make_shared<GestureEvent>(Gesture::liftLeftFoot),
        std::make_shared<GestureEvent>(Gesture::flexnes),
        std::make_shared<GestureEvent>(Gesture::handsStraightUp)
    };

    auto output = std::make_shared<TrollOutput>(node);

    return std::make_shared<Game>(node, startEvent, eventLibrary, output);
}

ros::Subscriber
setupGestureSubscription(ros::NodeHandle& node)
{
    std::string gestureTopic;
    node.param<std::string>("gesture_topic", gestureTopic, "/head/kinect2/bodyArray");

    int gestureQueueSize;
    node.param("gesture_queue_size", gestureQueueSize, 10);

    return node.subscribe(
        gestureTopic,
        gestureQueueSize,
        ntnu_cyborg_simon_says::gestures::bodyArrayMessageHandler);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "ntnu_cyborg_simon_says");
    ros::NodeHandle node("~");

    auto game              = setupGame(node);
    auto gestureSubscriber = setupGestureSubscription(node);

    ROS_INFO_NAMED("ntnu_cyborg_simon_says", "ntnu_cyborg_simon_says ready to serve!");

    int loopRateValue;
    node.param("loop_rate", loopRateValue, 10);

    ros::Rate loopRate(loopRateValue);

    while (true)
    {
        ros::spinOnce();
        game->process();
    }
}

