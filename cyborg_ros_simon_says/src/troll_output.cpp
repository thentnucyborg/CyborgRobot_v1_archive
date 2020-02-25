#include "ntnu_cyborg_simon_says/troll_output.h"

#include "trollnode/Expression.h"
#include "ros/ros.h"

ntnu_cyborg_simon_says::TrollOutput::TrollOutput(ros::NodeHandle node)
{
    node.param<std::string>("troll_expression", expression_, "smile");

    std::string trollExpressionTopic;
    node.param<std::string>("troll_expression_topic", trollExpressionTopic, "/trollExpression");

    ROS_INFO_NAMED("ntnu_cyborg_simon_says", "Publishing speech to topic: %s", trollExpressionTopic.c_str());

    int trollExpressionQueueSize;
    node.param("troll_expression_queue_size", trollExpressionQueueSize, 10);

    publisher_ = node.advertise<trollnode::Expression>(
        trollExpressionTopic, trollExpressionQueueSize);
}

void
ntnu_cyborg_simon_says::TrollOutput::say(const std::string& text)
{
    trollnode::Expression message;

    message.speech     = text;
    message.expression = expression_;

    publisher_.publish(message);

    ROS_INFO_NAMED("ntnu_cyborg_simon_says", "SPEECH: %s", text.c_str());
}

