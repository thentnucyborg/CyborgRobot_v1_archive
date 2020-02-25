#include <cstdlib>
#include <map>
#include <typeinfo>
#include "ros/ros.h"
#include "cyborg_state_machine/state_msgs.h"
#include "std_msgs/String.h"




template <typename ROSMessageType>
class ResourceGateKeeper
{
    public:        
        
        ResourceGateKeeper(std::string resourceName, std::string inputTopicEnding, std::string outputTopic);
       
    private:
    
        std::string resourceName; 
        std::string inputTopicEnding;
        std::string outputTopic;
        ros::NodeHandle n;
        ros::Publisher resourcePublisher;
        ros::Subscriber stateChangeSubscriber;
        ros::Subscriber moduleSubscriber;    
        
        void newStateCb(const cyborg_state_machine::state_msgs& newStateMsg);
        void dataReceivedCb(const ROSMessageType msg);
             

};

template<typename ROSMessageType>
ResourceGateKeeper<ROSMessageType>::ResourceGateKeeper(std::string resourceName, std::string inputTopicEnding, std::string outputTopic){

    //Set class variables
    this->resourceName = resourceName;
    this->inputTopicEnding = inputTopicEnding;
    this->outputTopic = outputTopic;
    
    //Subscribe to state change topic and connect callback function "newStateCb"
    stateChangeSubscriber = n.subscribe("/cyborg_state_machine/state_change", 1, &ResourceGateKeeper::newStateCb, this);
    
    //Advertise output topic
    resourcePublisher = n.advertise<ROSMessageType>(outputTopic, 1000);
    ROS_INFO("Gate keeper initialized");
}


//Callback connected to state change topic. Called whenever a state change message is received.
template<typename ROSMessageType>
void ResourceGateKeeper<ROSMessageType>::newStateCb(const cyborg_state_machine::state_msgs& newStateMsg){

    ROS_INFO("Changed to state \"%s\"", newStateMsg.change_to.c_str());
    
    
    //Shut down module subscriber from previous state
    this->moduleSubscriber.shutdown();
    
    //Retrieve resource mapping dictionary from parameter server
    std::map<std::string, std::string> resourceTopicRemap;
    n.getParam("/state_resources", resourceTopicRemap);
    
    //Search for name of own resource in mapping
    if (resourceTopicRemap.find(resourceName) != resourceTopicRemap.end()){
    
            //Get name of module with resource access from dictionary
            std::string currentTopicName =  resourceTopicRemap.find(resourceName)->second;   
            
            //Subscribe to topic published by module with resource access
            this->moduleSubscriber = this->n.subscribe(currentTopicName + inputTopicEnding, 1000, &ResourceGateKeeper<ROSMessageType>::dataReceivedCb, this);
            
            ROS_INFO("Found \"%s\" in ParamServer. Listening to input from \"%s\"", resourceName.c_str(), currentTopicName.c_str());
                       
    }    
    
    
}

//Callback connected to input topic. Publishes input data to output topic whenever received.
template<class ROSMessageType>
void ResourceGateKeeper<ROSMessageType>::dataReceivedCb(const ROSMessageType msg){
    
    resourcePublisher.publish(msg);
    ROS_INFO("Message routed to resource topic: \"%s\"", outputTopic.c_str());

}



