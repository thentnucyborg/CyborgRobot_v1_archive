#include "ros/ros.h"
#include "std_msgs/String.h"
#include <k2_client/k2_client.h>
#include <k2_client/BodyArray.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <limits.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <string>
#include <stdio.h>
#ifdef WINDOWS
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
#endif

char cCurrentPath[FILENAME_MAX];

using namespace std;

void bodies_sub_cb(const k2_client::BodyArray msg){
  string current(cCurrentPath);
  std::string path = current + "/src/eit/src/take_a_selfie2.py"; // TODO write to resource file?
  for(int i = 0; i < 6; i++){
    if(msg.bodies[i].jointPositions[0].position.z < 1.5 && msg.bodies[i].jointPositions[0].position.z > 0.0){
        //Ask for selfie
	std::string cdOut = "cd ../../";
	system(cdOut.c_str());
	//std::string filename = "~/catkin_ws/src/eit/src/SelfieScript.py";
	std::string command = "python ";
	command += path;
	system(command.c_str());
        ROS_INFO("%f", msg.bodies[i].jointPositions[0].position.z);
        return;
    }
  }
  ROS_INFO("No close bodies...");
}


int main(int argc,char **argv){
  if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))) {
      return errno;
  }
  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';

  ros::init(argc,argv,"selfieNode");
  ros::NodeHandle n;

  // Subsribe to topic "bodyArray" published by k2_klient package node startBody.cpp
  ros::Subscriber bodies_sub = n.subscribe("head/kinect2/bodyArray", 1, bodies_sub_cb);

  ros::Rate loop_rate(0.5);

  ROS_INFO_NAMED("selfieNode", "selfieNode: Running ROS node...");
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_NAMED("selfieNode",  "selfieNode: Quitting... \n" );

  return 0;
}

