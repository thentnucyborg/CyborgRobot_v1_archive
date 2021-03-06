/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

#include "ros/ros.h"
#include "facerec/FaceRec.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
    std::ifstream file;
	file.open(filename.c_str(), ifstream::in);

    if (!file.is_open()) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}

bool face_rec(facerec::FaceRec::Request &req,facerec::FaceRec::Response &res) {

    string fn_csv = string(req.csv);
	
    // These vectors hold the images and corresponding labels.
    vector<Mat> images;
    vector<int> labels;
    // Read in the data. This can fail if no valid
    // input filename is given.
    try {
        read_csv(fn_csv, images, labels);
    } catch (cv::Exception& e) {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        // nothing more we can do
        exit(1);
    }
	

    int height = images[0].rows;

    Mat testSample = imread(req.pic,CV_BGR2GRAY);

    Ptr<FaceRecognizer> model = createLBPHFaceRecognizer();
    model->train(images, labels);
	
	model->set("threshold", 40);

    res.predictedLabel = model->predict(testSample);

	ROS_INFO("Sending back response: [%ld]", (long int)res.predictedLabel);
    
	return true;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "facerec_server");
	ros::NodeHandle n;
 
	ros::ServiceServer service = n.advertiseService("face_rec", face_rec);
	ROS_INFO("Ready to detect faces.");
	ros::spin();

    return 0;
}
