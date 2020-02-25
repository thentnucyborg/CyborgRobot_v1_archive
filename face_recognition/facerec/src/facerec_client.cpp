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
#include "database_manager/StorePerson.h"
#include "database_manager/ShowImage.h"
#include "database_manager/GetPersonName.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>

using namespace cv;
using namespace std;

// Function Headers
void crop_face(Mat frame, string image_path);

// Global variables
// Copy this file from opencv/data/haarscascades to target folder
string face_cascade_name = "/home/charlie/Documents/OpenCV/opencv-2.4.12/data/haarcascades/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
string window_name = "Capture - Face detection";
int filenumber; // Number of file to be saved
string filename;


void add_person_in_database(const string& path){
	ros::NodeHandle n;
 
	ros::ServiceClient client = n.serviceClient<database_manager::StorePerson>("add_person_to_database");

	database_manager::StorePerson srv;

	srv.request.imagepath = path;

	ROS_INFO("Enter your first name?");
	string first_name, last_name;
	cin >> first_name;
 	srv.request.first_name = first_name;
	ROS_INFO("Enter your last name?");
	cin >> last_name;
	srv.request.last_name = last_name;

	if (client.call(srv))
	{
		ROS_INFO("Status: %s", srv.response.status.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service add_person_to_database");
	}
}

void show_image(int label){
	ros::NodeHandle n;
 
	ros::ServiceClient client = n.serviceClient<database_manager::ShowImage>("show_image");

	database_manager::ShowImage srv;

	srv.request.pid = label;

	if (client.call(srv))
	{
		ROS_INFO("Status: %s", srv.response.status.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service show image");
	}
}

void get_person_name(int label){
	ros::NodeHandle n;
 
	ros::ServiceClient client = n.serviceClient<database_manager::GetPersonName>("get_person_name");

	database_manager::GetPersonName srv;

	srv.request.pid = label;

	if (client.call(srv))
	{
		ROS_INFO("%s", srv.response.status.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service get person name");
	}
	
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "facerec_client");
	
	if (argc != 2){
		ROS_INFO("Usage: facerec_client picture.pgm");
	return -1;
	}
	
	string csv_file_path, image_path;
	csv_file_path = "/home/charlie/database/labels.csv";
	image_path = argv[1];

	// Crop the image
	// Load the cascade
	if (!face_cascade.load(face_cascade_name)){
	        printf("--(!)Error loading\n");
        	return (-1);
	}

	// Read the image file
	Mat frame = imread(image_path);

	// Apply the classifier to the frame
	if (!frame.empty()){
      		crop_face(frame,image_path);
    	}
    	else{
        	printf(" --(!) No captured frame -- Break!");
        	//break;
    	}
	int c = waitKey(10);

    	if (27 == char(c)){
        	//break;
    	}

	// Face recognition
	ros::NodeHandle n;
 
	ros::ServiceClient client = n.serviceClient<facerec::FaceRec>("face_rec");

	facerec::FaceRec srv;
 	srv.request.csv = csv_file_path;
	srv.request.pic = image_path;

	if (client.call(srv))
	{
		ROS_INFO("Predicted label: %ld", (long int)srv.response.predictedLabel);
	}
	else
	{
		ROS_ERROR("Failed to call service facerec");
		return -1;
	}
	if (srv.response.predictedLabel != -1){
		show_image(srv.response.predictedLabel);
		get_person_name(srv.response.predictedLabel);
		
		ROS_INFO("Is this you? (t/f)");
		string userinput;
		cin >> userinput;
		if (userinput == "t"){
			ROS_INFO("Wooho, we found you!!");
		}
		else{
			ROS_INFO("Can I add you as my friend? (t/f)");
			cin >> userinput;
			if (userinput == "t"){
				add_person_in_database(image_path);
				ROS_INFO("Wooho, I have now added you as my friend! :D");			
			}
			else{
				ROS_INFO("Buhu! :(");			
			}
		}
	}
	else 
	{
		ROS_INFO("Im sorry, I dont know you! Can I add you as my friend? (t/f)");
		string userinput;
		cin >> userinput;
		if (userinput == "t"){
			add_person_in_database(image_path);
			ROS_INFO("Wooho, I have now added you as my friend! :D");			
		}
		else{
			ROS_INFO("Buhu! :(");			
		}
	}
  return 0;
}


void crop_face(Mat frame,string image_path){
    std::vector<Rect> faces;
    Mat frame_gray;
    Mat crop;
    Mat res;
    Mat gray;
    string text;
    stringstream sstm;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    // Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    // Set Region of Interest
    cv::Rect roi_b;
    cv::Rect roi_c;

    size_t ic = 0; // ic is index of current element
    int ac = 0; // ac is area of current element

    size_t ib = 0; // ib is index of biggest element
    int ab = 0; // ab is area of biggest element

    for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)

    {
        roi_c.x = faces[ic].x;
        roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)

        roi_b.x = faces[ib].x;
        roi_b.y = faces[ib].y;
        roi_b.width = (faces[ib].width);
        roi_b.height = (faces[ib].height);

        ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }

        crop = frame(roi_b);
        resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR); // This will be needed later while saving images
        cvtColor(crop, gray, CV_BGR2GRAY); // Convert cropped image to Grayscale

        // Form a filename
        filename = "";
        stringstream ssfn;
        ssfn << filenumber << ".pgm";
        filename = ssfn.str();
        filenumber++;

        imwrite(image_path, gray);

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
        rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
 	}

    /*// Show image
    sstm << "Crop area size: " << roi_b.width << "x" << roi_b.height << " Filename: " << filename;
    text = sstm.str();

    putText(frame, text, cvPoint(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
    imshow("original", frame);

    if (!crop.empty())
    {
        imshow("detected", crop);
    }
    else
        destroyWindow("detected");*/
}

