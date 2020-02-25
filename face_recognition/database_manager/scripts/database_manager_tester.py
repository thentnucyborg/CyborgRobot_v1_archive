#!/usr/bin/env python

##
## THIS CODE IS NOT MEANT TO BE USED!
##
##      This code was used for some testing, but we have decided to leave it here,
##          because it might be useful as example code for interatcting with the
##          database_manager_server
##

import sys
import rospy
from database_manager.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('add_person_to_database')
    try:
        add_person = rospy.ServiceProxy('add_person_to_database', StorePerson)
        img_path = "/home/charlie/eit/orl_faces/s1/1.pgm"
        respons = add_person(img_path, "Jan0", "Jeans0")
        print respons.status

        img_path = "/home/charlie/eit/orl_faces/s1/2.pgm"
        respons = add_person(img_path, "Jan1", "Jeans0")
        print respons.status

        img_path = "/home/charlie/eit/orl_faces/s2/1.pgm"
        respons = add_person(img_path, "Jan1", "Jeans2")
        print respons.status


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.wait_for_service('forget_person_by_id')
    try:
        forget_person = rospy.ServiceProxy('forget_person_by_id', ForgetPersonByID)
        respons = forget_person(1)
        print respons.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    
    rospy.wait_for_service('add_picture_to_existing_person')
    try:
        add_picture = rospy.ServiceProxy('add_picture_to_existing_person', AddPictureOfPerson)
        img_path = "/home/charlie/eit/orl_faces/s2/2.pgm"
        respons = add_picture(img_path, 2)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
