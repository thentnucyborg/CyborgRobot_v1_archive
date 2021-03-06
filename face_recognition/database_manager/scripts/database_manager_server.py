#!/usr/bin/env python

# -*- coding: utf-8 -*-
from database_manager.srv import *
import sys
import os
import time
import shutil
import rospy
import webbrowser

# Path to the root folder in the database. Please set this to where you want
# your databse.
base_path = "/path/to/database"

# A pointer to an instance of the Database_manager class. Is set when this
# script runs.
dbm = 0

#
# A class for the info that gets stored in the info.txt files.
# Serves as an abstraction for the Database_manager class, so it does not have
# to worry to much about the deatils.
#
class Person:

    def __init__(self, first_name, last_name, timestamp=0):
        self.first_name = first_name
        self.last_name = last_name
        if timestamp==0:
            self.timestamp = time.time()
        else:
            self.timestamp = timestamp

    # Use to_string to get the text that should be written to info.txt.
    def to_string(self):
        out = "first_name "+self.first_name+"\nlast_name "+self.last_name+"\ntimestamp "+str(self.timestamp)
        return out

    def update_timestamp(self):
        self.timestamp = time.time()


#
# The class that takes care of organizing the database.
# This class adds and removes persons and pictures of persons. It updates the
# .csv file after making changes to the database.
class Database_manager:
    
    # Used as the default age limit. I.e. if the timestamp of a person is older
    # than this limit, that person should be deleted from the database. The
    # value is in seconds. 62208000 seconds = 2 years
    n_DEFAULT_AGE_LIMIT = 62208000
    
    def __init__(self, base_path, age_limit=n_DEFAULT_AGE_LIMIT):
        # The default maximum limit for how many images will be stored for each
        # person.
        n_IMAGE_LIMIT = 50

        self.accepted_image_formats = ["jpg", "jpeg", "png", "pgm"]

        # The root folder for the database - "/path/to/database"
        self.base_path = base_path
        self.age_limit = age_limit

        # Find used ids, to easily find available ids later
        self.find_used_ids()

        # The next available id
        self.next_available_id = 0
        self.find_next_available_id()

        # The limit for how many images gets stored for each person
        self.image_limit_per_person = n_IMAGE_LIMIT

    # Look through the database to see which IDs are taken
    def find_used_ids(self):
        self.used_ids = []

        # Find all the ids that have been used
        for dirname, dirnames, filenames in os.walk(self.base_path):
            for subdirname in dirnames:
                # Get the number from the directoryname
                i = int(subdirname[1:])
                self.used_ids.append(i)
        
        # Sort the list
        self.used_ids.sort()

    # Looks through the list of used IDs to find an available one
    def find_next_available_id(self):
        while self.next_available_id in self.used_ids:
            self.next_available_id = self.next_available_id + 1

    # Go through the database and check the age of entries, delete them if they
    # are too old
    def forget_expired_entries(self):

        # Find old guys
        old_ids = []
        for i in self.used_ids:
            file_name = self.base_path + "/s" + str(i) + "/info.txt"
            info_file = open(file_name,'r')
            for line in info_file:
                tmp = line.split(' ', 1)
                if tmp[0] == "timestamp":
                    if self.age_limit < time.time() - float(tmp[1]):
                        old_ids.append(i)
                        break

        # Forget the old guys
        for i in old_ids:
            self.forget_person_by_id(i)

        return "Old entries deleted."

    # Find the person with correct id and remove him/her
    def forget_person_by_id(self, person_id):
        file_name = self.base_path + "/s" + str(person_id)
        shutil.rmtree(file_name)
        self.used_ids.remove(person_id)
        return "Person deleted from database."

    # Find all people witht the given name and remove them. Might find an
    # arbitrary number of people (but hopefully no more than one) in the
    # database. So that should be handled.
    def forget_person_by_name(self, first_name, last_name):
        # TODO
        print "forget_person_by_name() is not implemented. Nothing happened"
        pass

    # Takes an image and a Person object that contains the necessary
    # information about the person and stores it in the database
    def store_person(self, img, person):
        new_directory = self.base_path + "/s" + str(self.next_available_id)
        if not os.path.exists(new_directory):
            os.chmod(self.base_path, 0777)
            os.makedirs(new_directory)
            # The returned image number should be 0, since this directory
            # should be empty
            img_number = self.get_next_available_img_number(new_directory)
            if img_number==-1:
                return "Unexpected error. Created a new directory for the new person, but the new directory seems to be filled with images already."
            img_format = img.split('.', 1)[1]
            if img_format not in self.accepted_image_formats:
                return img+" is not in an accepted image format"
            new_filename = new_directory+"/"+str(img_number)+"."+img_format
            with open(img, 'rb') as f:
                data = f.read()

            with open(new_filename, 'wb') as f:
                f.write(data)
            
            info_file = new_directory+"/info.txt"
            with open(info_file, 'w') as f:
                f.write(person.to_string())

            self.used_ids.append(self.next_available_id)
            self.find_next_available_id()
            self.create_csv()

            # Keep the database fresh. Remove people that have not been seen in a
            # long time.
            self.forget_expired_entries()

            return "New person successfully added to the database."

        else:
            return "[database_manager.py].[store_person] tried to make a new directory: (", new_directory, ") but this directory already exists" 
    
    
    def get_person_by_id(self, pid):
        person_dir = self.base_path + "/s" + str(pid)
        # Check that the directory already exists
        if os.path.exists(person_dir):
            filename = person_dir+"/info.txt"
            with open(filename, 'r') as f:
                data = f.readlines()
        
            first_name = ""
            last_name = ""
            timestamp = time.time()
        
            for line in data:
                tmp = line.split(' ', 1)
                if tmp[0] == "first_name":
                    first_name = tmp[1]
                elif tmp[0] == "last_name":
                    last_name = tmp[1]
                elif tmp[0] == "timestamp":
                    timestamp = float(tmp[1])

            return first_name[:-1]+" "+last_name
        else:
            return "Could not find directory"

    # Finds an image of person with ID pid, and shows it.
    # Only used for demonstration purpose. Should probably be implemented in
    # another way (and by another code, just use the .csv file) for the NTNU
    # Cyborg
    def show_image(self, pid):
        file_name = self.base_path + "/labels.csv"
        SEPARATOR = ";"

	data = []
        with open(file_name, 'r') as f:
            data = f.readlines()

        for line in data:
            tmp = line.split(";", 1)
            if int(tmp[1]) == pid:
                webbrowser.open(tmp[0])
                return "Image should now be opened in your webbrowser"

        return "Could not find any image of the person with ID="+str(pid)
         

    # Adds a copy of the image with path=img to the directory of the person
    # with id=pid (label)
    def add_picture_of_person(self, img, pid):
        # The directory to put the new image in
        person_dir = self.base_path + "/s" + str(pid)
        # Check that the directory already exists
        if os.path.exists(person_dir):
            # Check that we won't exceed the image limit by adding another
            # image
            img_number = self.get_next_available_img_number(person_dir)
            if img_number==-1:
                return "Maximum number of images limit reached for this person, image was not added."
            img_format = img.split('.', 1)[1]
            if img_format not in self.accepted_image_formats:
                return img+"is not in an accepted image format"
            new_filename = person_dir+"/"+str(img_number)+"."+img_format
            with open(img, 'rb') as f:
                data = f.read()

            with open(new_filename, 'wb') as f:
                f.write(data)

            self.update_timestamp(pid)
            self.create_csv()
            
            return "Image of person successfully added to person."
        else:
            return "There are no persons in the database with id: "+str(pid)

    # Updates the timestamp for when the person with id 'pid' was last
    # interacted with to the current time
    def update_timestamp(self, pid):
        file_name = self.base_path + "/s"+ str(pid) + "/info.txt"
        with open(file_name, 'r') as f:
            data = f.readlines()

        out_file = open(file_name, 'w')
        timestamp_printed = False
        for line in data:
            tmp = line.split(' ', 1)
            if tmp[0] == "timestamp":
                new_line = "timestamp "+str(time.time())+"\n"
                out_file.write(new_line)
                timestamp_printed = True
            else:
                out_file.write(line)
        if not timestamp_printed:
            new_line = "timestamp "+str(time.time())+"\n"
            out_file.write(new_line)

    # Creates a new/updated csv file for the database. This is a modified
    # version of the script found here:
    # http://docs.opencv.org/2.4/modules/contrib/doc/facerec/facerec_tutorial.html#appendixft
    def create_csv(self):
        file_name = self.base_path + "/labels.csv"
        csv_file = open(file_name, "w")
        BASE_PATH = self.base_path
        SEPARATOR = ";"

        label = 0
        for dirname, dirnames, filenames in os.walk(BASE_PATH):
            for subdirname in dirnames:
                subject_path = os.path.join(dirname, subdirname)
                # use this number from the directory as the label
                label = int(subdirname[1:])
                for filename in os.listdir(subject_path):
                    if filename != "info.txt" and filename != "info.txt~":
                        abs_path = "%s/%s" % (subject_path, filename)
                        csv_file.write("%s%s%d\n" % (abs_path, SEPARATOR, label))

        csv_file.close()

    def get_next_available_img_number(self, dir_path):
        used_numbers = []
        for filename in os.listdir(dir_path):
            tmp = filename.split('.', 1)
            if tmp[1] in self.accepted_image_formats:
                used_numbers.append(int(tmp[0]))
        for i in range(0, self.image_limit_per_person):
            if i not in used_numbers:
                return i
        print "Maximum number of images ("+str(image_limit_per_person)+") reached for this person. "+directory
        return -1

# Takes an image(path to file), first name and last name. Creates a new person
# in the database with this initial info. The image is copied from its location
# and stored in the database.
def handle_add_person_to_database(req):
    person = Person(req.first_name, req.last_name)
    status = dbm.store_person(req.imagepath, person)
    return status

# Adds a picture of a person to the database. Takes in the image path and the
# id of the person
def handle_add_picture_to_existing_person(req):
    status = dbm.add_picture_of_person(req.imagepath, req.pid)
    return status

# Finds persons in the database that have not been updated in a long time, i.e.
# older than the specified number of seconds in the age_limit parameter in the
# Database_manager constructor. Defaults to six months. Needs no parameters.
def handle_forget_expired_entries(req):
    status = dbm.forget_expired_entries()
    return status

# Takes an id as parmeter and deletes the entry for the person with that id
def handle_forget_person_by_id(req):
    status = dbm.forget_person_by_id(req.pid)
    return status

def handle_update_timestamp(req):
    status = dbm.update_timestamp(req.pid)
    return status

def handle_get_person_name(req):
    status = dbm.get_person_by_id(req.pid)
    return status

def handle_show_image(req):
    status = dbm.show_image(req.pid)
    return status

def main():
    
    global base_path

    # Check that a path to the database has been set
    if base_path=="/path/to/database":
        # base_path has not been set, just use the current working directory
        base_path = os.getcwd()
        base_path = base_path +"/database"
    
    # Check that the set path is an existing directory, if not -> create it
    if not os.path.exists(base_path):
        os.makedirs(base_path)
    print "Using database located at " + base_path

    # Create the database manager
    global dbm
    dbm = Database_manager(base_path)

    # Create a csv based on the existing content in the database
    dbm.create_csv()
    print "Created fresh .csv file"

    rospy.init_node('face_database_manager')
    s1 = rospy.Service('add_person_to_database', StorePerson, handle_add_person_to_database)
    s2 = rospy.Service('add_picture_to_existing_person', AddPictureOfPerson, handle_add_picture_to_existing_person)
    s3 = rospy.Service('forget_expired_entries', ForgetExpiredEntries, handle_forget_expired_entries)
    s4 = rospy.Service('forget_person_by_id', ForgetPersonByID, handle_forget_person_by_id)
    s5 = rospy.Service('update_timestamp', UpdateTimestamp, handle_update_timestamp)
    s6 = rospy.Service('get_person_name', GetPersonName, handle_get_person_name)
    s7 = rospy.Service('show_image', ShowImage, handle_show_image)

    print "DatabaseManager service is ready."
    rospy.spin()

if __name__ == "__main__":
    main()
