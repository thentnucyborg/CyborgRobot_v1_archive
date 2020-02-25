#!/usr/bin/env python

import rospy						#Requires installation
from std_msgs.msg import String		#Is included in the above library

#Callback function being called when the question-bank should be updated
def callback(data):
	rospy.loginfo(data)

	question_bank_data = str(data).split(": ")[1]

	write_to_file(question_bank_data)

	print question_bank_data

#Writes data to file
def write_to_file(data):
	data+="\n"
	
	try:
		f = open('wordbank.txt', 'a')
		f.write(data)
		f.close()
	
	except:
		print "not able to write to file"

#ROS function that subscribes to the topic containing data published to by the communication.py module.
def listen_for_new_question_bank_data():

	rospy.init_node("listen_for_wordbank_updates",anonymous=True)

	content = rospy.Subscriber("question_bank_stream",String,callback)

	rospy.spin()




if __name__ == '__main__':
	try:

		listen_for_new_question_bank_data()
		 
	except rospy.ROSInterruptException:
		print "_______________________ERROR OCCURED____________________________"




