#!/usr/bin/env python
import os
from os import path

import rospy
from std_msgs.msg import String

import speech_recognition

import requests
import ast

import random
import time


def wit_ai(filename):

	data_1 = open(filename)
	data_opened = data_1.read()
	data_1.close()

	header = {"Authorization": "Bearer 5E7RXG6EAQPJW4E7TSZTEGGEWH6WAJCP",'Content-type': 'audio/wav'} 

	r = requests.post("https://api.wit.ai/speech?v=20160215",data=data_opened,headers = header)
	
	response = r.content

	person_said = ""

	try:
		person_said = ast.literal_eval(response)["outcomes"][0]["_text"]
		print person_said

	except:
		print "Couldnt recognize speech"
	return person_said

def recognize_with_google(filename,language):

	languages_supported = ["en-US","no-NO"]
	language_to_apply=""

	if language not in languages_supported:
		language_to_apply = "en-US"
	else:
		language_to_apply = language

	recognizer = speech_recognition.Recognizer()
	
	path_to_wav_source= os.getcwd()
	path_to_wav_source+="/%s"%(filename)

	with speech_recognition.WavFile(path.join(path.dirname(path.realpath(path_to_wav_source)),filename)) as speech_to_interpret:
		WAV_interpretor = recognizer.record(speech_to_interpret)

	try:
		response = recognizer.recognize_google(WAV_interpretor,key = None, language=language_to_apply,show_all = False)
		print"Google: \n" + response
		return str(response)

	except:
	
		response = recognizer.recognize_google(WAV_interpretor,key = "AIzaSyDn-8swRvG5-WnNONOpAQ4fv4fnzuvLyaY", language=language_to_apply,show_all = False)
		print"Google_private_key: \n" + response

		return response
		
def perform_speech_to_text(filename,language):
	try: 
		response = recognize_with_google(filename,language)
		return response
	except:
		response = wit_ai(filename)	
		return response

def record():
	while(1):
		for i in range(0,20):
			#p = os.system('sox -r 16000 -t alsa default detected_sound%s.wav silence 1 0.1 1% 1 1.5 1%'%(str(i)))
			p = os.system('sox -r 16000 -t alsa default detected_sound%s.wav %s'%(str(i),'silence 1 0.1 1% 1 1.5 1%'))
		#update_node(without_stream("sound_1.wav"))
		
		for i in range(0,7):
			#randclip = random.randint(0,19)
			for rand_clip in range(0,19):
				update_node(perform_speech_to_text("detected_sound%s.wav"%(str(rand_clip)),"en-US"))
				time.sleep(0.8)

def update_node(person_said):
	publish_sound = rospy.Publisher("text_from_speech",String, queue_size = 10)
	rospy.init_node("text_talker",anonymous=True)
	rate = rospy.Rate(10)
	#while not rospy.is_shutdown():
	str_to_send = person_said
	#rospy.loginfo(str_to_send)
	print "Broadcasting: ______"+str_to_send+"_________"
	publish_sound.publish(str_to_send)
	#rate.sleep()


if __name__ == '__main__':
    try:
        #update_node()
        #try_sphinx()
        record()
    except rospy.ROSInterruptException:
        pass





