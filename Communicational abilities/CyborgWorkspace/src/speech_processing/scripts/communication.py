#!/usr/bin/env python

import os
import sys
import time	
import random

import requests 								#Requires installation				
import json                 					#Requires installation 

import rospy     								#Requires installation														
from std_msgs.msg import String							
from speech_processing.msg import Expression	

import cleverbot 								#Requires installation				
from cleverbot import Cleverbot 				#Requires installation

from profanity import profanity   				#Requires installation

response = ""

#Load the question bank. This is only executed in when starting up the program. 						 					
def load_question_bank():
	question_bank = {}
	try:
		f = open("wordbank.txt","r")                       
		
		for line in f.readlines():
			question = line.split(":::")[0]
			answer = line.split(":::")[1].split("\n")[0]
			question_bank[question] = answer 

		f.close()
	except:
		print "not able to load wordbank"

	return question_bank 

#Update the ROS topic sending the information to the trollface software.
def update_troll(text_reply):

	publish_interpretion = rospy.Publisher("trollExpression",Expression, queue_size = 100)

	expr_msg = Expression()
	expr_msg.speech = text_reply["reply"]
	expr_msg.expression = get_facial_expression(text_reply["mode"])
	
	publish_interpretion.publish(expr_msg)

#Maps the generated behavioural mode with the corresponding facial expression on the trollface
def get_facial_expression(mode):
	default_expression = "smile"

	facial_expressions = {}
	facial_expressions["report_mode"] 		= 	"neutral"
	facial_expressions["jokes_mode"] 		= 	"surprise"
	facial_expressions["repeating_mode"] 	= 	"neutral"
	facial_expressions["cleverbot_mode"] 	= 	"smile"
	facial_expressions["abusive_mode"]		= 	"angry"
	facial_expressions["tired_mode"] 		= 	"pain"
	facial_expressions["lunch_mode"] 		= 	"blink"
	facial_expressions["selfie_mode"] 		= 	"duckface"
	facial_expressions["command_mode"] 	= 	"suspicious"

	if mode in facial_expressions.keys():
		return facial_expressions[mode]
	else: 
		return default_expression

#Updates the question-bank with the question and corresponding answer to make the NTNU Cyborg learn from earlier conversations. 
def update_question_bank(content):

	publish_to_question_bank = rospy.Publisher("question_bank_stream",String, queue_size = 100)

	publish_to_question_bank.publish(content)

#Callback function being called when valid speech is detected.
def callback(data,args):
	rospy.loginfo(data)

	abusive_jokes = args[0]
  	cyborg_jokes = args[1]
  	question_bank = args[2]

  	cyborgs_reply = {}

  	try:
		sentence_from_speech = str(data).split(": ")[1]
		if(not check_for_speech_in_string(sentence_from_speech)):

			if sentence_from_speech in question_bank.keys():
				cyborgs_reply = {"reply":question_bank[sentence_from_speech],"mode":"cleverbot_mode"}
				
			else:
				cyborgs_reply = generate_cyborgs_reply(sentence_from_speech,abusive_jokes,cyborg_jokes)


			print "Machine: "+cyborgs_reply["reply"]


			update_troll(cyborgs_reply)

			question_bank_data = sentence_from_speech
			question_bank_data+=":::"
			question_bank_data+= cyborgs_reply["reply"]
			update_question_bank(question_bank_data)

			question_bank[sentence_from_speech] = cyborgs_reply["reply"]

	except:
		print "Error in attempting to generate cyborgs_reply"
	 	pass

#Checks if the text string containing speech is valid or not	
def check_for_speech_in_string(input_string):
	return ('' in input_string and len(input_string)==2) 

#Contains ROS functionality for detecting speech. The ROS topic "text_from_speech" is pulished to by the module "speech_to_text"
def listen_for_conversation():

	rospy.init_node("listen_for_conversation",anonymous=True)
	
	jokes = init_jokes_databases()
	abusive_jokes = jokes[0]
	cyborg_jokes = jokes[1]
	question_bank_dict = load_question_bank()

	print "wordbank:"
  	print question_bank_dict

	content = rospy.Subscriber("text_from_speech",String,callback,(abusive_jokes,cyborg_jokes,question_bank_dict))
	
	rospy.spin()

#Uses a cleverbot in Python to generate an answer from a question.
def generate_text_from_cleverbot(text_raw):
	cb = Cleverbot()
	try:

		cleverbot_reply = cb.ask(text_raw)
		return cleverbot_reply
	
	except:
		print "Cleverbot failed to reply"
		pass
	
	#API:
	#Cleverbot: https://pypi.python.org/pypi/cleverbot

def generate_answer_from_cleverbot_2(msg):
	
	apiKey= "0rR6tpz4t3llcskM"
	diff = "abglso-4582-sks"
	url = "http://www.personalityforge.com/api/chat/?apiKey=%s&chatBotID=%s&message=%s&externalID=%s"%(apiKey,6,msg,diff)
	response = json.loads(requests.get(url).content)["message"]["message"]
	return response

#Function returning the exactly same string as the argument. This is used in the "repeating mode" for the robot.
def repeat(text_raw):
	return text_raw

#Recognizes the word "selfie" from a sentence. Returns a bool value. 
def recognize_selfie(text_raw):
	try:
		if "selfie" in text_raw.lower():
			return True
		else:
			return False
	except:
		pass

#Returns a dictionary containing 200 NTNU Cyborg Jokes jokes. Based on Chuck Norris jokes, but the name in the jokes is changed to NTNU Cyborg.
def chuck_norris_database(): 

	#source of API:
	#http://www.icndb.com/api/
	
	#For NTNU Cyborg jokes:
	try:
		JSON_response = json.loads(requests.get("http://api.icndb.com/jokes/random/200?firstName=NTNU&lastName=Cyborg&escape=html&escape=javascript").content)
		

		#Extract the information not needed from the JSON response and insert only the useful information in a dictionary. 
		chuck_dict = {}

		for i in range(0,len(JSON_response["value"])):
			chuck_dict[i] = JSON_response["value"][i]["joke"]

		return chuck_dict

	except:
		pass

#Removes unwanted content of the Yo-moma API as it contains unwanted content
def convert_joke(joke):
	words_to_change = ["mamma","mama","momma","mommas","mama's","mamma's","mamas"]
	lowered_joke = joke.lower()

	final_joke =""
	try:
		list_of_words = lowered_joke.split(" ")
		if(list_of_words[1] in words_to_change):
			list_of_words[1]="mother"
			list_of_words[0]="Your"

		
		for word in list_of_words:
			final_joke+=word
			final_joke+=" "

	except:
		pass

	return final_joke

#Returns an abusive joke about the persons mother.
def get_abusive_joke_database():
	#http://yomomma.info/
	abusive_jokes = {}
	for joke_number in range(0,50):
		JSON_response = str(json.loads(requests.get("http://api.yomomma.info").content)["joke"])

		abusive_joke = convert_joke(JSON_response)

		abusive_jokes[joke_number] = abusive_joke
	
	return abusive_jokes

def get_random_abusive_joke(jokes_database):
	random_number = random.randint(0,49)
	return jokes_database[random_number]

#Checks if a text string contains bad words. Returns a bool value
#Based on: https://pypi.python.org/pypi/profanity/1.1

def check_if_containing_bad_words(sentence):
	try:
		is_offensive = profanity.contains_profanity(sentence)
		return is_offensive

	except:
		pass

	return False

#Function to provide a weather report. Returns a string reporting about the wather inclusive wind conditions.
def weather_forecaster(text_raw,weather_mode):
	
	#API:
	#http://openweathermap.org/current
	api_key = "98ff91c2af69dcdea4b8fd540883ed98"
	
	if "weather" in text_raw.lower() or weather_mode == True:
		try:
			weather = requests.get("http://api.openweathermap.org/data/2.5/weather?q=Trondheim,no&appid=%s"%(api_key))
			weather_dict = json.loads(weather.content)
			weather_description = str(weather_dict["weather"][0]["description"])
			wind_conditions = weather_dict["wind"]
			wind_speed = str(wind_conditions["speed"])
			
			weather_report_of_today = "Can report of "+weather_description+" in Trondheim today. Wind speed is "+wind_speed+" meters per second."
			return weather_report_of_today
		except:
			pass
	
	else:
		return ""

#Function that takes a sentence containing a bad word and filters the bad words with the word "heaven". Based on the Python library profanity.
def return_censored_sentence(sentence):
	word_to_exchange_with_bad_word = "heaven"
	censored_sentence = ""

	try:
		profanity.set_censor_characters("*")
		censored_string = profanity.censor(sentence)

		words_in_censored_string = censored_string.split(" ")
		for i in range(0,len(words_in_censored_string)):
			if "*" in words_in_censored_string[i]:

				words_in_censored_string[i] = ""

		for word in words_in_censored_string:
			censored_sentence+=word 
			censored_sentence+= " "

		print censored_sentence
	
	except:
		pass

	return sentence
	
#Returns the current hour of the day as a string.
def return_time_of_the_day():
	return int(time.strftime("%H"))

#Function making the robot refusing to talk before 9AM as it is too early and NTNU Cyborg is tired.
def morning_angry(): 
	if return_time_of_the_day()<9:
		return "It is damn early. Give me a coffee and I will reply."
	else:
		return "negative"

#Makes NTNU Cyborg being not able to reply from 12-13 PM due to lunch.
def lunch_angry():
	if return_time_of_the_day()==12:
		return "I am having lunch. Do not interupt me."
	else:
		return "negative"

#Used to search for commands. If a command is detected, then the robot should perform this action.
def move_on_command(text_raw):
	if "left" in text_raw and "move" in text_raw:
		return "Moving left"
	elif "right" in text_raw and "move" in text_raw:
		return "Moving right"
	elif "reverse" in text_raw and "move" in text_raw:
		return "Moving in reverse"
	elif "stop" in text_raw:
		return "Full stop"

	return "commands_not_detected"

def init_jokes_databases():
	
	jokes_database = []
	
	print "________________INIT______________\n"
	jokes_database.append(get_abusive_joke_database())
	jokes_database.append(chuck_norris_database())
	print "________________INIT SUCCESSFUL____________\n"
	
	return jokes_database

#Function only used for testing.
def testing_function():

	print "________________CALIB______________\n"
	abusive_jokes_database =  get_abusive_joke_database()
	cyborg_jokes_database =  chuck_norris_database()
	print "________________CALIB SUCCESSFULL____________\n"

	while(1):
		text_to_reply = raw_input("Write a question: ")
		
		cyborgs_reply = generate_cyborgs_reply(text_to_reply,abusive_jokes_database,cyborg_jokes_database)

		print cyborgs_reply

#Main function generating Cyborgs mode and reply. It first checks for six special modes. These are:
#-> If it is too early for the robot to reply
#-> If it has lunch
#-> If the keyword weather is detected in the sentece so that NTNU Cyborg will provide a weather report
#-> Checks for bad words in the sentence. Enters an abusive mode if bad words are detected.
#-> If commands are detected, it goes into command_mode. Commands are to be performed.
#-> If selfie is detected, go to selfie mode

#If noen of these special cases are detected, then a mode is calculated randomly, but distributed in the following way:
# Cleverbot mode 50%
# Joking mode 20%
# Weather-report mode 10%
# Abusive mode 10%
# Repeating mode 10%

def check_for_special_modes(text_to_reply):
	cyborgs_reply = {}

	if morning_angry()!= "negative":
			cyborgs_reply["reply"]=morning_angry()
			cyborgs_reply["mode"] = "tired_mode"
			return cyborgs_reply

	elif lunch_angry()!= "negative":
		cyborgs_reply["reply"]=lunch_angry()
		cyborgs_reply["mode"] = "lunch_mode"
		return cyborgs_reply

	elif "weather" in text_to_reply.lower():
		cyborgs_reply["reply"]=weather_forecaster(text_to_reply,False)
		cyborgs_reply["mode"] = "report_mode"
		return cyborgs_reply

	elif check_if_containing_bad_words(text_to_reply)==True: 
		cyborgs_reply["reply"]="When you say a bad thing to me I want revenge. "
		cyborgs_reply["reply"]+=get_random_abusive_joke(abusive_jokes_database)
		cyborgs_reply["mode"] = "abusive_mode"
		return cyborgs_reply

	elif move_on_command(text_to_reply)!="commands_not_detected": 
		cyborgs_reply["reply"]= move_on_command(text_to_reply)
		cyborgs_reply["mode"] = "command_mode"
		return cyborgs_reply

	elif(recognize_selfie(text_to_reply)==True):
		cyborgs_reply["reply"]="Lets have selfie together."
		cyborgs_reply["mode"] = "selfie_mode"
		return cyborgs_reply

	return cyborgs_reply

def generate_cyborgs_reply(text_to_reply,abusive_jokes_database,cyborg_jokes_database):
	cyborgs_reply = {}

	try:
		cyborgs_reply = check_for_special_modes(text_to_reply)

		if(len(cyborgs_reply)>0):
			pass

		else:
			mode = generate_cyborgs_mode_based_on_probability()

			reply = ""
				
			if(mode == "repeating_mode"):
				reply = repeat(text_to_reply)
			elif(mode == "cleverbot_mode"):
				#reply = generate_text_from_cleverbot(text_to_reply)
				reply = generate_answer_from_cleverbot_2(text_to_reply)
			elif(mode == "jokes_mode"):
				random_nr = random.randint(0,99)
				reply = cyborg_jokes_database[random_nr]
			elif(mode == "abusive_mode"):
				reply = get_random_abusive_joke(abusive_jokes_database)
			elif(mode == "report_mode"):
				reply = weather_forecaster(text_to_reply,True)

			else:
				reply = generate_text_from_cleverbot(text_to_reply)

			cyborgs_reply["reply"]=reply
			cyborgs_reply["mode"]=mode
		
	except:
		pass

	return cyborgs_reply

#Generates the correct mood for the NTNU Cyborg if none of the six special cases in the function above are not detected. 
def generate_cyborgs_mode_based_on_probability():
	modes_available = ["repeating_mode","cleverbot_mode","jokes_mode","abusive_mode","report_mode"]
	random_number = random.randint(0,9)

	if random_number>=0 and random_number<=4:
		mode_number = 1
	elif random_number >=5 and random_number <=6:
		mode_number = 2
	elif random_number == 7:
		mode_number = 4
	elif random_number == 8:
		mode_number = 3
	elif random_number == 9:
		mode_number = 0

	mode = modes_available[mode_number]

	return mode


if __name__ == '__main__':
	try:

		listen_for_conversation()
		 

	except rospy.ROSInterruptException:
		print "_______________________ERROR OCCURED____________________________"
		pass



