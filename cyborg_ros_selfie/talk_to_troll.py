#!/usr/bin/env python

import rospy
import random

try:
	from trollnode.msg import Expression
except:
	pass
	#from std_msgs.msg import String


pub = ""
def talker(speech, express):

    if express == "": express = talk_random_expression()
    #speak(speech, express)

    expr_msg = Expression()
    expr_msg.expression = express
    expr_msg.speech = speech

    rospy.loginfo(speech +", "+express)

    pub.publish(expr_msg)
    #pub.publish(speech +", "+express)


def talk_random_expression():
    expr = ["happy", "angry", "smile", "sad", "disgust", "surprise", "fear", "suspicios",
        "blink", "pain", "duckface"]
    #talker(speech, expr[random.randint(0, len(expr)-1)])
    return random.choice(expr)

#def talk_default_expression(speech):
#   talker(speech, "happy")


def createPub():
    try:
        global pub
        #pub = rospy.Publisher('chatter', String, queue_size=10)
        pub = rospy.Publisher('trollExpression', Expression, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        return True
    
        #talker(0)#rospy.myargv(argv=sys.argv)[0])
    except:
        return False
