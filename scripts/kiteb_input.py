#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
import rospy
from std_msgs.msg import String
from kite_ros.msg import Kitepos


def msg_to_dict(data):
    message={}
    message['msgname'] = data.name
    message['msgposx'] = data.posx
    message['msgposy'] = data.posy
    message['msgkiteangle'] = data.kiteangle
    message['msgdirx'] = data.dirx
    message['msgdiry'] = data.diry
    return message

params = False
def call_arduino(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    message = msg_to_dict(data)

    #will then call something in kiteb_process with message and should
    #return an answer as dictionary

    #that then calls kiteb_output
    # calc angles
    # publish converted results


def call_kite_infer(data):
    message = msg_to_dict(data)
    message={}
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    return message

