#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
import rospy
from std_msgs.msg import String
from kite_ros.msg import Kitepos

params = False
def call_arduino(data):
    message={}
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    message['msgname'] = data.name
    message['msgposx'] = data.posx
    msgposy = data.posy
    msgkiteangle = data.kiteangle
    msgdirx = data.dirx
    msgdiry = data.diry

    #will then call something in kiteb_process with message and should
    #return an answer as dictionary

    #that then calls kiteb_output


    # calc angles
    # publish converted results

def call_kite_infer(data):
    message={}
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    msgname = data.name
    msgposx = data.posx
    msgposy = data.posy
    msgkiteangle = data.kiteangle
    msgdirx = data.dirx
    msgdiry = data.diry
    # no params for kite_infer - just need to work out from position??
    return message

