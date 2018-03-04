#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
import rospy
from std_msgs.msg import String
from kite_ros.msg import Kitepos
from message_converter import convert_ros_message_to_dictionary
from kiteb_process import proc_arduino
from kiteb_output import pub_base_msg


def call_arduino(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    message = convert_ros_message_to_dictionary(data)

    answer=proc_arduino(message)

    #will then call something in kiteb_process with message and should
    #return an answer as dictionary
    #answer=proc_arduino(message)

    pub_base_msg(answer)


def call_kite_infer(data):
    #message = msg_to_dict(data)
    message={}
    message['timestamp'] = data.header.stamp
    message['rleft'] = data.rleft
    message['msgkiteangle'] = data.kiteangle
    message['msgdirx'] = data.dirx
    message['msgdiry'] = data.diry
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    return message

def call_manual():
    #message = msg_to_dict(data)
    message={}
    message['timestamp'] = ''
    message['rleft'] = 100
    message['rcent'] = 200
    message['rright'] = 300
    message['heading'] = 10
    message['varx'] = 150
    message['vary'] = 250
    message['varz'] = 350

    rospy.loginfo(rospy.get_caller_id() + "I heard manual data")
    return message

#kite_arduino_msg.header.stamp = nh.now();
#kite_arduino_msg.rleft = analogRead(2);
#kite_arduino_msg.rcent = analogRead(0);
#kite_arduino_msg.rright = analogRead(3);
#kite_arduino_msg.heading = head;

#kite_arduino_msg.varx = mag[X]+800;
#kite_arduino_msg.vary = yval;
#kite_arduino_msg.varz = mag[Z]+800;