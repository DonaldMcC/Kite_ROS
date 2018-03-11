#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
import rospy
from message_converter import convert_ros_message_to_dictionary
from kiteb_process import proc_arduino
from kiteb_output import pub_base_msg


def call_arduino(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    message = convert_ros_message_to_dictionary(data)

    answer=proc_arduino(message)

    pub_base_msg(answer)

    return


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
