#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up

# this will publish final mesage back into ROS framweork and then routings would calc what
# to do next this message will be the actual angles - may well be similar format for desired angles
# but that will come later

import rospy
from kite_ros.msg import Kitebase
from message_converter import convert_dictionary_to_ros_message


# This publishes from the answer dictionary a new ros message based on the conversion of
# the arduino resistor values into an angle for the bar and some other possible tension and angle info that
# may at some point prove useful

# we run this and subscribe to test we have end to end process
# there will be anothe message kitbase and then basic motion detection picks that up to display the actual barangle

# message should be barangle plus estimated force and perhaps any useful output from compass gyroscope
# this displays on basic motion detection and in theory we then have full capture and should
# move to calibrate the bar angle part in proc_arduino and then fly the thing again


def pub_base_msg(answer):
    pub = rospy.Publisher('kitebase_node', Kitebase, queue_size=10)
    #rospy.init_node('kitebase_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    #msg = convert_dictionary_to_ros_message(Kitebase, answer)
    msg = Kitebase()
    msg.forceleft = answer['forceleft']
    msg.forceright = answer['forceright']
    msg.barangle = answer['barangle']
    #msg.posx = posx
    #msg.posy = posy
    #msg.kiteangle = kiteangle
    #msg.dirx = dirx
    #msg.diry = diry
    rospy.loginfo(msg)
    pub.publish(msg)
    return


if __name__ == '__main__':
    # talker()
    try:
        test_answer={'forceleft':10, 'forceright':30, 'barangle':35}
        pub_base_msg(test_answer)
    except rospy.ROSInterruptException:
        pass