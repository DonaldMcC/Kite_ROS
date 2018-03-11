#!/usr/bin/env python
# kitebar will be an entry point for routine to initially convert the values from
# arduino resistors and publish the kitebar angles and force - in time it should however support other modes
# where the kitebar angles are inferred from the kite or directly controlled by keyboard or other controls
# there should always be some sort of input, some processing and publishing of output message with the angle and
# estimeated forces on the bar

# it is proposed to structure this over 4 mddules
# kiteb_input will handle reading ROS messages and convert into dictionaries most likely to transfer to
# kiteb_process which should do the conversion of the data to get the result
# kiteb_ouput will publish the ROS message
# this module will read parameters and establish the operating mode and then call relevant input, conversion and
# output functions or classes


import rospy
from kite_ros.msg import Kitepos
from kiteb_input import call_arduino
from kitetb_params import get_params, params

def kitebar(source):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('kitebar')

    if source == 'arduino':
        rospy.Subscriber("kite_arduino", Kitepos, call_arduino)
        # think everything else can happen in call arduino after the

    elif source == 'kite_infer':
        rospy.Subscriber("kite_arduino", Kitepos, call_arduino)
        # get kitepos
        # calc angles from kitepos
        # publish converted results

    elif source == 'manual':
        pass
        # get previous angle or 0
        # process keyboard or joystick input source
        # publish updated results

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def startnode(source='arduino'):
    # startup should attempt to read from ROS parameter server - but if not there it
    # should start with default source value objective is that the angle of the bar
    # and tension on the strings can be published based on 3 options
    # 1 arduino - based on the readings from the arduino resistors and we will calibrate
    # as parameters which are used to convert the resistance readings into the angle of the bar
    # and the rough tension on the kitelines

    # retrieve the source from param server or set it to the source if not set
    global params
    params = get_params()
    kitebar(params['source'])
    return


if __name__ == '__main__':
    startnode()
