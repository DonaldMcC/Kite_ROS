#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def kitebar(source):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('kitebar')


    if source == 'arduino':
        rospy.Subscriber("chatter", String, callback)
        # get params - may not call this all the time
        leftmax, leftmin, centremin, centremax, rightmax, rightmin = get_params()

        # calc angles
        # publish converted results
    elif source == 'kite_infer':
        pass
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

    #retrieve the source from param server or set it to the source if not set
    source = rospy.get_param('source', source)
    kitebar(source)


def get_params():
    leftmax = rospy.get_param('leftmax', 1000)
    leftmin = rospy.get_param('leftmin', 0)
    centremax = rospy.get_param('centremax', 1000)
    centremin = rospy.get_param('centremin', 0)
    rightmax = rospy.get_param('rightmax', 1000)
    rightmin = rospy.get_param('rightmin', 0)
    return leftmax, leftmin, centremax, centremin, rightmax, rightmin

if __name__ == '__main__':
    startnode()
