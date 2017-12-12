#!/usr/bin/env python
# these values were derived from testing of the base units three resistors.
# may move and import from file at some point but for now they are just hard coded here

import rospy


def set_kite_base():
    rospy.set_param('leftmax', 1023)
    rospy.set_param('leftmin', 509)
    rospy.set_param('centremaxleft', 672)
    rospy.set_param('centremiddle', 829)
    rospy.set_param('centremaxright', 1015)
    rospy.set_param('rightmax', 227)
    rospy.set_param('rightmin', 697)


def set_params():
    set_kite_base()


if __name__ == '__main__':
    set_params()
