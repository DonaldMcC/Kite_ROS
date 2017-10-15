#!/usr/bin/env python
# http://wiki.ros.org/rospy/Overview/Parameter%20Server
# https://docs.python.org/3/howto/curses.html looks like it will be a way to go for now


import rospy

# def calc_route(centrex=400, centrey=300, halfwidth=200, radius=100

# Using raw python objects
# rospy.set_param_raw('a_string', 'baz')
# rospy.set_param_raw('~private_int', 2)
# rospy.set_param_raw('list_of_floats', [1., 2., 3., 4.])
# rospy.set_param_raw('bool_True', True)
# rospy.set_param_raw('gains', {'p': 1, 'i': 2, 'd': 3})

# rospy.get_param('gains/P') #should return 1

centrex = 400
centrey = 300
halfwidth = 200
radius = 100


def set_params():
    rospy.set_param_raw('centrex', centrex)
    rospy.set_param_raw('centrey', centrey)
    rospy.set_param_raw('halfwidth', halfwidth)
    rospy.set_param_raw('radius', radius)


def run_setter():
    set_params()


if __name__ == '__main__':
    try:
        run_setter()
    except rospy.ROSInterruptException:
        pass
