#!/usr/bin/env python
# from ros wiki for initial test

import rospy
params = {}


def get_params():
    """This will download all exepcted parameters from the ROS parameter server"""
    # TODO look at identifying values not received for now this is setting default value but that may not be best
    # possibly additional parameters required here
    global params
    params['source'] = rospy.get_param('source', 'arduino')
    params['leftmax'] = rospy.get_param('leftmax', 1000)
    params['leftmin'] = rospy.get_param('leftmin', 0)
    params['centremaxleft'] = rospy.get_param('centremaxleft', 1000)
    params['centremiddle'] = rospy.get_param('centremiddle', 500)
    params['centremaxright'] = rospy.get_param('centremaxright', 0)
    params['maxangleleft'] = rospy.get_param('maxangleleft', -65)
    params['maxangleright'] = rospy.get_param('maxangleright', 60)
    params['rightmax'] = rospy.get_param('rightmax', 1000)
    params['rightmin'] = rospy.get_param('rightmin', 0)
    return params
