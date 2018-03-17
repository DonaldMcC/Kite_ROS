#!/usr/bin/env python
# from ros wiki for initial test

import rospy
params = {}


def get_params():
    """This will download all exepcted parameters from the ROS parameter server"""
    # TODO look at identifying values not received for now this is setting default value but that may not be best
    # possibly additional parameters required here


    global params
    # params['source'] = rospy.get_param('source', 'arduino')
    # params['leftmax'] = rospy.get_param('leftmax', 1000)
    # params['leftmin'] = rospy.get_param('leftmin', 0)
    # params['centremaxleft'] = rospy.get_param('centremaxleft', 1000)
    # params['centremiddle'] = rospy.get_param('centremiddle', 500)
    # params['centremaxright'] = rospy.get_param('centremaxright', 0)
    # params['maxangleleft'] = rospy.get_param('maxangleleft', -65)
    # params['maxangleright'] = rospy.get_param('maxangleright', 60)
    # params['rightmax'] = rospy.get_param('rightmax', 1000)
    # params['rightmin'] = rospy.get_param('rightmin', 0)


    # Think above will be used again but for now setting to actual values of current rigup
    # and appears resistance is a linear function - so we can just test with these functions
    params['source'] = rospy.get_param('source', 'arduino')
    params['leftmax'] = rospy.get_param('leftmax', 1023)
    params['leftmin'] = rospy.get_param('leftmin', 505)
    params['centremaxleft'] = rospy.get_param('centremaxleft', 674)
    params['centremiddle'] = rospy.get_param('centremiddle', 826)
    params['centremaxright'] = rospy.get_param('centremaxright', 1005)
    params['maxangleleft'] = rospy.get_param('maxangleleft', -46)
    params['maxangleright'] = rospy.get_param('maxangleright', 47)
    params['rightmax'] = rospy.get_param('rightmax', 222)
    params['rightmin'] = rospy.get_param('rightmin', 686)

    return params
