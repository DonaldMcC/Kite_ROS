#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
# This will generally provide functions to convert the resistance readings to angles
# should be fairly straightforward
# result will be an outmessage dictionary - this module should not require any ROS functions

from kitebar import params


def get_force(messageval, minohm, maxohm):
    # think we need to figure out shape of force chart for this as well
    force = 0
    return force

def get_angle(rcent):
    # need to establish if resistor is linear across range - probalby some graphing required and also what
    # are the max angle then we have a plot to work with
    a=params['centremaxleft']
    b=params['centremiddle']
    c=params['centremaxright']
    angle = 0

    return angle


def proc_arduino(message):
    """This will receive a dictionary and initially just convert the three resistors via a function"""
    answer = {}
    answer['forceleft'] = get_force(message.rleft, params.leftmin, params.leftmax)
    answer['forceright'] = get_force(message.rright, params.rightmin, params.rightmax)
    answer['barangle'] = get_angle(message.rcent)
    return answer