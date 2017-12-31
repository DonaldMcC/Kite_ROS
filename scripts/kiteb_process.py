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


def linearmap(value, minx, maxx, miny, maxy):
    return miny + (value-minx)/(maxx-minx) * (maxy-miny)


def get_angle(rcent, centremaxleft, centremiddle, centremaxright, maxangleleft, maxangleright):
    # need to establish if resistor is linear across range - probalby some graphing required and also what
    # are the max angle then we have a plot to work with
    # TODO will need maxangle for this as well


    # allow resistor to operate either way +ve
    if centremaxleft < centremaxright:
        if rcent >= centremiddle and rcent<= centremaxright:
            angle = linearmap(rcent, centremiddle, centremaxright, 0, maxangleright)
        elif rcent >= centremaxleft and rcent < centremiddle:
            #this should return a -ve and maxangleleft should also be set as -ve
            angle = linearmap(rcent, centremiddle, centremaxleft, 0, maxangleleft)
        else:
            print('centre angle outwith permitted range')
    else:  # centremaxleft is +ve so we need to invert mapping
        if rcent <= centremiddle and rcent >= centremaxright:
            angle = linearmap(rcent, centremiddle, centremaxright, 0, maxangleright*-1)
        elif rcent <= centremaxleft and rcent > centremiddle:
            # this should return a -ve and maxangleleft should also be set as -ve
            angle = linearmap(rcent, centremiddle, centremaxleft, 0, maxangleleft * -1)
        else:
            print('centre angle outwith permitted range')

    return angle


def proc_arduino(message):
    """This will receive a dictionary and initially just convert the three resistors via a function"""
    answer = {}
    answer['forceleft'] = get_force(message.rleft, params.leftmin, params.leftmax)
    answer['forceright'] = get_force(message.rright, params.rightmin, params.rightmax)
    answer['barangle'] = get_angle(message.rcent, params.centremaxleft, params.centremiddle, params.centremaxright)
    return answer