#!/usr/bin/env python
# from ros wiki for initial testing
# kitebar will pick up
# This will generally provide functions to convert the resistance readings to angles
# should be fairly straightforward
# result will be an outmessage dictionary - this module should not require any ROS functions

from kiteb_param import params


def get_force(messageval, minohm, maxohm, minforce, maxforce):
    # think we need to figure out shape of force chart for this as well
    # this currently serves no purpose but will do once we understand springs

    force = linearmap(messageval, minohm, maxohm, minforce, maxforce)
    return force


def linearmap(value, minx, maxx, miny, maxy):
    """Transform resistance to angle in degrees
        >>> linearmap(300, 100, 500, -60, 0)
        -30.0
        >>> linearmap(400, 100, 500, 60, 0)
        15.0
    """
    return miny + (value-minx)/(1.0 * maxx-minx) * (maxy-miny)


def get_bar_angle(rcent, centremaxleft, centremiddle, centremaxright, maxangleleft, maxangleright):
    # TODO need to establish if resistor is linear across range - probalby some graphing required and also what
    # are the max angle then we have a plot to work with

    """Transform resistance to angle in degrees
        >>> get_bar_angle(300, 100, 500, 900, -60, 60)
        -30.0
        >>> get_bar_angle(600, 100, 500, 900, -60, 60)
        15.0
        >>> get_bar_angle(600, 900, 500, 100, -60, 60)
        -15.0
        >>> get_bar_angle(100, 900, 500, 100, -60, 90)
        90.0
    """

    angle = 0
    # allow resistor to operate either way +ve
    if centremaxleft < centremaxright:
        if centremiddle <= rcent <= centremaxright:
            angle = linearmap(rcent, centremiddle, centremaxright, 0, maxangleright)
        elif centremaxleft <= rcent < centremiddle:
            # this should return a -ve and maxangleleft should also be set as -ve
            angle = linearmap(rcent, centremiddle, centremaxleft, 0, maxangleleft)
        else:
            print('centre angle outwith permitted range')
    else:  # centremaxleft is +ve so we need to invert mapping
        if centremiddle <= rcent <= centremaxleft:
            angle = linearmap(rcent, centremiddle, centremaxright, 0, maxangleleft*-1)
        elif centremaxright <= rcent < centremiddle:
            # this should return a -ve and maxangleleft should also be set as -ve
            angle = linearmap(rcent, centremiddle, centremaxleft, 0, maxangleright * -1)
        else:
            print('centre angle outwith permitted range')

    return angle


def proc_arduino(message):
    """This will receive a dictionary and initially just convert the three resistors via a function"""
    answer = {'forceleft': get_force(message['rleft'], params['leftmin'], params['leftmax'],
                                     params['leftmin'], params['leftmax']),
              'forceright': get_force(message['rright'], params['rightmin'], params['rightmax'],
                                      params['rightmin'], params['rightmax']),
              'barangle': get_bar_angle(message['rcent'], params['centremaxleft'], params['centremiddle'],
                                        params['centremaxright'], params['maxangleleft'], params['maxangleright'])}
    return answer


def _test():
    import doctest
    doctest.testmod(verbose=True)


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()
