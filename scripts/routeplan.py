#!/usr/bin/env python
#
"""
This file should do the following things

    1   Calculate a default route for the kite based on settings or parameters applied
    2   Identify the flight zone which will be left, right or centre in fig8
        and park_left, park_right
    3   Do we have a current route - if so let's continue it for now unless zone changed
        probably need a safety check now
    4   If not what zone are we in - if park for now we will just go left or right and aim to stay above
        the zone
    5   Probably then go left a bit and right a bit - lets call that wiggle mode
    6   Then move into fig 8 with upturns - let's always start left and should be aimed high - probably just need
        to display a centre line and always draw fig 8 and resize manually for now - full automation of that can
        be later
    7   At some point we then flick from park to fig8 and it should then set off towards say bottom left of fig8
    8   Once there we flick into upturn and measure turn radius for a few cycles - turn stops when kite is round 180deg
    9   Then repeat to other side -
    10  At some point we would switch to doing down turns but that can probably be well after upturns work reliably so
    11  Upturns only for now
"""
import numpy as np

def calc_route(centrex=400, centrey=300, halfwidth=200, radius=100):
    """This just calculates the 6 points in our basic figure of eight
    should be easy enough and we then draw lines between each point and
    get the last point

    >>> calc_route(400, 300, 200, 100)
    [(200, 400), (100, 300), (200, 200), (600, 400), (700, 300), (600, 200)]

    """

    leftx = centrex - halfwidth
    rightx = centrex + halfwidth

    pt0 = (leftx, centrey + radius)
    pt1 = (leftx - radius, centrey)
    pt2 = (leftx, centrey - radius)
    pt3 = (rightx, centrey + radius)
    pt4 = (rightx + radius, centrey)
    pt5 = (rightx, centrey - radius)

    return [pt0, pt1, pt2, pt3, pt4, pt5]


def get_phase(center, mode, centrex, centrey,  routepoints, currtarget, currphase):
    if mode == 'park':
        # TODO this will change as even with park will need a strategy to get to the centre
        target = (centrex, centrey)
        phase = 'hold'
    else:  #  fig8
        target = (centrex, centrey)
        phase = 'hold'
    return(target, phase)


def get_zone(centre, mode, centrex, centrey, routepoints):
    if mode == 'park':
        if centre[0] <= centrex:
            zone='Park Left'
        else:
            zone='Park Right'
    else:  # fig8  either up turn or down
        if centre[0] < routepoints[1][0]:
            zone='Left'
        elif centre[0] > routepoints[4][0]:
            zone='Right'
        else:
            zone='Centre'
    return zone


# flightpath = calc_route()


def get_angle(box):
    # (0, array([809, 359], dtype=int64))
    # (1, array([743, 310], dtype=int64))
    # (2, array([802, 230], dtype=int64))
    # (3, array([868, 279], dtype=int64))
    pass


def _test():
    import doctest
    doctest.testmod()


if __name__ == '__main__':
    # Can run with -v option if you want to confirm tests were run
    _test()
