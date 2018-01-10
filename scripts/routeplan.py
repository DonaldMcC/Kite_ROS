#!/usr/bin/env python
#
"""
This file should do the following things

    1   Calculate a default route for the kite based on settings or parameters applied - done
    2   Identify the flight zone which will be left, right or centre in fig8
        and park_left, park_right - done
    3   Do we have a current route - if so let's continue it for now unless zone changed
        probably need a safety check now
    4   If not what zone are we in - if park for now we will just go left or right and expect to go up - which will
        bring all the off-camera issues
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

from collections import deque

import numpy as np



class Kite:

    def __init__(self, x=0, y=0, manx=0, many=0, mode='Park', manangle=0):
        self.x = x
        self.y = y
        self.mode = mode
        self.manx = manx
        self.many = many
        self.manangle = manangle
        self.pts = deque(maxlen=16)
        (self.dX, self.dY) = (0, 0)
        self.direction = ""
        self.kiteangle = 0
        self.zone = ""
        self.phase = ""
        self.targetx = 0
        self.targety = 0


    def get_zone(self, control):
        if self.mode == 'Park':
            if self.x <= control.centrex:
                zone = 'Park Left'
            else:
                zone = 'Park Right'
        else:  # fig8  either up turn or down
            if self.x < control.routepoints[1][0]:
                zone = 'Left'
            elif self.x > control.routepoints[4][0]:
                zone = 'Right'
            else:
                zone = 'Centre'
        return zone


    def get_phase(self, control):
        if self.mode == 'Park':
            # TODO this will change as even with park will need a strategy to get to the centre
            target = (control.centrex, control.centrey)
            phase = 'Hold'
        else:  # fig8
            target = (control.centrex, control.centrey)
            phase = 'Hold'
        return (target, phase)



class Base:

    def __init__(self, barangle=0):
        self.barangle = barangle


class Controls:


    def __init__(self, inputmode=0, step=8):
        try:  # this will fail on windows but don't need yet and not convinced I need to set parameters separately
            self.centrex = rospy.get_param('centrex', 400)
            self.centrey = rospy.get_param('centrey', 300)
            self.halfwidth = rospy.get_param('halfwidth', 200)
            self.radius = rospy.get_param('radius', 100)
        except (NameError, KeyError) as e:
            #  mode='fig8'
            self.centrex = 400
            self.centrey = 300
            self.halfwidth = 200
            self.radius = 100
        self.routepoints = calc_route(self.centrex, self.centrey, self.halfwidth, self.radius)
        self.inputmodes = ('Standard', 'SetFlight', 'ManFly')
        self.inputmode = inputmode
        self.step = step
        self.modestring = self.getmodestring()
        self.route = False


    def getmodestring(self):
        if self.inputmode == 0:  # Standard
            return 'STD: Left Right Up Down Wider Narrow Expand Contract Pause Mode Quit'
        elif self.inputmode == 1:
            return 'SETFLIGHTMODE: Park Fig8 Mode Quit'
        else:
            return 'MANFLIGHT: Left Right Up Down Pause Mode Quit'


    def keyhandler(self, key, kite):
    # this will now support a change of flight mode and operating mode so different keys will
    # do different things depending on inputmode,

        if self.inputmode == 0:  # Standard
            if key == ord("l"):  # left
                self.centrex -= self.step
            elif key == ord("r"):  # right
                self.centrex += self.step
            elif key == ord("u"):  # up
                self.centrey -= self.step
            elif key == ord("d"):  # down
                self.centrey += self.step
            elif key == ord("w"):  # wider
                self.halfwidth += self.step
            elif key == ord("n"):  # narrower
                self.halfwidth -= 1
            elif key == ord("e"):  # expand
                self.radius += self.step
            elif key == ord("c"):  # contract
                self.radius -= self.step
            elif key == ord("p"):  # pause - this may apply in all moades
                time.sleep(10)
        elif self.inputmode == 1:  # SetFlight
            if key == ord("p"):  # park
                self.mode = 'park'
            elif key == ord("f"):  # fig8
                self.mode = 'fig8'
        elif self.inputmode == 2:  # ManFlight - maybe switch to arrows
            if key == ord("l"):  # left
                kite.manx -= self.step  # this will change
            elif key == ord("r"):  # right
                kite.manx += self.step
            elif key == ord("u"):  # up
                kite.many -= self.step
            elif key == ord("d"):  # down
                kite.many += self.step
            elif key == ord("p"):  # pause - this may apply in all moades
                time.sleep(10)

        if key == ord("m"):  # modechange
            self.inputmode += 1
            if self.inputmode == 3:  # simple toggle around 3 modes
                self.inputmode = 0
            self.modestring=self.getmodestring()

        self.routepoints = calc_route(self.centrex, self.centrey, self.halfwidth, self.radius)
        return



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
