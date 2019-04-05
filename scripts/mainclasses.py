#!/usr/bin/env python
#
"""
This file should do the following things

    1   Calculate a default route for the kite based on settings or parameters applied - done
    2   Identify the flight zone which will be left, right or centre in fig8
        and park_left, park_right - done
    3   Identify the flight mode - currently proposing park wiggle and fig8up we always start with park
    4   Has phase change or zone changed or are we starting
    3   Set route - otherwise we know the preferred route
    4   Staring point will always be park and the target for that is kite angle of zero and to be at top of centre line
    5   Probably then go left a bit and right a bit - lets call that wiggle mode - but we align first
    6   Then move into fig 8 with upturns - let's always start left and should be aimed high - probably just need
        to display a centre line and always draw fig 8 and resize manually for now - full automation of that can
        be later - this will be set in motion manually
    7   Once there we flick into upturn and measure turn radius for a few cycles - turn stops when kite is round 180deg
    8   Then repeat to other side -
    9   At some point we would switch to doing down turns but that can probably be well after upturns work reliably so
    10  Upturns only for now
"""

import time
from collections import deque

from move_func import get_heading_points


class Base(object):

    def __init__(self, barangle=0, parkangle=0, maxright=45, maxleft=-45, lag=1,
                 targetbarangle=0, kitebarratio=1, updatemode=2):
        self.barangle = barangle
        self.parkangle = parkangle
        self.maxright = maxright
        self.maxleft = maxleft
        self.lag = lag
        self.barangles = deque(maxlen=16)
        self.targetbarangle = targetbarangle
        self.kitebarratio = kitebarratio  # this will be the rate of change of barangle to kite angle
        self.updatemode = updatemode # 0 will be unconnected and 1 will bar angles kite 2 is kite angles bar


class Kite(object):

    def __init__(self, x=0, y=0, mode='Park', phase='Park', targetheading=0, targetangle=0,
                 thickness=1, leftballx=0, leftbally=0, rightballx=0, rightbally=0):
        self.x = x
        self.y = y
        self.mode = mode
        self.phase = phase
        self.pts = deque(maxlen=16)
        self.kiteangles = deque(maxlen=16)
        self.timestamps = deque(maxlen=16)
        (self.dX, self.dY) = (0, 0)
        self.direction = ""
        self.kiteangle = 0
        self.contourarea = 0
        self.zone = "Centre"
        self.targettype = 'Angle'
        self.targetx = 0
        self.targety = 0
        self.changezone = False
        self.changephase = False
        self.routechange = False
        self.found = False
        self.targetheading = targetheading
        self.targetangle = targetangle
        self.thickness = 1
        self.leftballx = 0
        self.leftbally = 0
        self.rightballx = 0
        self.rightbally = 0
        self.turncomplete = False
        self.turncomplete_angle = 60

    def get_zone(self, leftx, rightx):
        """
        >>> k.get_zone(100,600)
        'Left'

        >>> l=Kite(400)
        >>> l.get_zone(300,600)
        'Centre'

        :param leftx:
        :param centrex:
        :param rightx:
        :return:
        """

        if self.x < leftx:
            self.zone = 'Left'
        elif self.x > rightx:
            self.zone = 'Right'
        else:
            self.zone = 'Centre'
        return self.zone

    def get_phase(self):
        if self.mode == 'Park':
            # For park this is now OK we want to get kiteangle to zero
            phase = 'Hold'
        elif self.mode == 'Wiggle':
            phase = 'Wiggle'
        else:  # fig8 - assumed
            if self.zone == 'Centre':
                self.phase = 'Xwind'
            elif self.zone == 'Left':
                if self.turncomplete or self.kiteangle > self.turncomplete_angle:
                    self.phase = 'Xwind'
                    self.turncomplete = True
                    self.routechange=True
                else:
                    self.phase = 'TurnRight'
            else:  # Right zone
                if self.turncomplete or self.kiteangle < (0-self.turncomplete_angle):
                    self.phase = 'Xwind'
                    self.turncomplete = True
                    self.routchange = True
                else:
                    self.phase = 'Turnleft'
        return

    def update_zone(self, control):
        currentzone = self.zone
        self.get_zone(control.routepoints[0][0], control.routepoints[3][0])
        if self.zone != currentzone:
            self.changezone = True
        else:
            self.changezone = False
        if self.changezone:  # set to false at start of next turn
            self.turncomplete = False

    def update_phase(self):
        currentphase = self.phase
        self.get_phase()
        if self.phase != currentphase:
            self.changephase = True
        else:
            self.changephase = False

    def get_wiggle_angle(self):
        if self.kiteangle > 0:
            return -10
        else:
            return 10

    def update_target(self, leftx, lefty, centrex, centrey, rightx, righty):
        # this gets called when mode, zone, phase or route changes
        # print('update targ called')
        if self.mode == 'Park':
            # For park this is now OK we want to get kiteangle to zero
            self.targettype = 'Angle'
            self.targetangle = 0
            self.targetx = centrex
            self.targety = centrey
        elif self.mode == 'Wiggle':
            self.targettype = 'Angle'
            self.targetangle = self.get_wiggle_angle()
            self.targetx = centrex
            self.targety = centrey
        else:  # fig8 - by definition
            if self.zone == 'Centre' or self.phase == 'Xwind':
                # Either we have just left the right or left turnzone so if nearest
                # left we go right and if nearest right we go left
                # or we have changed from park or wiggle to xwind which will be presumed to happen
                # with kite upwards and seems reasonable to just go for the longer xwind distance
                self.targettype = 'Point'
                if abs(self.x - leftx) > abs(self.x-rightx):
                    self.targetx = leftx
                    self.targety = lefty
                else:
                    self.targetx = rightx
                    self.targety = righty
                # self.targetangle = get_heading_points((self.x, self.y), (self.targetx, self.targety))
            elif self.changezone:  # think we should still set this roughly in the turn phase
                self.targettype = self.phase
                if self.phase == 'TurnR':
                    self.targetangle = 90
                else:
                    self.targetangle = -90
                    
                # TODO - may compute the target location
            else:
                print ('End of update_target reached without cover expected cases most likely ')
                # TODO ensure change of flight mode is barred unless in the centre zone - seems sensible and should
                # mena changemode and changephase generally only triggered in centre zone

        return


class Controls(object):

    def __init__(self, config ='Standard', inputmode=0, step=8):
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
        # possible config ('Standard', 'SetFlight', 'ManFly')
        self.config = config
        self.inputmode = inputmode
        self.step = step
        self.modestring = self.getmodestring()
        self.route = False
        self.maxy = 20  # this should be for top of centre line and sets they y target point for park mode
        self.slow = 0.0

    def getmodestring(self):
        if self.inputmode == 0:  # Standard
            return 'STD: Left Right Up Down Wider Narrow Expand Contract Pause Mode Quit'
        elif self.inputmode == 1:
            return 'SETFLIGHTMODE: Park Fig8 Simulate Normal Mode Quit'
        else:
            return 'MANFLIGHT: Left Right Up Down Pause Anti Clock Gauche rigHt Mode Quit'

    def keyhandler(self, key, kite, base=None):
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
            elif key == ord("s"):  # slow
                self.slow += 0.1
            elif key == ord("f"):  # fast
                self.slow = 0.0
            elif key == ord("p"):  # pause - this may apply in all modes
                time.sleep(10)
            # kite.routechange = True - don't want this triggered every time
        elif self.inputmode == 1:  # SetFlight
            if key == ord("p"):  # park
                kite.mode = 'Park'
            elif key == ord("w") and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Wiggle'
            elif key == ord("f") and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Fig8'
            elif key == ord("s"):  # simulation
                self.mode = 1
            elif key == ord("n"):  # normal with kite being present
                self.mode = 0
        elif self.inputmode == 2:  # ManFlight - maybe switch to arrows
            if key == ord("l"):  # left
                kite.x -= self.step  # this will change
            elif key == ord("r"):  # right
                kite.x += self.step
            elif key == ord("u"):  # up
                kite.y -= self.step
            elif key == ord("d"):  # down
                kite.y += self.step
            elif key == ord("g"):  # bar gauche
                base.barangle -= self.step
            elif key == ord("h"):  # bar rigHt
                base.barangle += self.step
            elif key == ord("a"):  # anti clockwise
                kite.kiteangle -= self.step
            elif key == ord("c"):  # clockwise
                kite.kiteangle += self.step
            elif key == ord("p"):  # pause - this may apply in all moades
                time.sleep(10)

        if key == ord("m"):  # modechange
            print (self.inputmode)
            self.inputmode += 1
            if self.inputmode == 3:  # simple toggle around 3 modes
                self.inputmode = 0
            self.modestring = self.getmodestring()

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


def _test():
    import doctest
    doctest.testmod(extraglobs={'k': Kite()})


if __name__ == '__main__':
    # Can run with -v option if you want to confirm tests were run
    _test()