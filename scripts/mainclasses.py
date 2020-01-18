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


class Config(object):
    def __init__(self, source=2, setup='Standard', masklimit=10000,
                 logging=0, numcams=1, input='keyboard', check_motor_sim=False):
        self.source = source
        self.setup = setup
        self.masklimit = masklimit
        self.logging = logging
        self.numcams = numcams
        self.input = input
        self.check_motor_sim = check_motor_sim


class Base(object):

    def __init__(self, barangle=0, parkangle=0, maxright=40, maxleft=-40, lag=1,
                 targetbarangle=0, kitebarratio=1, updatemode='Standard', inferbarangle=0):
        self.barangle = barangle
        self.parkangle = parkangle
        self.maxright = maxright
        self.maxleft = maxleft
        self.lag = lag
        self.barangles = deque(maxlen=16)
        self.targetbarangle = targetbarangle
        self.inferbarangle = inferbarangle
        self.kitebarratio = kitebarratio  # this will be the rate of change of barangle to kite angle
        self.updatemode = updatemode  # Standard will be unconnected and Manbar will be bar angles kite
        self.mockangle = 0
        self.reset = False

class Kite(object):

    def __init__(self, x=0, y=0, mode='Park', phase='Park', targetheading=0, targetangle=0, kiteangle=0):
        self.x = x
        self.y = y
        self.mode = mode
        self.phase = phase
        self.pts = deque(maxlen=16)
        self.kiteangles = deque(maxlen=16)
        self.timestamps = deque(maxlen=16)
        (self.dX, self.dY) = (0, 0)
        self.direction = ""
        self.kiteangle = kiteangle
        self.contourarea = 0
        self.zone = "Centre"
        self.targettype = 'Angle'
        self.targetx = 0
        self.targety = 0
        self.changezone = True
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
        >>> k=Kite(400)
        >>> k.get_zone(100,600)
        'Left'

        >>> l=Kite(400)
        >>> l.get_zone(300,600)
        'Centre'

        :param leftx:
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
            self.phase = 'Hold'
        elif self.mode == 'Wiggle':
            self.phase = 'Wiggle'
        else:  # fig8 - assumed
            if self.zone == 'Centre':
                self.phase = 'Xwind'
            elif self.zone == 'Left':
                if self.turncomplete or self.kiteangle > self.turncomplete_angle:
                    self.phase = 'Xwind'
                    self.turncomplete = True
                    self.routechange = True
                else:
                    self.phase = 'TurnRight'
            else:  # Right zone
                if self.turncomplete or self.kiteangle < (0 - self.turncomplete_angle):
                    self.phase = 'Xwind'
                    self.turncomplete = True
                    self.routechange = True
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
                if abs(self.x - leftx) > abs(self.x - rightx):
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
                print('End of update_target reached without cover expected cases most likely')
                # TODO ensure change of flight mode is barred unless in the centre zone -
                # seems sensible and should
                # mena changemode and changephase generally only triggered in centre zone

        return


class Controls(object):

    def __init__(self, config='Standard', step=8):
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
        if self.config == 'Standard':
            self.inputmode = 0
        elif self.config == 'Manfly':  # Manfly
            self.inputmode = 2
        else:
            self.inputmode = 3  # Manbar
        self.step = step
        self.modestring = self.getmodestring(True)
        self.route = False
        self.maxy = 20  # this should be for top of centre line and sets they y target point for park mode
        self.slow = 0.0
        self.newbuttons = []

    def getmodestring(self, inputmode):
        # So now always 11 buttons and first 5 and last 2 are std and iteration through should be std
        # so we would have a defined transition of names based on which change took place
        if inputmode == 0:  # Standard
            return 'STD: Left Right Up Down Pause Wider Narrow Expand Contract Mode Quit'
        elif inputmode == 1:
            return 'SETFLIGHTMODE: Left Right Up Down Pause Park Wiggle Fig8 Reset Mode Quit'
        elif self.inputmode == 2:
            return 'MANFLIGHT: Left Right Up Down Pause Anti Clock Gauche rigHt Mode Quit'
        else:  # inputmode = 3
            return 'MANBAR: Left Right Up Down Pause Anti Clock Gauche rigHt Mode Quit'

    @staticmethod
    def get_change_mode_buttons(inputmode):
        if inputmode == 0:
            newbuttons = [('Mode: STD:', 'Mode: STD:'), ('Wider', 'Wider'), ('Narrow', 'Narrow'),
                          ('Expand', 'Expand'), ('Contract', 'Contract')]
        elif inputmode == 1:
            newbuttons = [('Mode: STD:', 'Mode: SETFLIGHTMODE:'), ('Wider', 'Park'), ('Narrow', 'Wiggle'),
                          ('Expand', 'Fig8'), ('Contract', 'Reset')]
        elif inputmode == 2:
            newbuttons = [('Mode: STD:', 'Mode: MANFLIGHT'), ('Wider', 'Anti'), ('Narrow', 'Clock'),
                          ('Expand', 'Gauche'), ('Contract', 'rigHt')]
        else:
            newbuttons = [('Mode: STD:', 'Mode: MANBAR:')]
        return newbuttons

    def joyhandler(self, joybuttons, joyaxes, kite, base, event=None):
        # Using https://github.com/arnaud-ramey/rosxwiimote as a ros package to capture
        # the joystick message this was because std one tried to do bluetooth
        # connection to wiimote via python and it didn't work perhaps as only
        # seems to expect early versions of wiimote

        # The axes messages is as follows:
        # 0. left - right rocker(3 possible values: -1 = left 0 = released 1 = right)
        # 1. up - down rocker(3 possible values: -1 = left 0 = released 1 = right)
        # 2. nunchuk left - right joystick(floating value in the range - 1 = left..1 = right)
        # 3. nunchuk down - up joystick(floating value in the range - 1 = down.. 1 = up)

        # 0. XWII_KEY_A - maybe the pause button
        # 1. XWII_KEY_B - this should toggle the rockers between move and stretch squashc
        # 2. XWII_KEY_PLUS - probably the faster button and poss some other things
        # 3. XWII_KEY_MINUS - probably the slower button and poss some other things
        # 4. XWII_KEY_HOME this should be the quit key
        # 5. XWII_KEY_ONE  this will do an input mode change
        # 6. XWII_KEY_TWO  and this will do a flight mode change
        # 7. XWII_KEY_C - so this will be left or anticlockwise flight depending on key b
        # 8. XWII_KEY_Z   and this will be right or clockwise kite depending on key b

        # in terms of what we do with this the basic idea is that the nunchuk flies the kite
        # and the rockers support the route moving about
        reset_stitcher = False
        if self.inputmode == 0:  # Standard
            if (joybuttons and joyaxes[0] == -1) or event == 'Left':  # left:  # left
                self.centrex -= self.step
                kite.routechange = True
            elif (joybuttons and joyaxes[0] == 1) or event == 'Right':  # right
                self.centrex += self.step
                kite.routechange = True
            elif (joybuttons and joyaxes[1] == 1) or event == 'Up':  # up
                self.centrey -= self.step
                kite.routechange = True
            elif (joybuttons and joyaxes[1] == -1) or event == 'Down':  # down
                self.centrey += self.step
                kite.routechange = True
            elif event == 'Wider':  # wider
                self.halfwidth += self.step
            elif event == 'Narrow':  # narrower
                self.halfwidth -= self.step
            elif event == 'Expand':  # expand
                self.radius += self.step
            elif event == 'Contract':  # contract
                self.radius -= self.step
            elif (joybuttons and joybuttons[3] == 1) or event == 'Slow':  # slow
                self.slow += 0.1
            elif (joybuttons and joybuttons[2] == 1) or event == 'Fast':  # fast
                self.slow = 0.0
            elif (joybuttons and joybuttons[0] == 1) or event == 'Pause':  # pause - this may apply in all modes
                time.sleep(10)

        if self.inputmode == 1:  # SetFlight
            if (joybuttons and joyaxes[0] == -1) or event == 'Left':  # left:  # left
                self.centrex -= self.step
                kite.routechange = True
            elif (joybuttons and joyaxes[0] == 1) or event == 'Right':  # right
                self.centrex += self.step
                kite.routechange = True
            elif (joybuttons and joyaxes[1] == 1) or event == 'Up':  # up
                self.centrey -= self.step
                kite.routechange = True
            elif (joybuttons and joyaxes[1] == -1) or event == 'Down':  # down
                self.centrey += self.step
                kite.routechange = True
            if joybuttons and joybuttons[6] == 1:  # move mode forward
                if kite.mode == 'Park':
                    kite.mode = 'Wiggle'
                elif kite.mode == 'Wiggle':
                    kite.mode = 'Fig8'
                else:
                    kite.mode = 'Park'
            if event == 'Wider':  # park
                kite.mode = 'Park'
            elif event == 'Narrow' and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Wiggle'
            elif event == 'Expand' and kite.zone == 'Centre':  # must be in central zone to change mode
                kite.mode = 'Fig8'
            elif event == 'Contract':  # Reset message
                base.reset = True
        elif self.inputmode == 2:  # ManFlight - maybe switch to arrows - let's do this all
            if joybuttons:
                if joyaxes[0] != 0:  # -1 = left +1 = right
                    self.centrex += self.step * int(joyaxes[0])
                    kite.routechange = True
                elif joyaxes[1] != 0:  # 1 = up -1 = down so needs inverted
                    self.centrey += self.step * int(joyaxes[1])
                    kite.routechange = True
                if joybuttons[7] == 0 and joybuttons[8] == 0:
                    kite.x += (self.step * joyaxes[2])
                    kite.y -= (self.step * joyaxes[3])
                elif joybuttons[7] == 1:  # c button pressed - but not working - as calced from kite
                    base.barangle += (self.step / 2 * joyaxes[2])
                else:  # z button pressed
                    kite.kiteangle += (self.step / 2 * joyaxes[2])
            # move via buttons
            if event == 'Left':  # left
                kite.x -= self.step
            elif event == 'Right':  # right
                kite.x += self.step
            elif event == 'Up':  # up
                kite.y -= self.step
            elif event == 'Down':  # down
                kite.y += self.step
            elif event == 'Expand':  # bar gauche
                base.barangle -= self.step
            elif event == 'Contract':  # bar rigHt
                base.barangle += self.step
            elif event == 'Wider':  # anti clockwise
                kite.kiteangle -= self.step
            elif event == 'Narrow':  # clockwise
                kite.kiteangle += self.step
        elif self.inputmode == 3:  # ManBar - maybe switch to arrows - let's do this all
            if joybuttons:
                if joyaxes[0] != 0:  # -1 = left +1 = right
                    self.centrex += self.step * int(joyaxes[0])
                    kite.routechange = True
                elif joyaxes[1] != 0:  # 1 = up -1 = down so needs inverted
                    self.centrey += self.step * int(joyaxes[1])
                    kite.routechange = True
            if joybuttons and joybuttons[7] == 0 and joybuttons[8] == 0:
                base.barangle += (self.step / 2 * joyaxes[2])
            if event == 'Wider':  # anti-clockwise
                base.barangle -= self.step  # this will change
            elif event == 'Narrow':  # clockwise
                base.barangle += self.step
            elif event == 'Left':  # left
                kite.x -= self.step
            elif event == 'Right':  # right
                kite.x += self.step
            elif event == 'Up':  # up
                kite.y -= self.step
            elif event == 'Down':  # down
                kite.y += self.step
        if (joybuttons and joybuttons[5] == 1) or event == 'Mode':  # modechange
            self.inputmode += 1
            if self.inputmode == 4:  # simple toggle around 3 modes
                self.inputmode = 0
            self.modestring = self.getmodestring(self.inputmode)
            self.newbuttons = self.get_change_mode_buttons(self.inputmode)

        if event == 'Pause':
            time.sleep(10)

        return joybuttons and joybuttons[4] == 1, reset_stitcher  # quit


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
