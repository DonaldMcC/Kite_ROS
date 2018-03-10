#  This module will cover setting the target bar angle based on desired route.  The actual bar angle
#  will come via arduino or some other rospy simulation message
#   their will be a number of situations
#   1 park mode - aim is just to keep the kiteangle to zero so we adjust bar angle accordingly
#   2 wiggle mode - aim will I think be to get kiteangle to specified value and once there we change so
#     essentially a variation of 1 with targetangle not being zero - when we hit the target angle we trigger
#     change phase - aim of wiggle is to learn something about the kite
#   3 xwind phase - here there are some options as to how best to proceed - if we have clear direction may use that
#     if no direction then we could calculate and use targetangle - but this seems a fairly different approach
#   4 turn phase - when starting we will have a maximum setting on the base and will just use that
#   5 once we are nearing end of turn we should switch to angle setting again
#     if kite not found we will need to look at best plan - if high then we would aim to park and there should be
#     a value of the base that has been calculated and lets us do this - if left or right then we should be in a
#     turn phase but recovery will be tricky - if low we probably just need to go to base angle and hope for the best
#
#  so inputs will be phase, kitefound, position, targetangle, target point and we will somehow need to establish
#  when coming out of a turn - let's assume we don't need PID style control yet as wind is dynamic and there is no real
#  stability - however there is undoubtedly lag between moving the bar and the kite adjusting and if we don't have a
#  system to cope with that then it will fail - we probably need to project and anticipate at the start and end
#  but also if possible learn the likely 

from collections import deque
from routeplan import Kite, Controls


class Base(object):

    def __init__(self, barangle=0, parkangle=0, maxright=45, maxleft=-45, lag=1,
                 targetbarangle=0, kitebarratio=1):
        self.barangle = barangle
        self.parkangle = parkangle
        self.maxright = maxright
        self.maxleft = maxleft
        self.lag = lag
        self.barangles = deque(maxlen=16)
        self.targetbarangle = targetbarangle
        self.kitebarratio = kitebarratio  # this will be the rate of change of barangle to kite angle
    

def calcbarangle(kite, base, controls):
    """This should just basically set the target bar angle based on the mode phase
    and zone we are in when in park or wiggle mode
    >>> k=Kite(400, targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> calcbarangle(k,b,c)
    35

    >>> k=Kite(400, phase='TurnR', targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> calcbarangle(k,b,c)
    45

    """
    if kite.phase == "TurnR" or kite.phase == "TurnL":
        return setangleturn(kite, base)
    else:
        return setangle(kite, base, controls)


def setangle(kite, base, controls):
    """This will return targetbarangle for park mode based largely on kiteangle
    We will start simple but may move to a pid mode if required

    >>> k=Kite(400, targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> setangle(k,b,c)
    35

    """  .
    delta = kite.targetangle - kite.kiteangle
    targetbarangle = checklimits(base.barangle + (delta * base.kitebarratio),
                                 base.maxleft, base.maxright)
                                 
    return targetbarangle


def setangleturn(kite, base):
    """This should be a simple function as we will always aim to turn as fast as poss
    identifying the point to ease off from max turn should be done as part of phase setting and not here

    >>> k=Kite(400)
    >>> b=Base(400)
    >>> setangleturn(k,b)
    -45
    """
    if kite.phase == "TurnR":
        targetbarangle = base.maxright
    else:
        targetbarangle = base.maxleft
    return targetbarangle    

                
def checklimits(angle, maxleft, maxright):
    """

    :param angle:
    :param maxleft:
    :param maxright:
    :return:

    >>> checklimits(50,-45,30)
    30
    >>> checklimits(-50,-45,30)
    -45
    >>> checklimits(-20,-45,30)
    -20

    """
    if angle < maxleft:
        angle = maxleft
    elif angle > maxright:
        angle = maxright
    return angle


def _test():
    import doctest
    doctest.testmod(verbose=False)


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()  
