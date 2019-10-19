import numpy as np
import cv2

from mainclasses import Kite, Controls, Base
# http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
# boundaries = [([0, 0, 0], [40, 40, 40])]
# green
# boundaries = [([10, 100, 10], [100, 255, 100])]
# orange
# boundaries = [([0, 50, 100], [100, 200, 255])]

# iphone video
# contourmin = 3000
# wind
contourmin = 800


def kitemask(c, frame, kitecolours='kite1'):
    # This sets the properties for the kite we are looking for
    # setup for now is just for kite1 but we can be looking for in
    # different conditions and this might affect the colours
    # so think we amend this to add the object for indoorkite
    # and 
    if cv2.contourArea(c) < contourmin:
        return 0

    if kitecolours == 'indoorkite':
        boundaries = [([10, 10, 140], [70, 70, 200])]

    else:
        boundaries = [([0, 0, 0], [30, 30, 30]),
                      ([10, 10, 100], [100, 100, 255]),
                      ([0, 50, 100], [120, 220, 255])
                      ]
        # iphone
        boundaries = [([0, 0, 100], [100, 100, 255]),
                      ([0, 50, 150], [120, 220, 255])
                      ]
        # wind 64,111,106
        boundaries = [([0, 0, 100], [100, 100, 255]),
                      ([0, 50, 100], [120, 220, 255])
                      ]

    totmask = 1

    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        low = np.array(lower, dtype="uint8")
        upp = np.array(upper, dtype="uint8")

        (x, y, w, h) = cv2.boundingRect(c)
        roi = frame[y:y + h, x:x + w]
        # loop over the boundaries
        mask = cv2.inRange(roi, low, upp)
        totmask *= np.sum(mask)
        print(x, y, w, h, "cont", cv2.contourArea(c))
        print("mask: ", np.sum(mask), totmask)
    return totmask


def calcbarangle(kite, base, controls):
    """This should just basically set the target bar angle based on the mode phase
    and zone we are in when in park or wiggle mode
    >>> k=Kite(400, targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> calcbarangle(k,b,c)
    10

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
    """This will return targetbarangle for park mode based largely on kite target angle
    We will start simple but may move to a pid mode if required

    >>> k=Kite(400, targetangle=10)
    >>> b=Base(barangle=15, kitebarratio=2)
    >>> c=Controls(1)
    >>> setangle(k,b,c)
    10
    """

    # targetbarangle = checklimits((kite.targetangle * base.kitebarratio), base.maxleft, base.maxright)
    targetbarangle = checklimits(kite.targetangle, base.maxleft, base.maxright)
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


def getangle(resistance, maxleft=-45, maxright=45, resistleft=340, resistright=740):
    """
    :param maxleft:
    :param resistance:
    :return angle:

    >>> getangle(340)
    -45
    >>> getangle(540)
    0
    >>> getangle(740)
    45
    >>> getangle(350)
    0
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear

    if resistance >= resistleft and resistance <= resistright:
        angle = maxleft + ((resistance - resistleft) * (maxright - maxleft) / (resistright - resistleft))
        #angle = -45 + ((resistance - resistleft)/ 400.0)
    else:
        angle = 0
    return int(angle)


def getresist(angle, maxleft=-45, maxright=45, resistleft=340, resistright=740):
    """
    :param maxleft:
    :param angle:
    :return angle:

    >>> getresist(-45)
    340
    >>> getresist(0)
    540
    >>> getresist(45)
    740
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear

    if angle >= maxleft and angle <= maxright:
        resistance = resistleft + ((angle - maxleft) * (resistright - resistleft)/ (maxright - maxleft) )
    else:
        resistance = (resistleft + resistright) / 2

    return int(resistance)


def _test():
    import doctest
    doctest.testmod(verbose=False)


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()
