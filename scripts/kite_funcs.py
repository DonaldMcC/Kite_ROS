import numpy as np
import cv2

# http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
# boundaries = [([0, 0, 0], [40, 40, 40])]
# green
# boundaries = [([10, 100, 10], [100, 255, 100])]
# orange
# boundaries = [([0, 50, 100], [100, 200, 255])]

# iphone video
# contourmin = 3000
# msi wind
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


def getangle(resistance, maxleft=-20, maxright=20, resistleft=628, resistright=458, resistcentre=543):
    """
    :param resistcentre:
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param resistance:
    :return angle:

    >>> getangle(740)
    -45
    >>> getangle(540)
    0
    >>> getangle(340)
    45
    >>> getangle(350)
    42
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear - have now changed
    # so that values beyond maxleft and maxright should be supported

    if resistance > resistcentre:
        angle = ((resistance - resistcentre) * maxleft) / (resistleft - resistcentre)
    elif resistance < resistcentre:
        angle = ((resistance - resistcentre) * maxright) / (resistright - resistcentre)
    else:
        angle = 0
    return int(angle)

#TODO - this has default values which can be inconsistent with main classes - will redefine as
# Constants and then use in both places
def getresist(angle, maxleft=-20, maxright=20, resistleft=628, resistright=458, resistcentre=543):
    """
    :param resistcentre:
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param angle:
    :return angle:

    >>> getresist(-45)
    740
    >>> getresist(-5)
    562
    >>> getresist(0)
    540
    >>> getresist(45)
    340
    >>> getresist(10)
    495
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear

    if angle < 0:
        resistance = resistleft + ((angle - maxleft) * (resistcentre - resistleft) / (0 - maxleft))
    elif angle > 0:
        resistance = resistright + ((maxright - angle) * (resistcentre - resistright) / maxright)
    else:
        resistance = resistcentre
    return int(resistance)


def get_action(output, barangle, targetbarangle):
    # Now added ability to send motor message 6 for leftonly and 7 for rightonly
    # and will now add speed into the message as %age of max value up to 99 but 0 is max speed
    MAXLEFT = -20  # These are to try and avoid breaking the bar
    MAXRIGHT = 20  # similarly to protect bar as attached close to pivot
    TOLERANCE = 1  # degreee of tolerance
    action = 0
    if abs(output) < TOLERANCE:
        action = 0  # stop
    elif output > 0 and barangle > MAXLEFT:
        action = 300  # Left
    elif output < 0 and barangle < MAXRIGHT:
        action = 400  # Right
    # TODO think about how PID impacts this if at all - speed should prob be used
    # action = int(msg + speed) if 0 < speed < 100 else int(msg)
    return action


def _test():
    import doctest
    doctest.testmod(verbose=False)


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()
