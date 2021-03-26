import numpy as np
import cv2

conmaxright = 20
conmaxleft = -30
conresistleft =110
conresistright = 267
conresistcentre = 200

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
    print('kitemask')
    if kitecolours == 'indoorkite':
        boundaries = [([10, 10, 140], [70, 70, 200])]
    elif kitecolours == 'kite1':
        boundaries = [([0, 0, 100], [100, 100, 255]),
                      ([0, 50, 100], [120, 220, 255])
                      ]
    else:   # 'kite2'
        boundaries = [([0, 0, 0], [30, 30, 30]),
                      ([10, 10, 100], [100, 100, 255]),
                      ([0, 50, 100], [120, 220, 255])
                      ]
        # iphone
        boundaries = [([0, 0, 100], [100, 100, 255]),
                      ([0, 50, 150], [120, 220, 255])
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

    >>> checklimits(50, -45, 30)
    30
    >>> checklimits(-50, -45, 30)
    -45
    >>> checklimits(-20, -45, 30)
    -20
    """

    angle = max(angle, maxleft)
    angle = min(angle, maxright)
    return angle


def getangle(resistance, maxleft=conmaxleft, maxright=conmaxright,
             resistleft=conresistleft, resistright=conresistright, resistcentre=conresistcentre):
    """
    :param resistcentre:
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param resistance:
    :return angle:

    >>> getangle(267)
    20
    >>> getangle(200)
    0
    >>> getangle(110)
    -30
    >>> getangle(155)
    -15
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear - have now changed
    # so that values beyond maxleft and maxright should be supported

    if resistance > resistcentre:
        angle = ((resistance - resistcentre) * maxright) / (resistright - resistcentre)
    elif resistance < resistcentre:
        angle = ((resistance - resistcentre) * maxleft) / (resistleft - resistcentre)
    else:
        angle = 0
    return int(angle)


def getresist(angle, maxleft=conmaxleft, maxright=conmaxright, resistleft=conresistleft,
              resistright=conresistright, resistcentre=conresistcentre):
    """
    :param resistcentre:
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param angle:
    :return angle:

    >>> getresist(-30)
    110
    >>> getresist(-15)
    155
    >>> getresist(0)
    200
    >>> getresist(20)
    267
    >>> getresist(10)
    233
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


def get_action(output, barangle):
    # Now added ability to send motor message 6 for leftonly and 7 for rightonly
    # and will now add speed into the message as %age of max value up to 99 but 0 is max speed
    """
        :param output:
        :param barangle:
        :return action:

        >>> get_action(-10, -5)
        300
        >>> get_action(0, 0)
        0
        """

    MAXLEFT = conmaxleft  # These are to try and avoid breaking the bar
    MAXRIGHT = conmaxright  # similarly to protect bar as attached close to pivot
    TOLERANCE = 1  # degreee of tolerance
    action = 0
    if abs(output) < TOLERANCE:
        action = 0  # stop
    elif output < 0 and barangle > MAXLEFT:
        action = 300  # Left
    elif output > 0 and barangle < MAXRIGHT:
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
