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


def getangle(resistance, maxleft=-45, maxright=45, resistleft=740, resistright=340):
    """
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
    # for all of these and we will for now assume resistor is linear

    if resistleft >= resistance >= resistright:
        angle = maxright - ((resistance - resistright) * (maxright - maxleft) / (resistleft - resistright))
        # angle = -45 + ((resistance - resistleft)/ 400.0)
    else:
        angle = 0
    return int(angle)


def getresist(angle, maxleft=-45, maxright=45, resistleft=740, resistright=340):
    """
    :param resistright:
    :param resistleft:
    :param maxright:
    :param maxleft:
    :param angle:
    :return angle:

    >>> getresist(-45)
    740
    >>> getresist(0)
    540
    >>> getresist(45)
    340
    """

    # calibration is based on 0 being the centre and maxleft and maxright being
    # defined in degrees - the corrsesponding values of the resistor should be taken
    # for all of these and we will for now assume resistor is linear

    if maxleft <= angle <= maxright:
        resistance = resistleft + ((angle - maxleft) * (resistright - resistleft) / (maxright - maxleft))
    else:
        # expected value at centrepoint
        resistance = (resistleft + resistright) / 2

    return int(resistance)


def _test():
    import doctest
    doctest.testmod(verbose=False)


if __name__ == '__main__':
    'Can run with -v option if you want to confirm tests were run'
    _test()
