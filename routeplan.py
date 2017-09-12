#!/usr/bin/env python
#
"""
This file should do the following things

    1   Calculate a default route for the kite based on settings or parameters applied
    2   When given the current position of the kite identify the target vector and current vector and
        consequently whether to go left or right
"""

def calc_route(mode='park', centrex=400, centrey=300, halfwidth=200, radius=100):
    """This just calculates the 6 points in our basic figure of eight
    should be easy enough and we then draw lines between each point and
    get the last point

    >>> calc_route('fig8', 400, 300, 200, 100)
    [(200, 400), (100, 300), (200, 200), (600, 400), (700, 300), (600, 200)]
    >>> calc_route('park', 400, 300, 200, 100)
    [(500, 400), (500, 200), (300, 200), (300, 400)]
    """

    if mode == 'fig8':
        leftx = centrex - halfwidth
        rightx = centrex + halfwidth

        pt0 = (leftx, centrey + radius)
        pt1 = (leftx - radius, centrey)
        pt2 = (leftx, centrey - radius)
        pt3 = (rightx, centrey + radius)
        pt4 = (rightx + radius, centrey)
        pt5 = (rightx, centrey - radius)

        return [pt0,pt1,pt2,pt3,pt4,pt5]

    else: #park
        pt0 = (centrex + radius, centrey + radius)
        pt1 = (centrex + radius, centrey - radius)
        pt2 = (centrex - radius, centrey - radius)
        pt3 = (centrex - radius, centrey + radius)

        return [pt0,pt1,pt2,pt3]

    #cv2.Line(img, pt1, pt2, color, thickness=1, lineType=8, shift=0)


def get_phase(x,y,width):
    """This might return the flight phase but not sure I need it
    should be easy enough and we then draw lines between each point and
    get the last point"""
    phase = 'left'
    return phase

#flightpath = calc_route()


def _test():
    import doctest
    doctest.testmod()

if __name__ == '__main__':
    # Can run with -v option if you want to confirm tests were run
    _test()

