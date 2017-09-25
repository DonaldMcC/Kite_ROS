#!/usr/bin/env python
import cv2
import numpy as np
import time
import routeplan
from collections import deque
from move_func import get_angle

#camera = cv2.VideoCapture(0)
#camera=cv2.VideoCapture('IMG_0464.MOV')
camera=cv2.VideoCapture('choppedkite_horizshort.mp4')

es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10))
kernel = np.ones((5,5),np.uint8)
background = None

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
pts = deque(maxlen=16)
counter = 0
(dX, dY) = (0, 0)
direction = ""

#http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
boundaries = [
	([17, 15, 100], [50, 56, 200]),
	([86, 31, 4], [220, 88, 50]),
	([25, 146, 190], [62, 174, 250]),
	([103, 86, 65], [145, 133, 128])
]

boundaries = [
	([0, 0, 0], [40, 40, 40])
]
#greem
boundaries = [
	([10, 100, 10], [100, 255, 100])
]

#orange
boundaries = [
	([0, 50, 100], [100, 200, 255])
]

boundaries = [
	([0, 0, 0], [40, 40, 40]),
	([10, 100, 10], [100, 255, 100]),
	([0, 50, 100], [100, 200, 255])
]


for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    low = np.array(lower, dtype="uint8")
    upp = np.array(upper, dtype="uint8")

try: # this will fail for now but don't need yet
    mode = rospy.get_param('mode')
    centrex = rospy.get_param('centrex')
    centrey = rospy.get_param('centrey')
    halfwidth = rospy.get_param('halfwidth')
    radius = rospy.get_param('radius')
except NameError:
    #mode='fig8'
    mode='park'
    centrex = 400
    centrey = 300
    halfwidth = 200
    radius = 100

routepoints = routeplan.calc_route(mode, centrex, centrey, halfwidth, radius)

#this is just for display flight decisions will be elsewhere
def drawroute(routepoints):
    for i, j in enumerate(routepoints):
        if i < len(routepoints) - 1:
            cv2.line(frame, (j[0], j[1]), (routepoints[i+1][0], routepoints[i+1][1]), (255, 0, 255), thickness=1, lineType=8, shift=0)
        else:
            cv2.line(frame, (j[0], j[1]), (routepoints[0][0], routepoints[0][1]), (255, 0, 255), thickness=1,
                     lineType=8, shift=0)
    return

while (True):
    ret, frame = camera.read()
    if background is None:
        background = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        background = cv2.GaussianBlur(background, (21, 21), 0)
        continue
  
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
    diff = cv2.absdiff(background, gray_frame)
    diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    diff = cv2.dilate(diff, es, iterations = 2)
    image, cnts, hierarchy = cv2.findContours(diff.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  
    for c in cnts:
        if cv2.contourArea(c) < 1500:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        roi = frame[y:y+h, x:x+w]
        # loop over the boundaries
        mask = cv2.inRange(roi, low, upp)
        if np.sum(mask) > 1000:
            #outputframe = cv2.bitwise_and(roi, mask, mask=mask)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
            finalframe = frame[y:y+h, x:x+w]
            center = (x +(w//2), y + (h // 2))
            pts.appendleft(center)

            #Min Araa seems reasonable to get angle of kite
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            kiteangle = get_angle(box)
            cv2.putText(frame, str(int(kiteangle)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 255), 3)

            if counter % 100 == 0:
                for i, item in enumerate(box):
                    print(i, item)
            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

            for i in np.arange(1, len(pts)):
                if pts[i] is None:
                    continue

                # check to see if enough points have been accumulated in
                # the buffer
                if counter >= 10 and i == 1 and pts[10] is not None:
                # compute the difference between the x and y
                # coordinates and re-initialize the direction
                # text variables
                    dX = pts[i][0] - pts[-10][0]
                    dY = pts[i][1] - pts[-10][1]
                    (dirX, dirY) = ("", "")

                    # ensure there is significant movement in the
                    # x-direction
                    if np.abs(dX) > 20:
                        dirX = "East" if np.sign(dX) == 1 else "West"

                    # ensure there is significant movement in the
                    # y-direction
                    if np.abs(dY) > 20:
                        dirY = "South" if np.sign(dY) == 1 else "North"

                    # handle when both directions are non-empty
                    if dirX != "" and dirY != "":
                        direction = "{}-{}".format(dirY, dirX)

                    # otherwise, only one direction is non-empty
                    else:
                        direction = dirX if dirX != "" else dirY

                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
                    cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

                    # show the movement deltas and the direction of movement on
                    # the frame
                    cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 255), 3)
                    cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
                        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (0, 0, 255), 1)
                    continue
            continue

    drawroute(routepoints)
    #cv2.imshow("roi", finalframe)
    #cv2.imshow("mask", mask)
    cv2.imshow("contours", frame)
    counter += 1
    if counter % 100 == 0:
        pass
        #print(center)
        #print(pts[10][0],pts[10][1])
        #print(dX, dY)
        #print(direction)
    # cv2.imshow("dif", diff)
    # keys should be Left, Right, Up, Down, Widen and Narrow, Extend and Contract which should set all routes
    # Pause should hold for 5 secs
    # TODO add remaining key controls
    key = cv2.waitKey(1000 / 12) & 0xff
    if key == ord("l"):
        centrex -= 1
        routepoints = routeplan.calc_route(mode, centrex, centrey, halfwidth, radius)
    elif key == ord("r"):
        centrex += 1
        routepoints = routeplan.calc_route(mode, centrex, centrey, halfwidth, radius)
    elif key == ord("p"):
        time.sleep(5)
    elif key == ord("q"):
        break
    # if \cv2.waitKey(1000 / 12) & 0xff == ord("q"):
    # break++


cv2.destroyAllWindows()
camera.release()
