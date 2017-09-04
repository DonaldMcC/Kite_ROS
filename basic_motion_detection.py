#!/usr/bin/env python
import cv2
import numpy as np
import routeplan

#camera = cv2.VideoCapture(0)
#camera=cv2.VideoCapture('IMG_0464.MOV')
camera=cv2.VideoCapture('choppedkite_horizshort.mp4')

es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10))
kernel = np.ones((5,5),np.uint8)
background = None


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
    centrex = rospy.get_param('centrex')
    centrey = rospy.get_param('centrey')
    halfwidth = rospy.get_param('halfwidth')
    radius = rospy.get_param('radius')
except NameError:
    centrex = 400
    centrey = 300
    halfwidth = 200
    radius = 100

routepoints = routeplan.calc_route(centrex, centrey, halfwidth, radius)

def drawroute(routepoints):
    for i, j in enumerate(routepoints):
        if i < 5:
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
            continue

    drawroute(routepoints)
    cv2.imshow("roi", finalframe)
    #cv2.imshow("mask", mask)
    cv2.imshow("contours", frame)
    # cv2.imshow("dif", diff)
    # keys should be Left, Right, Up, Down, Widen and Narrow, Extend and Contract which should set all routes
    key = cv2.waitKey(1000 / 12) & 0xff
    if key == ord("l"):
        centrex -= 1
        routepoints = routeplan.calc_route(centrex, centrey, halfwidth, radius)
    elif key == ord("r"):
        centrex += 1
        routepoints = routeplan.calc_route(centrex, centrey, halfwidth, radius)
    elif key == ord("q"):
        break
    # if \cv2.waitKey(1000 / 12) & 0xff == ord("q"):
    # break

cv2.destroyAllWindows()
camera.release()
