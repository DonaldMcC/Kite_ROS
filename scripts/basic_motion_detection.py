#!/usr/bin/env python
# this will be the main module for configuring the desired route and
# for seeing and reporting back on the position and angle of
# the kite - it will also be the only video output from the package and consequently
# will display and also allow direct updating of the proposed route
# however there will also be a separate ros node that will allow setting the route
# and flying mode of the kit in due course and these will communicate via ROS messages

# inputs
# the module will support main input either input from a single webcam or from a video file initially - this may
# extend to rosbag files in future
#
# outputs
# the main output will be  a ROS message reporting the x and y coordinates of the kite and it should also be
# possible to record the input if required
#
# initial configuration
# file can be started with arguments and should then not prompt for input
# if started without arguments it should ask if webcam or file to be loaded
#
# while in flow it should be possible to
# 1 amend the flight mode
# 2 switch from sending actual kite position to manually controlled one
# 3 adjust the routing - it should default when the flight mode is changed
# 4 on playback it should be possible to go into slow motion

# standard library imports
import time

# library imports
import numpy as np
import cv2
import rospy

# modules
from routeplan import Kite, Base, Controls
from move_func import get_angle
from talker import kite_pos, kiteimage
from cvwriter import initwriter, writeframe


# this is just for display flight decisions will be elsewhere
def drawroute(route, centrex, centrey):
    global frame
    for i, j in enumerate(route):
        if i < len(route) - 1:
            cv2.line(frame, (j[0], j[1]), (route[i+1][0], route[i+1][1]),
                     (255, 0, 255), thickness=1, lineType=8, shift=0)
        else:
            cv2.line(frame, (j[0], j[1]), (route[0][0], route[0][1]),
                     (255, 0, 255), thickness=1, lineType=8, shift=0)
    cv2.line(frame, (centrex, 0), (centrex, centrey * 2),
             (255, 0, 0), thickness=2, lineType=8, shift=0)
    return


def drawcross(manx,many):
    global frame  #
    # stuff below was to allow angle calculation of angle - which may well
    # do once we have got direction of travel unpicked
    crosssize =10
    starthorx = manx-crosssize
    endhorx = manx + crosssize
    endhory = many
    starthory = many
    startvery = many - crosssize
    endvery = many + crosssize
    endverx = manx
    startverx = manx
    cv2.line(frame, (starthorx, starthory), (endhorx, endhory),
                 (255, 0, 255), thickness=2, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (endverx, endvery),
                 (255, 0, 255), thickness=2, lineType=8, shift=0)

    return



#Main routine start
# this will need to not happen if arguments are passed
source = 2 # change back to 1 to get prompt
while source not in {1, 2}:
    source = input('Key 1 for camera or 2 for source')
# should define source here
if source == 1:
    camera = cv2.VideoCapture(0)
    logging = 1
# camera=cv2.VideoCapture('IMG_0464.MOV')
else:
    logging = 0
    # TODO at some point will change this to current directory and append file - not urnger
    camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/choppedkite_horizshort.mp4')


#initiate class instances
control=Controls()
kite=Kite(control.centrex, control.centrey)
base=Base()
# Initialisation steps
es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
kernel = np.ones((5, 5), np.uint8)
background = None
imagemessage = kiteimage()

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
counter = 0

# http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
# boundaries = [([0, 0, 0], [40, 40, 40])]
# green
# boundaries = [([10, 100, 10], [100, 255, 100])]
# orange
# boundaries = [([0, 50, 100], [100, 200, 255])]

boundaries = [([0, 0, 0], [40, 40, 40]),
              ([10, 100, 10], [100, 255, 100]),
              ([0, 50, 100], [100, 200, 255])
              ]

for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    low = np.array(lower, dtype="uint8")
    upp = np.array(upper, dtype="uint8")


writer = None
cv2.startWindowThread()
cv2.namedWindow('contours')
fps = 15


while True:  # Main module loop
    ret, frame = camera.read()
    if background is None:
        background = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        background = cv2.GaussianBlur(background, (21, 21), 0)
        continue

    if logging and writer is None:
        # h, w = frame.shape[:2]
        height, width = 480, 640
        writer = initwriter("record.avi", height, width, fps)

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
    diff = cv2.absdiff(background, gray_frame)
    diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    diff = cv2.dilate(diff, es, iterations=2)
    image, cnts, hierarchy = cv2.findContours(diff.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # lets draw and move cross for manual flying
    if control.inputmodes[control.inputmode] == 'ManFly':
        drawcross(kite.manx, kite.many)

    # Aim to identify the kite and print the direction and so forth - maybe we do let this run
    # even when manual but we just output and draw position of manual x or whatever
    for c in cnts:
        if cv2.contourArea(c) < 1500:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        roi = frame[y:y+h, x:x+w]
        # loop over the boundaries
        mask = cv2.inRange(roi, low, upp)
        if np.sum(mask) > 1000:
            # outputframe = cv2.bitwise_and(roi, mask, mask=mask)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
            finalframe = frame[y:y+h, x:x+w]
            center = (x + (w//2), y + (h // 2))
            if control.inputmodes[control.inputmode] != 'ManFly':
                kite.pts.appendleft(center)
                kite.x = center[0]
                kite.y = center[1]
            else:
                kite.pts.appendleft((kite.manx, kite.many))

            # Min Area seems reasonable to get angle of kite
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)

            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

            continue

# start direction and analysiss
    for i in np.arange(1, len(kite.pts)):
        if kite.pts[i] is None:
            continue
        # check to see if enough points have been accumulated in the buffer
        if counter >= 10 and i == 1 and kite.pts and len(kite.pts) > 10 and kite.pts[10] is not None:
            # compute the difference between the x and  y  coordinates and re-initialize the direction
            # text variables
            kite.dX = kite.pts[i][0] - kite.pts[-10][0]
            kite.dY = kite.pts[i][1] - kite.pts[-10][1]
            (dirX, dirY) = ("", "")

            # ensure there is significant movement in the x-direction
            if np.abs(kite.dX) > 20:
                dirX = "East" if np.sign(kite.dX) == 1 else "West"

            # ensure there is significant movement in the y-direction
            if np.abs(kite.dY) > 20:
                dirY = "South" if np.sign(kite.dY) == 1 else "North"

                # handle when both directions are non-empty
            if dirX != "" and dirY != "":
                kite.direction = "{}-{}".format(dirY, dirX)

                # otherwise, only one direction is non-empty
            else:
                kite.direction = dirX if dirX != "" else dirY

            # otherwise, compute the thickness of the line and draw the connecting lines
            thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
            cv2.line(frame, kite.pts[i - 1], kite.pts[i], (0, 0, 255), thickness)

            # show the movement deltas and the direction of movement on the frame
            cv2.putText(frame, kite.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)
            cv2.putText(frame, "dx: {}, dy: {}".format(kite.dX, kite.dY),
                                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
            continue

        kite.kiteangle = get_angle(box, kite.dX, kite.dY)
        kite.targetdict['zone'] = kite.get_zone(control)
        cv2.putText(frame, str(int(kite.kiteangle)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)
        cv2.putText(frame, kite.targetdict['zone'], (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)
# end of directiocv2.putText(frame, str(int(kite.kiteangle)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)n and analysis


    kite.update_zone(control)
    kite.update_phase()

    if kite.changezone or kite.changephase:
        kite.update_target()

    drawroute(control.routepoints, control.centrex, control.centrey)

    cv2.putText(frame, control.modestring, (200, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    kite_pos(control.centrex, control.centrey, kite.kiteangle, kite.dX, kite.dY, 0, 0)
    # cv2.imshow("roi", finalframe)
    # cv2.imshow("mask", mask)
    cv2.imshow("contours", frame)
    kiteimage.pubimage(imagemessage, frame)
    counter += 1
    # writeframe(writer, frame, height, width)

    if counter % 100 == 0:
        pass
        # print(center)
        # print(kite.pts[10][0],kite.pts[10][1])
        # print(kite.dX, kite.dY)
        # print(kite.direction)
    # cv2.imshow("dif", diff)
    # keys should be Left, Right, Up, Down, Widen and Narrow, Extend and Contract which
    # should set all routes
    # Pause should hold for 5 secs
    key = cv2.waitKey(8) & 0xff
    # think there will be a mode option in here as well
    # one key changes mode and we would show the possible keys somewhere
    if key == ord("q"):
        break
    elif key != -1:
        routepoints = control.keyhandler(key, kite)


print("[INFO] cleaning up...")
cv2.destroyAllWindows()
camera.release()
# vs.stop() - no idea what this was
if writer is not None:
    writer.release()
