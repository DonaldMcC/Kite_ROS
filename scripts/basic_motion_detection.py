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
# 2 switch from sending actual kite position to manually controlled one - think this
# should be a new object
# 3 adjust the routing - it should default when the flight mode is changed
# 4 on playback it should be possible to go into slow motion

# standard library imports

# library imports
import numpy as np
import time
import cv2
from move_func import get_heading_points, get_angled_corners

# modules
from routeplan import Kite, Controls
from barset import Base, calcbarangle
from move_func import get_angle
from talker import kite_pos, kiteimage
from cvwriter import initwriter, writeframe
from basic_listen_barangle import listen_kitebase, get_barangle
from kite_funcs import kitemask


# this is just for display flight decisions will be elsewhere
def drawroute(route, centrex, centrey):
    global frame
    for i, j in enumerate(route):
        if i < len(route) - 1:
            cv2.line(frame, (j[0], j[1]), (route[i + 1][0], route[i + 1][1]),
                     (0, 255, 70), thickness=2, lineType=8, shift=0)
        else:
            cv2.line(frame, (j[0], j[1]), (route[0][0], route[0][1]),
                     (0, 255, 70), thickness=2, lineType=8, shift=0)
    cv2.line(frame, (centrex, 0), (centrex, centrey * 2),
             (255, 0, 0), thickness=2, lineType=8, shift=0)
    return


def drawcross(manx, many, crosstype='Man', colour=(255,0,255)):
    global frame  #
    # stuff below was to allow angle calculation of angle - which may well
    # do once we have got direction of travel unpicked
    if crosstype == 'Man':
        crosssize = 10
        thickness = 2
    else:  # Target for now
        crosssize = 9
        thickness = 3
    starthorx = manx - crosssize
    endhorx = manx + crosssize
    endhory = many
    starthory = many
    startvery = many - crosssize
    endvery = many + crosssize
    endverx = manx
    startverx = manx
    cv2.line(frame, (starthorx, starthory), (endhorx, endhory),
             colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (endverx, endvery),
             colour, thickness=thickness, lineType=8, shift=0)
    return


def getdirection(kte):
    # start direction and analysis - this will be a routine based on class
    for i in np.arange(1, len(kte.pts)):
        if kte.pts[i] is None:
            continue
        # check to see if enough points have been accumulated in the buffer
        if counter >= 10 and i == 1 and kte.pts and len(kte.pts) > 10 and kte.pts[10] is not None:
            # compute the difference between the x and  y  coordinates and re-initialize the direction
            # text variables
            kte.dX = kte.pts[i][0] - kte.pts[-10][0]
            kte.dY = kte.pts[i][1] - kte.pts[-10][1]
            (dirX, dirY) = ("", "")

            # ensure there is significant movement in the x-direction
            if np.abs(kte.dX) > 20:
                dirX = "East" if np.sign(kte.dX) == 1 else "West"

            # ensure there is significant movement in the y-direction
            if np.abs(kte.dY) > 20:
                dirY = "South" if np.sign(kte.dY) == 1 else "North"

                # handle when both directions are non-empty
            if dirX != "" and dirY != "":
                kte.direction = "{}-{}".format(dirY, dirX)

                # otherwise, only one direction is non-empty
            else:
                kte.direction = dirX if dirX != "" else dirY

            # otherwise, compute the thickness of the line and draw the connecting lines
            kte.thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
            cv2.line(frame, kte.pts[i - 1], kte.pts[i], (0, 0, 255), kte.thickness)
            continue

    return


def display_base():
    # output bar values - TODO change to graphical circle with actual bar and target bar
    centx = outx + 60
    centy = 300
    radius = 60
    cv2.putText(frame, 'Base', (outx, centy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.circle(frame, (centx, centy), radius, (0, 255, 255), 2)
    #cv2.putText(frame, 'Act:' + str(base.barangle), (outx + 100, centy+100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    cv2.putText(frame, 'Act:' + '{:5.1f}'.format(base.barangle), (outx + 85, centy + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 255, 0), 2)
    cv2.putText(frame, 'Tgt:' + '{:5.1f}'.format(base.targetbarangle), (outx - 15, centy + 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    display_line(base.targetbarangle * -1, centx, centy, radius, (0, 255, 255))
    display_line(base.barangle * -1, centx, centy, radius, (0, 255, 0))
    return

def display_line(angle, cx,cy, radius, colour):
    pointx, pointy = get_angled_corners(cx + radius, cy, angle, cx, cy)
    pointx = int(pointx)
    pointy = int(pointy)
    offx = cx - (pointx - cx)
    offy = cy - (pointy - cy)

    cv2.line(frame, (offx, offy), (pointx, pointy), colour , 2)
    return

# Main routine start
# this will need to not happen if arguments are passed
source = 2  # change back to 1 to get prompt
config = 'std' # this is the kitebase present and no balls on the lines
# config = 'yellowballs'  # alternative when base not present will also possibly be combo

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
    #camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/choppedkite_horizshort.mp4')
    #camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/orig2605.avi')
    camera = cv2.VideoCapture(r'/home/donald/Videos/IMG_1389Trim1.mp4')
    print 'video:',camera.grab()

width = int(camera.get(3))
height = int(camera.get(4))

# initiate class instances
control = Controls()
actkite = Kite(control.centrex, control.centrey)
mankite = Kite(300, 400)
base = Base()
# Initialisation steps
es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
kernel = np.ones((5, 5), np.uint8)
background = None
imagemessage = kiteimage()

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
counter = 0
foundcounter = 0

# http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
# boundaries = [([0, 0, 0], [40, 40, 40])]
# green
# boundaries = [([10, 100, 10], [100, 255, 100])]
# orange
# boundaries = [([0, 50, 100], [100, 200, 255])]

#boundaries = [([0, 0, 0], [40, 40, 40]),
#              ([10, 100, 10], [100, 255, 100]),
#              ([0, 50, 100], [100, 200, 255])
#              ]

boundaries = [([0, 0, 0], [70, 70, 70]),
              ([10, 100, 10], [100, 255, 100]),
              ([0, 50, 100], [120, 220, 255])
              ]


for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    low = np.array(lower, dtype="uint8")
    upp = np.array(upper, dtype="uint8")

if config == 'std':  # otherwise not present
    listen_kitebase()
writer = None
cv2.startWindowThread()
cv2.namedWindow('contours')
fps = 15
# fps = camera.get(cv2.CV_CAP_PROP_FPS)

while True:  # Main module loop
    ret, frame = camera.read()
    if background is None:
        background = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        background = cv2.GaussianBlur(background, (21, 21), 0)
        continue

    if logging and writer is None:
        # h, w = frame.shape[:2]
        #height, width = 480, 640 - removed should now be set above
        writer = initwriter("record.avi", height, width, fps)
        origwriter = initwriter("origrecord.avi", height, width, fps)
        
    if logging:
        writeframe(origwriter, frame, height, width)
        
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
    diff = cv2.absdiff(background, gray_frame)
    #diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    # below didnt work
    # diff = cv2.adaptiveThreshold(diff,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)

    diff = cv2.dilate(diff, es, iterations=2)
    image, cnts, hierarchy = cv2.findContours(diff.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if control.mode == 1:
        kite = mankite
    else:
        kite = actkite
        
    kite.found = False
    # lets draw and move cross for manual flying
    if control.mode == 1:
        drawcross(mankite.x, mankite.y)
        kite.found = True

    #max(enumerate(list), key=(lambda x: x[1]), default=-1)
    #results = max(enumerate(cnts), key=kitemask(x,frame,low,upp), default=-1)
    #print(results[0], results[1])

    # identify the kite
    if control.mode == 0 and config == 'std':  # not detecting if in manual mode
        maxmask = -1
        index = -1
        for i, c in enumerate(cnts):
            mask = kitemask(c, frame, low, upp)
            if mask>maxmask:
                index= i
                maxmask = mask

        if maxmask > 0:
            kite.found = True
            c=cnts[index]
            kite.contourarea = cv2.contourArea(cnts[index])
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
            finalframe = frame[y:y + h, x:x + w]
            center = (x + (w // 2), y + (h // 2))
            kite.pts.appendleft(center)
            kite.x = center[0]
            kite.y = center[1]

            # Min Area seems reasonable to get angle of kite
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)

            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
            kite.kiteangle = get_angle(box, kite.dX, kite.dY)
        #print index, maxmask

    # start direction and analysis - this will be a routine based on class
    getdirection(kite)

    # show the movement deltas and the direction of movement on the frame
    cv2.putText(frame, kite.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.putText(frame, "dx: {}, dy: {}".format(kite.dX, kite.dY),
                        (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    cv2.putText(frame, "x: {}, y: {}".format(mankite.x, mankite.y),
                        (180, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)

    kite.targetheading = get_heading_points((kite.x, kite.y), (kite.targetx, kite.targety))
    kite.targetangle = kite.targetheading


        
    cv2.putText(frame, "Act Angle:" + str(int(kite.kiteangle)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Tgt Angle:" + str(int(kite.targetangle)), (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Tgt Heading:" + str(int(kite.targetheading)), (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
    cv2.putText(frame, "Mode:" + str(control.mode), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)
    cv2.putText(frame, "Area:" + str(kite.contourarea), (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),2)


    kite.update_zone(control)
    kite.update_phase()


    if kite.changezone or kite.changephase or kite.routechange:
        kite.update_target(control.routepoints[0][0], control.routepoints[0][1],
                           control.centrex, control.maxy, control.routepoints[3][0], control.routepoints[3][1])

    drawroute(control.routepoints, control.centrex, control.centrey)
    drawcross(kite.targetx, kite.targety, 'Target', (0, 150, 250))

    base.barangle = get_barangle()
    base.targetbarangle = calcbarangle(kite, base, control)


    if kite.found:
        tempstr = "Found: Yes"
    else:
        tempstr = "Found: No"

    # output flight values
    outx = width - 180
    fontsize = 0.5
    cv2.putText(frame, 'Zone:' + kite.zone, (outx, 140), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, tempstr, (outx, 160), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, 'Mode:' + kite.mode, (outx, 180), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, 'Phase:' + kite.phase, (outx, 200), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)
    cv2.putText(frame, control.modestring, (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 0, 255), 2)

    display_base()

    kite_pos(kite.x, kite.y, kite.kiteangle, kite.dX, kite.dY, 0, 0)
    # cv2.imshow("roi", finalframe)
    # cv2.imshow("mask", mask)
    cv2.imshow("contours", frame)
    kiteimage.pubimage(imagemessage, frame)


    counter += 1
    if kite.found:
        foundcounter += 1
    if logging: # not saving this either as it errors on other screen

        writeframe(writer, frame, height, width)

    # change to -1 for debugging
    key = cv2.waitKey(1) & 0xff
    # think there will be a mode option in here as well
    # one key changes mode and we would show the possible keys somewhere
    if key == ord("q"):
        break
    elif key != -1:
        routepoints = control.keyhandler(key, kite)
    time.sleep(control.slow)
    print counter
    if counter > 633:
        print 'found:', foundcounter
        break

print("[INFO] cleaning up...")
cv2.destroyAllWindows()
camera.release()
# vs.stop() - no idea what this was
if writer is not None:
    writer.release()
