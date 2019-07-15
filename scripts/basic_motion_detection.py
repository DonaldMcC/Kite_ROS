#!/usr/bin/env python
# this will be the main module for configuring the desired route and
# for seeing and reporting back on the position and angle of
# the kite - it will also be the only video output from the package and consequently
# will display and also allow direct updating of the proposed route

# inputs
# the module will support main input either input from a single webcam or from a video file - this may
# extend to rosbag files in future and support for a second camera now seems required as cameras I have do not
# provide coverage of a sufficiently large angle of the sky - possibly building in capability to angle cameras
# during operation should be looked at
# there will also be input from a resistor which is linked to the kitebar and will need to be calibrated in
# advance - this will be received as kiteangle.data generally from arduino 
#
# outputs
# the main output will be a ROS message reporting the x and y coordinates of the kite the current angle of the
# control bar and the motor instruction to change the angle of the bar.  It should also be
# possible to record the input if required - the motor instruction is on motormsg.data
#
# initial configuration
# file can be started with arguments and should then not prompt for input
# if started without arguments it should ask if webcam or file to be loaded
#
# while in flow it should be possible to
# 1 amend the flight mode - which is the flight path we are looking for the kite to try and follow
# 2 switch from sending actual kite position to manually controlled one
# 3 adjust the routing - it should default when the flight mode is changed
# 4 on playback it should be possible to go into slow motion

# Currently working on option to support auxiliary camera - conceptually think this is ok
# but should be optional and if present we will switch to that when kite goes above top of main
# image ie for now auxiliary camera is always above and we try not to fly off the sides

# standard library imports
import numpy as np
import time
import cv2

#pyimagesearch imports
from imutils.video import VideoStream
import imutils

# kite_ros imports
from move_func import get_heading_points, get_angled_corners
from mainclasses import Kite, Controls, Base, Config
from move_func import get_angle
from talker import kite_pos, kiteimage, motor_msg
from cvwriter import initwriter, writeframe
from basic_listen_barangle import listen_kiteangle, get_barangle
from kite_funcs import kitemask, calcbarangle


# this is just for display flight decisions will be elsewhere
def drawroute(route, centrex, centrey):
    global frame
    for k, l in enumerate(route):
        if k < len(route) - 1:
            cv2.line(frame, (l[0], l[1]), (route[k + 1][0], route[k + 1][1]),
                     (0, 255, 70), thickness=2, lineType=8, shift=0)
        else:
            cv2.line(frame, (l[0], l[1]), (route[0][0], route[0][1]),
                     (0, 255, 70), thickness=2, lineType=8, shift=0)
    cv2.line(frame, (centrex, 0), (centrex, centrey * 2),
             (255, 0, 0), thickness=2, lineType=8, shift=0)
    return


def drawcross(manx, many, crosstype='Man', colour=(255, 0, 255)):
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


def drawkite(kite):
    global frame  #
    # stuff below was to allow angle calculation of angle - which may well
    # do once we have got direction of travel unpicked
    height = 20
    width = 20
    thickness = 2
    colour = (0, 255, 255)
    starthorx = kite.x - width
    endhorx = kite.x + width
    endhory = kite.y
    starthory = kite.y
    startvery = kite.y - height
    endvery = kite.y + height * 2
    endverx = kite.x
    startverx = kite.x
    starthorx, starthory = get_angled_corners(starthorx, starthory, kite.kiteangle, kite.x, kite.y, 'int')
    endhorx, endhory = get_angled_corners(endhorx, endhory, kite.kiteangle, kite.x, kite.y, 'int')

    startverx, startvery = get_angled_corners(startverx, startvery, kite.kiteangle, kite.x, kite.y, 'int')
    endverx, endvery = get_angled_corners(endverx, endvery, kite.kiteangle, kite.x, kite.y, 'int')

    cv2.line(frame, (starthorx, starthory), (endhorx, endhory),
             colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (endverx, endvery),
             colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (starthorx, starthory), (endverx, endvery),
             colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (endverx, endvery), (endhorx, endhory),
             colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (endhorx, endhory), (startverx, startvery),
             colour, thickness=thickness, lineType=8, shift=0)
    cv2.line(frame, (startverx, startvery), (starthorx, starthory),
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
    centx = outx + 60
    centy = 300
    radius = 60
    cv2.putText(frame, 'Base', (outx, centy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.circle(frame, (centx, centy), radius, (0, 255, 255), 2)
    cv2.putText(frame, 'Act:' + '{:5.1f}'.format(base.barangle), (outx + 85, centy + 100), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 0), 2)
    cv2.putText(frame, 'Tgt:' + '{:5.1f}'.format(base.targetbarangle), (outx - 15, centy + 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    display_line(base.targetbarangle, centx, centy, radius, (0, 255, 255))
    display_line(base.barangle, centx, centy, radius, (0, 255, 0))
    return


def display_line(angle, cx, cy, radius, colour):
    pointx, pointy = get_angled_corners(cx + radius, cy, angle, cx, cy)
    pointx = int(pointx)
    pointy = int(pointy)
    offx = cx - (pointx - cx)
    offy = cy - (pointy - cy)
    cv2.line(frame, (offx, offy), (pointx, pointy), colour, 2)
    return


# MAIN ROUTINE START
# this will need to not happen if arguments are passed
source = 2  # change back to 1 to get prompt
# iphone
# masklimit = 10000
# wind
masklimit = 1000
# config = 'yellowballs'  # alternative when base not present will also possibly be combo
# KITETYPE = 'indoorkite'  # need to comment out for external
KITETYPE = 'kite1'

# so thinking we have kite and controls, the video frame, posible sensor class
# and perhaps a configuration class
# controls setup self.inputmodes = ('Standard', 'SetFlight', 'ManFly')

# config = Config(setup='Manfly', source=1)
config = Config(setup='Standard', source=1)

while config.source not in {1, 2}:
    config.source = input('Key 1 for camera or 2 for source')
# should define source here
if config.source == 1:
    #camera = cv2.VideoCapture(-1)
    # probably need to go below route to do stitching but need to understand differences first
    camera = VideoStream(src=-1).start()
    config.logging = 1
    # camera=cv2.VideoCapture('IMG_0464.MOV')
else:
    # TODO at some point will change this to current directory and append file - not urgent
    camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/choppedkite_horizshort.mp4')
    #camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/newkite1.mp4')
    # camera = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/orig2605.avi')
    # camera = cv2.VideoCapture(r'/home/donald/Downloads/IMG_1545.MOV')
    print('video:', camera.grab())

# width = int(camera.stream.get(3))
# height = int(camera.stream.get(4))


# initiate class instances
control = Controls(config.setup)
actkite = Kite(control.centrex, control.centrey)
mankite = Kite(300, 400)
base = Base(updatemode=1, kitebarratio=3)

# Initialisation steps
es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
kernel = np.ones((5, 5), np.uint8)
background = None
imagemessage = kiteimage()

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
counter = 0
foundcounter = 0

if config.setup == 'Standard':  # otherwise not present
    listen_kiteangle() # this then updates base.barangle via the callback function
writer = None
cv2.startWindowThread()
cv2.namedWindow('contours')
fps = 15
# fps = camera.get(cv2.CV_CAP_PROP_FPS)

if control.config == "Manfly":
    kite = mankite
else:
    kite = actkite

while True:  # Main module loop
    # Read Frame
    ret, frame = camera.stream.read()
    # change above for videostream from pyimagagesearch
    # ret, frame = camera.read()
    if background is None:
        background = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        background = cv2.GaussianBlur(background, (21, 21), 0)
        continue

    if config.logging and writer is None:
        # h, w = frame.shape[:2]
        # height, width = 480, 640 - removed should now be set above
        height, width, channels = frame.shape
        writer = initwriter("record.avi", height, width, fps)
        origwriter = initwriter("origrecord.avi", height, width, fps)

    if config.logging:
        writeframe(origwriter, frame, height, width)

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
    diff = cv2.absdiff(background, gray_frame)
    # diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    diff = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]

    diff = cv2.dilate(diff, es, iterations=2)
    image, cnts, hierarchy = cv2.findContours(diff.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # lets draw and move cross for manual flying
    if control.config == "Manfly":
        if base.updatemode == 1:
            kite.kiteangle = base.barangle * base.kitebarratio
        drawkite(kite)
        kite.found = True

    # identify the kite
    if control.config != "Manfly" and config.setup == 'Standard':  # not detecting if in manual mode
        kite.found = False
        maxmask = -1
        index = -1
        for i, c in enumerate(cnts):
            mask = kitemask(c, frame, KITETYPE)
            if mask > maxmask:
                index = i
                maxmask = mask
            # (x, y, w, h) = cv2.boundingRect(c)
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 125, 0), 2)

        if maxmask > masklimit:
            kite.found = True
            c = cnts[index]
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
        # print index, maxmask

    if kite.found:
        tempstr = "Found: Yes"
    else:
        tempstr = "Found: No"

    base.barangle = get_barangle(kite, base, control)

    # Establish route
    if kite.changezone or kite.changephase or kite.routechange:
        kite.update_target(control.routepoints[0][0], control.routepoints[0][1],
                           control.centrex, control.maxy, control.routepoints[3][0], control.routepoints[3][1])

    # start direction and analysis - this will be a routine based on class
    getdirection(kite)
    kite.targetheading = get_heading_points((kite.x, kite.y), (kite.targetx, kite.targety))
    kite.targetangle = kite.targetheading

    kite.update_zone(control)
    kite.update_phase()
    base.targetbarangle = calcbarangle(kite, base, control)

    if kite.zone == 'Centre' or kite.phase == 'Xwind':
        kite.targetangle = get_heading_points((kite.x, kite.y), (kite.targetx, kite.targety))
    # display output
    # show the movement deltas and the direction of movement on the frame
    cv2.putText(frame, kite.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.putText(frame, "dx: {}, dy: {}".format(kite.dX, kite.dY),
                (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    cv2.putText(frame, "x: {}, y: {}".format(mankite.x, mankite.y),
                (180, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)

    cv2.putText(frame, "Act Angle:" + str(int(kite.kiteangle)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Tgt Angle:" + str(int(kite.targetangle)), (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Tgt Heading:" + str(int(kite.targetheading)), (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Mode:" + str(control.config), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Area:" + str(kite.contourarea), (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    drawroute(control.routepoints, control.centrex, control.centrey)
    drawcross(kite.targetx, kite.targety, 'Target', (0, 150, 250))

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

    motor_msg(base.barangle, base.targetbarangle)

    # cv2.imshow("roi", finalframe)
    # cv2.imshow("mask", mask)
    cv2.imshow("contours", frame)
    # below commented due to failing on 18.04
    # kiteimage.pubimage(imagemessage, frame)

    counter += 1
    if kite.found:
        foundcounter += 1

    if config.logging:  # not saving this either as it errors on other screen
        writeframe(writer, frame, height, width)

    # change to -1 for debugging
    # 10 seems to work better than 1 on virtualbox - not sure what the issue is
    key = cv2.waitKey(10) & 0xff
    # think there will be a mode option in here as well
    # one key changes mode and we would show the possible keys somewhere
    if key == ord("q"):
        break
    elif key != -1:
        routepoints = control.keyhandler(key, kite, base)
    time.sleep(control.slow)
    print(counter)
    # if counter > 633: # turn off expiry after so many frames
    #     print('found:', foundcounter)
    #     break

print("[INFO] cleaning up...")
cv2.destroyAllWindows()
camera.stop()
if writer is not None:
    writer.release()
