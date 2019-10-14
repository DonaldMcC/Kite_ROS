# USAGE
# python realtime_stitching.py
# https://www.pyimagesearch.com/2016/01/25/real-time-panorama-and-image-stitching-with-opencv/
# This is going to be primary method of configuring when using two cameras for kite flying
# and given current hardware two cameras looks like how we will try and move forward for now
# there are a couple of issues at present
# 1 I have no real idea how the devices are assigned src numbers
# 2 Once 2 sources have been found the left/top stream needs to be that and first attempt at flipping streams in
# software fails as they seem to stop but not restart - so let's try as swap the cables for now to adjust the streams
# 3 the right/bottom stream equally - my current setup is to stitch 2nd image below the first
#   but the opencv routine works for panoramic stitching so images are being rotated, stitched and then result
#   rotated back to get what I need - it may in due course make sense to change this and have bottom image as main
#   one because main kite flying should be fig8s on bottom camera and the stitching isn't perfect - however will only
#   actually make this change if required
# 4 I want the ability to save the transform/stitching warehouse - once the cameras are fixed on tripod this should stay
#   fixed and it could easily be quite hasslesome to try and get valid matrix when outside - so idea will be to use a
#   saved one and keep cameras fixed - probably just with a prompt to rebuild if required in this routine
#   having done that I think may well change basic_motion_detection to just use saved matrix by default

# import the necessary packages
from __future__ import print_function
from basicmotiondetector import BasicMotionDetector
from panorama import Stitcher
from imutils.video import VideoStream
import numpy as np
import datetime
import imutils
import time
import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, default='cachedH.npy',
                    help='Filename to load cached matrix')
parser.add_argument('--load', type=str, default='yes',
                    help='Do we load cached matrix')
args = parser.parse_args()

# initialize the video streams and allow them to warmup
print("[INFO] starting cameras...")
found1 = False
found2 = False
for i in range(0, 7):
    leftStream = VideoStream(src=i).start()
    # leftStream = cv2.VideoCapture(4)
    left = leftStream.read()
    if left is not None:
        print('got one stream on ', i)
        found1 = True
        break
    else:
        print(i)

for j in range(i+1, 9):
    # rightStream = VideoStream(usePiCamera=True).start()
    rightStream = VideoStream(src=j).start()
    right = rightStream.read()
    if right is not None:
        print('got second stream on ', j)
        found2 = True
        break
    else:
        print(j)
# rightStream = cv2.VideoCapture(2)
if found1 and found2:
    print('got two streams:', i, j)

else:
    print('Not got 2 streams so cannot continue - check your cameras are connected')
    exit()

# initialize the image stitcher, motion detector, and total
# number of frames read
stitcher = Stitcher()

if args.load == 'yes':
    try:
        stitcher.cachedH = np.load(args.file)
    except (FileNotFoundError, IOError):
        print("File not found continuing:", args.file)


motion = BasicMotionDetector(minArea=500)
total = 0

# loop over frames from the video streams
while True:
    # grab the frames from their respective video streams
    left = leftStream.read()
    right = rightStream.read()

    height, width, channels = left.shape

    left = cv2.transpose(left)
    right = cv2.transpose(right)

    # height, width, channels = left.shape
    # print('after', height,width)

    # resize the frames
    left = imutils.resize(left, width=480)
    right = imutils.resize(right, width=480)

    # stitch the frames together to form the panorama
    # IMPORTANT: you might have to change this line of code
    # depending on how your cameras are oriented; frames
    # should be supplied in left-to-right order
    result = stitcher.stitch([left, right])

    # no homograpy could be computed
    if result is None and total > 32:
        print("[INFO] homography could not be computed")
        # break
    if result is not None:
        # timestamp on the image
        timestamp = datetime.datetime.now()
        ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
        cv2.putText(result, ts, (10, result.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        flipped = cv2.flip(cv2.transpose(result), 1)
        # show the output images
        cv2.imshow("Result", result)

    # increment the total number of frames read and draw the
    total += 1
    cv2.imshow("Left-Top Frame", left)
    cv2.imshow("Right-Bottom Frame", right)
    key = cv2.waitKey(1) & 0xFF

    # if the `r` key was pressed, break from the loop
    if key == ord("r"):
        stitcher.cachedH = None

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# do a bit of cleanup
print("[INFO] cleaning up - perhaps we ask to save homography matrix here with default name?")
# then if argument on running it will load matrix if it exists while motion_detection will always load
# whatever filename is coded into it - perhaps with argparse to do something different if file exists and if not warn
# and compute seems ok  - want to minimise prompts in the field
np.save('cachedH', stitcher.cachedH)
cv2.destroyAllWindows()
leftStream.stop()
rightStream.stop()
