#!/usr/bin/env python

import cv2
import time

if __name__ == '__main__' :
    # Start default camera
    # video = cv2.VideoCapture(0);
    video = cv2.VideoCapture(r'/home/donald/catkin_ws/src/kite_ros/scripts/orig2605.avi')

    # Find OpenCV version
    (major_ver, minor_ver, subminor_ver) = cv2.__version__.split('.')

    # With webcam get(CV_CAP_PROP_FPS) does not work.
    # Let's see for ourselves.

    if int(major_ver) < 3:
        fps = video.get(cv2.cv.CV_CAP_PROP_FPS)
        print(f"Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {fps}")
    else:
        fps = video.get(cv2.CAP_PROP_FPS)
        print(f"Frames per second using video.get(cv2.CAP_PROP_FPS) : {fps}")

    # Number of frames to capture
    num_frames = 180

    print(f"Capturing {num_frames} frames")

    # Start time
    start = time.time()

    # Grab a few frames
    for i in xrange(0, num_frames) :
        ret, frame = video.read()


    # End time
    end = time.time()

    # Time elapsed
    seconds = end - start
    print(f"Time taken : {seconds} seconds")

    # Calculate frames per second
    fps  = num_frames / seconds
    print(f"Estimated frames per second : {fps}")

    # Release video
    video.release()