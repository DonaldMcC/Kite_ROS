import numpy as np
import cv2


# http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# define the list of boundaries
# boundaries = [([0, 0, 0], [40, 40, 40])]
# green
# boundaries = [([10, 100, 10], [100, 255, 100])]
# orange
# boundaries = [([0, 50, 100], [100, 200, 255])]

#iphone video
#contourmin = 3000
#wind
contourmin = 800

def kitemask(c, frame):
    if cv2.contourArea(c) < contourmin:
        return 0

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
        print (x,y,w,h, "cont", cv2.contourArea(c))
        print ("mask: ", np.sum(mask), totmask)
    return totmask
