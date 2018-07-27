import numpy as np
import cv2


def kitemask(c, frame, low, upp):
    if cv2.contourArea(c) < 800:
        return 0
    (x, y, w, h) = cv2.boundingRect(c)
    roi = frame[y:y + h, x:x + w]
    # loop over the boundaries
    mask = cv2.inRange(roi, low, upp)
    return np.sum(mask)