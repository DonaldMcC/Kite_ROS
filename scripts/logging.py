import csv
from cvwriter import initwriter, writeframe

def writelogheader(config, kite, base, control, height, width, fps):
    # THis should initialise the logging
    if config.logging and config.writer is None:
        # h, w = frame.shape[:2]
        # height, width = 480, 640 - removed should now be set above
        # height, width, channels = frame.shape
        config.writer = initwriter("record.avi", height, width, fps)
        config.csvwriter
        origwriter = initwriter("origrecord.avi", height, width, fps)


def writelogs(config, kite, base, control, frame, height, width, fps):
    if config.logging:  # not saving this either as it errors on other screen
        writeframe(config.writer, frame, height, width)

def close():
    pass