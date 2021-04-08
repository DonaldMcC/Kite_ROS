from cvwriter import initwriter, writeframe

def writelogs(kite, base, control, config, height, width, fps):
    if config.logging and config.writer is None:
        # h, w = frame.shape[:2]
        # height, width = 480, 640 - removed should now be set above
        # height, width, channels = frame.shape
        config.writer = initwriter("record.avi", height, width, fps)
        origwriter = initwriter("origrecord.avi", height, width, fps)

    if config.logging:  # not saving this either as it errors on other screen
        writeframe(writer, frame, height, width)