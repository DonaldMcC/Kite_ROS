from file_csv_out import CSVDataWrite
from cvwriter import initwriter, writeframe

def writelogheader(config):
    # THis should initialise the logging
    if config.logging:
        config.csvwriter = CSVDataWrite()
        config.csvriter.open_output()
        myheaders = config.getlogheaders()
        config.csvriter.write_data(myheaders)
    return


def writepictheader(config, height, width, fps):
    if config.logging and config.writer is None:
        # h, w = frame.shape[:2]
        # height, width = 480, 640 - removed should now be set above
        # height, width, channels = frame.shape
        config.writer = initwriter("record.avi", height, width, fps)


def writelogs(config, kite, base, control, frame, height, width, fps):
    if config.logging:  # not saving this either as it errors on other screen
        writeframe(config.writer, frame, height, width)
        mydata = config.getlogdata()
        config.csvwriter.write_data(mydata)

def closelogs(config):
    config.csvwriter.close_output()
