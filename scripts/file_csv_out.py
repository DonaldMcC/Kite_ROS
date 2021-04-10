import os

class CSVDataWrite:
    """ For logging CSV files to disk for further analysis
    """

    def __init__(self):
        self.file_ref = None

    def openOutput(self, file_name, path=None, reset_file=False):
        """ Opens a file for CSV data ouptut.

            If path is not specified (an empty string is given as
            the path) then the file will be opened in the current
            execution directory.

            If the reset_file parameter is False then file will be
            opened in append mode. If True then file will be opened
            in write mode and any existing data will be deleted if
            the file already exists.
        """

        # create the fully qualified path name
        file_path = os.path.join(path, file_name)
        fmode = "w" if reset_file else "a"
        try:
            self.file_ref = open(file_path, fmode)
        except Exception as e:
            print("%s" % str(e))
        return

    def closeOutput(self):
        """ Close - If file is not open then an error is returned.
        """
        try:
            self.file_ref.close()
        except Exception as e:
            print("%s" % str(e))
        return


    def writeCSVData(self, datavals):
        """ Populates CSV data
        """

        # if self.file_ref is None then a file has not yet been
        # opened for this instance
        if self.file_ref == None:
            return 'No file'
        try:
            self.file_ref.write(outstr)
        except Exception as e:
            print ("%s" % str(e))
        return
