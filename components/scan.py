class Scan(object):
    """ Container for storing scan data & metadata """
    def __init__(self, data, metadata, image):
        self._data = data
        self._metadata = metadata
        self._image = image

    def __str__(self):
        """ string representation that can be written to a file, etc. """
        pass
