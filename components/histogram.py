class Histogram(QPixmap):
    """ Class for drawing the 2D histogram on the scan page """
    def __init__(self):
        self._pixmap = None
        pass

    def draw(self):
        pass

    @property
    def pixmap(self):
        return self._pixmap
