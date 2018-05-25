import datetime as dt

from PyQt5.QtGui import QPixmap, QPainter, QBrush, QColor, QLinearGradient, \
                        QPalette

from matplotlib import cm
import numpy as np

COMMENT_CHAR = '#'

class Scan(object):
    """ Container for storing scan data & metadata """
    def __init__(self, file, time, kind, stepper_points, vreg_points, data, metadata, has_image):
        self._file = file
        self._time = time
        self._kind = kind
        self._stepper_points = stepper_points
        self._vreg_points = vreg_points
        self._metadata = metadata
        self._data = data
        self._image_saved = has_image

        self._color_scale = cm.viridis
        self._histogram = None

    def preamble(self):
        pts = len(self._stepper_points) * len(self._vreg_points)

        preamble = ('{0} Emittance scan results\n'
                   '{0} {1} scan\n'
                   '{0} Time initiated: {2}\n'
                   '{0} Number of points: {3}\n'
                   '{0}\n'
                   '{0} User-defined metadata\n'
                   ).format(COMMENT_CHAR, self._kind, self._time, str(pts))

        for field, info in self._metadata.items():
            preamble += "{} {}: {}\n".format(COMMENT_CHAR, field, info['value'])

        preamble += "#\ntime,pos,v,i,i_rms\n"

        return preamble

    def make_histogram(self, width=500, height=500):
        """ Draw an image (2d histogram) of this scan's data to a QPixmap """
        w = width
        h = height
        if self._data is None:
            self._histogram = QPixmap(int(w), int(h))
            self._histogram.fill(QColor(255, 255, 255))
            return self._histogram

        self._histogram = QPixmap(int(w), int(h))
        self._histogram.fill(QColor(255, 255, 255))
        painter = QPainter(self._histogram)

        min_pos = min(self._data['pos'])
        max_pos = max(self._data['pos'])
        min_v = min(self._data['v'])
        max_v = max(self._data['v'])
        min_current = min(self._data['i'])[0]
        max_current = max(self._data['i'])[0]

        # number of unique position/voltage points
        n_pos_pts = len(np.unique(self._data['pos']))
        n_v_pts = len(np.unique(self._data['v']))

        rect_width = int(w / n_pos_pts)
        rect_height = int(h / n_v_pts)

        # add 1 px to some rectangles to fill in gaps due to rounding
        prev_x = 0
        prev_y = 0

        for i in range(len(self._data)):
            perc_pos = 0
            if min_pos != max_pos:
                # avoid divide by zero error if only one position point
                perc_pos = (self._data[i]['pos'] - min_pos) / (max_pos - min_pos)

            perc_v = 0
            if max_v != min_v:
                perc_v = (self._data[i]['v'] - min_v) / (max_v - min_v)

            if not np.isnan(self._data[i]['i'][0]):
                perc_current = 0
                if max_current != min_current:
                    perc_current = (self._data[i]['i'] - min_current) / (max_current - min_current)
                r, g, b, _ = (int(255 * q) for q in self._color_scale(int(perc_current * 255)))
                brush = QBrush(QColor(r, g, b))
            else:
                brush = QBrush(QColor(22,22,22))

            x = int(perc_pos * (w - rect_width))
            y = int((1.0 - perc_v) * (h - rect_height))

            height_fix = 0
            width_fix = 0
            x_fix = 0
            if prev_y - rect_height != y:
                height_fix = abs(y - (prev_y - rect_height))

            if prev_x + rect_width != x:
                width_fix = 2
                x_fix = -1

            painter.fillRect(x + x_fix, y, rect_width + width_fix, rect_height + height_fix, brush)

            prev_x = x
            prev_y = y

        return self._histogram

    def time_string(self, fmt='%Y-%m-%d %H:%M:%S'):
        return dt.datetime.strftime(self._time, fmt)

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, new_data):
        self._data = new_data

    def __str__(self):
        """ string representation that can be written to a file, etc. """
        pass

    @property
    def file(self):
        return self._file

    @property
    def kind(self):
        return self._kind

    @property
    def metadata(self):
        return self._metadata

    @property
    def image_saved(self):
        return self._image_saved
