from PyQt5.QtWidgets import QLabel, QVBoxLayout, QHBoxLayout, QFrame, \
                            QSizePolicy, QWidget
from PyQt5.QtGui import QPixmap, QPainter, QBrush, QColor, QLinearGradient, \
                        QPalette
from PyQt5.QtCore import Qt

from matplotlib import cm

import numpy as np

class QLine(QFrame):
    def __init__(self, orientation="horizontal"):
        super().__init__()
        if orientation == 'horizontal':
            self.setFrameShape(QFrame.HLine)
        else:
            self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)

class ScanViewWidget(QWidget):
    def __init__(self, scan=None):
        super().__init__()

        self._scan = scan

        self._plot_label = None
        self._color_scale_label = None
        self._scale_min_label = None
        self._scale_mid_label = None
        self._scale_max_label = None

        self._pix_color_scale = None
        self._pix_hist = None

        self._color_scale = cm.viridis

        self.initialize()

    @property
    def scan(self):
        return self._data

    @scan.setter
    def scan(self, newscan):
        self._scan = newscan
        self.draw_scan_hist()

    def initialize(self):
        """ Set up the QWidgets """
        # first container is a QHBoxLayout containing the "V" label and the plot surface
        hbox = QHBoxLayout()
        lblV = QLabel("V")
        lblV.setAlignment(Qt.AlignTop)
        lblV.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self._plot_label = QLabel()
        # Qt will try to rescale the label on each redraw unless we do this
        self._plot_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self._plot_label.setFrameShape(QFrame.Box)

        hbox.addWidget(lblV)
        hbox.addWidget(self._plot_label)

        # next we group this in a QVBoxLayout with the "mm" label
        vbox_inner = QVBoxLayout()
        lblmm = QLabel("mm")
        lblmm.setAlignment(Qt.AlignRight)
        lblmm.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        vbox_inner.addLayout(hbox)
        vbox_inner.addWidget(lblmm)

        # now add QLine, color scale surface, and color scale indicators
        vbox_outer = QVBoxLayout()
        ln = QLine()
        self._color_scale_label = QLabel()
        self._color_scale_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self._color_scale_label.setFrameShape(QFrame.Box)

        hbox_scale = QHBoxLayout()
        self._scale_min_label = QLabel('--')
        self._scale_min_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._scale_mid_label = QLabel('--')
        self._scale_mid_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self._scale_max_label = QLabel('--')
        self._scale_max_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        hbox_scale.addWidget(self._scale_min_label)
        hbox_scale.addWidget(self._scale_mid_label)
        hbox_scale.addWidget(self._scale_max_label)

        # put it all together
        vbox_outer.addLayout(vbox_inner)
        vbox_outer.addWidget(ln)
        vbox_outer.addWidget(self._color_scale_label)
        vbox_outer.addLayout(hbox_scale)

        self.setLayout(vbox_outer)

        # the only way these two lines will work is if the target
        # label has ignored QSizePolicy. Otherwise setting the pixmap
        # will update the size hint and cause the label to grow on every frame.
        self._color_scale_label.resizeEvent = self.on_scan_hist_resize
        self._plot_label.resizeEvent = self.on_scan_hist_resize

    def on_scan_hist_resize(self, event=None):
        self.draw_scan_color_scale()
        self.draw_scan_hist()

    def draw_scan_color_scale(self):
        # I have no idea why, but without subtracting 2, this gradient bar
        # grows every update. Maybe something to do with the border
        w = self._color_scale_label.width() - 2
        h = self._color_scale_label.height() - 2

        self._pix_color_scale = QPixmap(w, h)
        self._pix_color_scale.fill(QColor(255, 255, 255))
        painter = QPainter(self._pix_color_scale)

        gradient = QLinearGradient(0, 0, w, 0)
        for i in range(256):
            r, g, b, _ = (int(255 * q) for q in self._color_scale(i))
            gradient.setColorAt(i / 256., QColor(r, g, b))

        painter.fillRect(0, 0, w, h, gradient)
        self._color_scale_label.setPixmap(self._pix_color_scale)

    def draw_scan_hist(self):
        """ Draw the scan's 2d histogram in the form window and update the
            z-range labels.
        """
        w = float(self._plot_label.width())
        h = float(self._plot_label.height())

        if self._scan is None:
            px = QPixmap(int(w), int(h))
            px.fill(QColor(255, 255, 255))
            self._plot_label.setPixmap(px)
            self._scale_min_label.setText('--')
            self._scale_mid_label.setText('--')
            self._scale_max_label.setText('--')
            return

        self._plot_label.setPixmap(self._scan.make_histogram(w, h))

        min_pos = min(self._scan.data['pos'])
        max_pos = max(self._scan.data['pos'])
        min_v = min(self._scan.data['v'])
        max_v = max(self._scan.data['v'])
        min_current = min(self._scan.data['i'])[0]
        max_current = max(self._scan.data['i'])[0]

        self._scale_min_label.setText('{0:.4e}'.format(min_current))
        self._scale_mid_label.setText('{0:.4e}'.format((min_current + max_current) / 2.))
        self._scale_max_label.setText('{0:.4e}'.format(max_current))
