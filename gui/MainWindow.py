from PyQt5.QtWidgets import QMainWindow, QLabel, QFrame, QSizePolicy

from PyQt5.QtGui import QPixmap, QPainter, QBrush, QColor, QLinearGradient, \
                        QPalette

from PyQt5.QtCore import pyqtSignal, pyqtSlot

from .ui_MainWindow import Ui_MainWindow

from matplotlib import cm
import numpy as np

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # signal connections
        self.ui.rbVCalib.toggled.connect(self.on_calib_rb_changed)
        self.ui.rbVScan.toggled.connect(self.on_scan_rb_changed)
        self.ui.rbBothScan.toggled.connect(self.on_scan_rb_changed)

        self.ui.lblServerMsg.hide()

        # calibration page defaults
        #self.ui.lblVCalib.setText('1. Not set\n2. Not set')
        #self.ui.lblHCalib.setText('1. Not set\n2. Not set')
        #self.ui.lblVolCalib.setText('1. Not set\n2. Not set')

        # disable calibration page manually (I don't know why I can't do this in Creator)
        self.ui.tab.setEnabled(False)
        self.ui.gbHCalib.setEnabled(False)

        # scan page
        # the only way these two lines will work is if the target
        # label has ignored QSizePolicy. Otherwise setting the pixmap
        # will update the size hint and cause the label to grow on every frame.
        self.ui.lblScanStatus.resizeEvent = self.on_scan_hist_resize
        self.ui.lblColorScale.resizeEvent = self.on_scan_hist_resize
        self._scan_color_scale = cm.viridis

        # local copy of data arrays to plot
        self._vdata = None

    def on_calib_rb_changed(self):
        if self.ui.rbVCalib.isChecked():
            self.ui.gbVCalib.setEnabled(True)
            self.ui.gbHCalib.setEnabled(False)
        else:
            self.ui.gbVCalib.setEnabled(False)
            self.ui.gbHCalib.setEnabled(True)

    def on_scan_rb_changed(self):
        if not (self.ui.rbVScan.isChecked() or self.ui.rbHScan.isChecked()):
            self.ui.gbVScan.setEnabled(True)
            self.ui.gbHScan.setEnabled(True)
        elif (self.ui.rbVScan.isChecked()):
            self.ui.gbVScan.setEnabled(True)
            self.ui.gbHScan.setEnabled(False)
        else:
            self.ui.gbVScan.setEnabled(False)
            self.ui.gbHScan.setEnabled(True)

    def on_scan_hist_resize(self, event=None):
        self.draw_scan_hist(self._vdata)
        self.draw_scan_color_scale()

    def draw_scan_color_scale(self):
        # I have no idea why, but without subtracting 2, this gradient bar
        # grows every update
        w = self.ui.lblColorScale.width() - 2
        h = self.ui.lblColorScale.height() - 2

        self._pxcs = QPixmap(w, h)
        self._pxcs.fill(QColor(255, 255, 255))
        painter = QPainter(self._pxcs)

        gradient = QLinearGradient(0, 0, w, 0)
        for i in range(256):
            r, g, b, _ = (int(255 * q) for q in self._scan_color_scale(i))
            gradient.setColorAt(i / 256., QColor(r, g, b))

        painter.fillRect(0, 0, w, h, gradient)
        self.ui.lblColorScale.setPixmap(self._pxcs)

    def draw_scan_hist(self, data):

        if data is None:
            return

        self._vdata = data

        w = float(self.ui.lblScanStatus.width())
        h = float(self.ui.lblScanStatus.height())

        self._px = QPixmap(int(w), int(h))
        self._px.fill(QColor(255, 255, 255))
        painter = QPainter(self._px)

        min_pos = min(data['pos'])
        max_pos = max(data['pos'])
        min_v = min(data['v'])
        max_v = max(data['v'])
        min_current = min(data['i'])[0]
        max_current = max(data['i'])[0]

        self.ui.lblColorScaleMin.setText('{0:.4e}'.format(min_current))
        self.ui.lblColorScaleMid.setText('{0:.4e}'.format((min_current + max_current) / 2.))
        self.ui.lblColorScaleMax.setText('{0:.4e}'.format(max_current))

        # number of unique position/voltage points
        n_pos_pts = len(np.unique(data['pos']))
        n_v_pts = len(np.unique(data['v']))

        rect_width = int(w / n_pos_pts)
        rect_height = int(h / n_v_pts)

        # add 1 px to some rectangles to fill in gaps due to rounding
        prev_x = 0
        prev_y = 0

        for i in range(len(data)):

            perc_pos = (data[i]['pos'] - min_pos) / (max_pos - min_pos)
            perc_v = (data[i]['v'] - min_v) / (max_v - min_v)

            if max_v == min_v or max_pos == min_pos or max_current == min_current:
                continue

            if not np.isnan(data[i]['i'][0]):
                perc_current = (data[i]['i'] - min_current) / (max_current - min_current)
                r, g, b, _ = (int(255 * q) for q in self._scan_color_scale(int(perc_current * 255)))
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

        # set pixmap onto the label widget
        self.ui.lblScanStatus.setPixmap(self._px)

    # widget aliases
    @property
    def btnConnect(self):
        return self.ui.btnConnect

    @property
    def txtIp(self):
        return self.ui.txtIP

    @property
    def txtPort(self):
        return self.ui.txtPort

    @property
    def lblServerMsg(self):
        return self.ui.lblServerMsg

    @property
    def lblPollRate(self):
        return self.ui.lblPollRate

    @property
    def lblV(self):
        return self.ui.lblV

    @property
    def lblVer(self):
        return self.ui.lblVerPos

    @property
    def lblHor(self):
        return self.ui.lblHorPos

    @property
    def lblCur(self):
        return self.ui.lblCur

    @property
    def tabCalib(self):
        return self.ui.tab

    @property
    def tabScan(self):
        return self.ui.tab_2

    @property
    def statusBar(self):
        return self.ui.statusBar
