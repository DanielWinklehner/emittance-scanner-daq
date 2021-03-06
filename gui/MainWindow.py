from PyQt5.QtWidgets import QMainWindow, QLabel, QFrame, QSizePolicy, QGridLayout
from PyQt5.QtGui import QPixmap, QPainter, QBrush, QColor, QLinearGradient, \
                        QPalette
from PyQt5.QtCore import pyqtSignal, pyqtSlot

from .ui_MainWindow import Ui_MainWindow
from .widgets.DateTimePlotWidget import DateTimePlotWidget
from .widgets.ScanViewWidget import ScanViewWidget

from matplotlib import cm
import numpy as np
import pyqtgraph as pg

# global lists of controls because saving and loading use them
textboxes = [
    'txtVCalibUpper', 'txtVCalibLower', 'txtHCalibUpper', 'txtHCalibLower',
    'txtVMinPos', 'txtVMaxPos', 'txtVStepPos', 'txtVMinV', 'txtVMaxV', 'txtVStepV',
    'txtHMinPos', 'txtHMaxPos', 'txtHStepPos', 'txtHMinV', 'txtHMaxV', 'txtHStepV',
    'txtIP', 'txtPort', 'txtScanComDelay', 'txtVregMin', 'txtVregMax'
]

radiobuttons = [
    'rbVCalib', 'rbHCalib', 'rbVScanStatus', 'rbHScanStatus',
    'rbVScan', 'rbBothScan', 'rbHScan'
]

checkboxes = [
    'chkDanger', 'chkSaveImage', 'chkFieldMandatory', 'chkUpdatePlots'
]


class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # -- testing block --
        self._scan_view_widget = ScanViewWidget()
        self.ui.verticalLayout_15.insertWidget(1, self._scan_view_widget)
        # -- testing block --

        # pyqtgraph configuration
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 0.1)

        # signal connections
        self.ui.chkDanger.toggled.connect(lambda chk: self.ui.gbDanger.setEnabled(chk))
        self.ui.rbVCalib.toggled.connect(self.on_calib_rb_changed)
        self.ui.rbVScan.toggled.connect(self.on_scan_rb_changed)
        self.ui.rbVScanStatus.toggled.connect(self.on_scan_status_rb_changed)
        self.ui.rbBothScan.toggled.connect(self.on_scan_rb_changed)

        self.ui.lblServerMsg.hide()

        # disable calibration page manually (I don't know why I can't do this in Creator)
        self.ui.tab.setEnabled(False)
        self.ui.gbHCalib.setEnabled(False)

        # hide calibration page error labels
        self.ui.lblVCalibError.hide()
        self.ui.lblHCalibError.hide()
        self.ui.lblVolCalibError.hide()

        # scan page
        self.ui.lblScanError.hide()
        self.ui.lblMetadataError.hide()

        # the only way these two lines will work is if the target
        # label has ignored QSizePolicy. Otherwise setting the pixmap
        # will update the size hint and cause the label to grow on every frame.
        #self.ui.lblScanStatus.resizeEvent = self.on_scan_hist_resize
        #self.ui.lblColorScale.resizeEvent = self.on_scan_hist_resize
        #self._scan_color_scale = cm.viridis

        # local copy of scan to plot
        self._current_v_scan = None
        self._current_h_scan = None

        # monitoring plots
        self._vstepper_plot = DateTimePlotWidget()
        self._hstepper_plot = DateTimePlotWidget()
        self._vreg_plot = DateTimePlotWidget()
        self._pico_plot = DateTimePlotWidget()

        self._vstepper_settings = {
            'title': 'Vertical Stepper',
            'name': 'Position',
            'unit': 'Steps',
            'x': {'mode': 'auto', 'grid': True, 'log': False},
            'y': {'mode': 'auto', 'grid': True, 'log': False},
            'widget': {'color': '#0000FF'}
        }

        self._hstepper_settings = {
            'title': 'Horizontal Stepper',
            'name': 'Position',
            'unit': 'Steps',
            'x': {'mode': 'auto', 'grid': True, 'log': False},
            'y': {'mode': 'auto', 'grid': True, 'log': False},
            'widget': {'color': '#FF0000'}
        }

        self._vreg_settings = {
            'title': 'Voltage Regulator',
            'name': 'Voltage',
            'unit': 'V',
            'x': {'mode': 'auto', 'grid': True, 'log': False},
            'y': {'mode': 'auto', 'grid': True, 'log': False},
            'widget': {'color': '#11BB44'}
        }

        self._pico_settings = {
            'title': 'Picoammeter',
            'name': 'Current',
            'unit': 'A',
            'x': {'mode': 'auto', 'grid': True, 'log': False},
            'y': {'mode': 'auto', 'grid': True, 'log': False},
            'widget': {'color': '#AA3300'}
        }

        self._vstepper_plot.settings = self._vstepper_settings
        self._hstepper_plot.settings = self._hstepper_settings
        self._vreg_plot.settings = self._vreg_settings
        self._pico_plot.settings = self._pico_settings

        self._vstepper_plot.setMinimumSize(350, 300)
        self._hstepper_plot.setMinimumSize(350, 300)
        self._vreg_plot.setMinimumSize(350, 300)
        self._pico_plot.setMinimumSize(350, 300)

        self._plot_layout = QGridLayout()
        self._plot_layout.setContentsMargins(0, 0, 0, 0)
        self.ui.fmMonitor.setLayout(self._plot_layout)

        self._plot_layout.addWidget(self._vstepper_plot, 0, 0)
        self._plot_layout.addWidget(self._hstepper_plot, 0, 1)
        self._plot_layout.addWidget(self._vreg_plot, 1, 0)
        self._plot_layout.addWidget(self._pico_plot, 1, 1)

    ########################
    # Session saving/loading
    ########################

    def apply_session_properties(self, settings):
        self.resize(settings['window-width'], settings['window-height'])
        self.move(settings['window-pos-x'], settings['window-pos-y'])

        self.ui.splitScan.setSizes([settings['split-scan-first'],
                                    settings['split-scan-second']])

        self.ui.splitScanInfo.setSizes([settings['split-scaninfo-first'],
                                        settings['split-scaninfo-second']])

        for txt in textboxes:
            eval('self.ui.{}.setText("{}")'.format(txt, settings[txt]))

        for rb in radiobuttons:
            eval('self.ui.{}.setChecked({})'.format(rb, settings[rb]))

        for chk in checkboxes:
            eval('self.ui.{}.setChecked({})'.format(chk, settings[chk]))

    @property
    def session_properties(self):
        settings = {
            'split-scan-first': self.ui.splitScan.sizes()[0],
            'split-scan-second': self.ui.splitScan.sizes()[1],
            'split-scaninfo-first': self.ui.splitScanInfo.sizes()[0],
            'split-scaninfo-second': self.ui.splitScanInfo.sizes()[1],
            'window-pos-x': self.pos().x(),
            'window-pos-y': self.pos().y(),
            'window-width': self.frameSize().width(),
            'window-height': self.frameSize().height(),
        }

        for txt in textboxes:
            settings[txt] = eval('self.ui.{}.text()'.format(txt))

        for rb in radiobuttons:
            settings[rb] = eval('self.ui.{}.isChecked()'.format(rb))

        for chk in checkboxes:
            settings[chk] = eval('self.ui.{}.isChecked()'.format(chk))

        return settings

    ##########
    # Plotting
    ##########

    def update_plot_settings(self):
        self._vreg_plot.settings = self._vreg_settings
        self._vstepper_plot.settings = self._vstepper_settings
        self._hstepper_plot.settings = self._hstepper_settings
        self._pico_plot.settings = self._pico_settings

    def update_plot(self, device_name, device_data):
        # unzip list of tuples
        x, y = [pt[0] for pt in device_data], [pt[1] for pt in device_data]
        if device_name == 'vstepper':
            self._vstepper_plot.curve.setData(x, y)
        if device_name == 'hstepper':
            self._hstepper_plot.curve.setData(x, y)
        if device_name == 'vreg':
            self._vreg_plot.curve.setData(x, y)
        if device_name == 'pico':
            self._pico_plot.curve.setData(x, y)

    def on_calib_rb_changed(self):
        if self.ui.rbVCalib.isChecked():
            self.ui.gbVCalib.setEnabled(True)
            self.ui.gbHCalib.setEnabled(False)
        else:
            self.ui.gbVCalib.setEnabled(False)
            self.ui.gbHCalib.setEnabled(True)

    def enable_scan_controls(self, val):
        """ Enumerates controls that should be disabled when scan starts """
        self.ui.gbVScan.setEnabled(val)
        self.ui.gbHScan.setEnabled(val)
        self.ui.rbVScan.setEnabled(val)
        self.ui.rbHScan.setEnabled(val)
        self.ui.rbBothScan.setEnabled(val)
        self.ui.gbFileOptions.setEnabled(val)

        if val:
            # if we are re-enabling everything, make sure the groupboxes match
            # the selected radiobutton
            self.on_scan_rb_changed()

    def on_scan_rb_changed(self):
        if not (self.ui.rbVScan.isChecked() or self.ui.rbHScan.isChecked()):
            self.ui.gbVScan.setEnabled(True)
            self.ui.gbHScan.setEnabled(True)
            self.ui.rbVScanStatus.setEnabled(True)
            self.ui.rbHScanStatus.setEnabled(True)
        elif (self.ui.rbVScan.isChecked()):
            self.ui.gbVScan.setEnabled(True)
            self.ui.gbHScan.setEnabled(False)
            self.ui.rbVScanStatus.setChecked(True)
            self.ui.rbVScanStatus.setEnabled(True)
            self.ui.rbHScanStatus.setEnabled(False)
        else:
            self.ui.gbVScan.setEnabled(False)
            self.ui.gbHScan.setEnabled(True)
            self.ui.rbHScanStatus.setChecked(True)
            self.ui.rbVScanStatus.setEnabled(False)
            self.ui.rbHScanStatus.setEnabled(True)

    def clear_scans(self):
        self._current_v_scan = None
        self._current_h_scan = None

    def on_scan_status_rb_changed(self):
        if self.ui.rbVScanStatus.isChecked():
            self.draw_scan_hist(self._current_v_scan, 'Vertical')
        else:
            self.draw_scan_hist(self._current_h_scan, 'Horizontal')

    def draw_scan_hist(self, scan, kind):
        """ Wrapper function for self._scan_view_widget.draw_scan_hist() """

        #if scan is None:
        #    return

        # I don't like setting the variables here, but it's convenient since
        # we pass the scan object anyway
        if kind == 'Vertical':
            self._current_v_scan = scan
        else:
            self._current_h_scan = scan

        # don't update the drawing if the user if viewing the other scan
        if self.ui.rbVScanStatus.isChecked() and kind == 'Horizontal':
            return

        if self.ui.rbHScanStatus.isChecked() and kind == 'Vertical':
            return

        self._scan_view_widget.scan = scan

    @property
    # currently stored image
    def px(self):
        return self._px

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
