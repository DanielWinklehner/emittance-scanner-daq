from PyQt5.QtWidgets import QMainWindow

from .ui_MainWindow import Ui_MainWindow

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # signal connections
        self.ui.rbVMove.toggled.connect(self.on_calib_rb_changed)
        self.ui.rbVScan.toggled.connect(self.on_scan_rb_changed)
        self.ui.rbBothScan.toggled.connect(self.on_scan_rb_changed)

        self.ui.lblServerMsg.hide()

        # calibration page defaults
        self.ui.lblVCalib.setText('1. Not set\n2. Not set')
        self.ui.lblHCalib.setText('1. Not set\n2. Not set')
        self.ui.lblVolCalib.setText('1. Not set\n2. Not set')

        # disable calibration page manually (I don't know why I can't do this in Creator)
        self.ui.tab.setEnabled(False)
        self.ui.gbHCalib.setEnabled(False)

    def on_calib_rb_changed(self):
        if self.ui.rbVMove.isChecked():
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
