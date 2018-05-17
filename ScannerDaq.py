#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Thomas Wester <twester@mit.edu>
#
# GUI for running the emittance scanners

import sys
import socket
import select
import time
import timeit
import queue
import threading
import json
from collections import deque

import numpy as np
import datetime as dt
from scipy import interpolate

from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QFileDialog, QDialog, \
                            QPushButton, QLabel, QHBoxLayout

from gui import MainWindow


def arange_inclusive(start, stop, step, decimals=6):
    """ Inclusive range of values given start, stop and step """
    res = [start]
    step_num = 1
    while res[-1] < stop:
        tmp = round(start + step_num * step, decimals)
        if tmp > stop:
            break
        res.append(tmp)
        step_num += 1

    return np.asarray(res)


def close_enough(val1, val2, epsilon=1e-6):
    """ Function for comparing floats """
    return abs(val1 - val2) < epsilon


def flatten(lst):
    """ Appends all nested elements in a list or tuple to a single list """
    # python trickery to flatten
    return list(sum(lst, ()))


# need this to be a global function
def calibrate(val, device, reverse=False, alt=False):
    """ Converts a value to a device's units based on its calibration """
    if None in flatten(device['calibration']) or len(device['calibration']) < 2:
        # if calibration not set, clean up argument & return
        if isinstance(val, np.ndarray):
            # we passed in a numpy array, so use numpy syntax
            return np.around(val, decimals=4).astype(device['type'])
        else:
            # we passed in a single value, so use default python syntax
            return device['type'](val)

    # sort calibration points by x value
    pts = sorted(device['calibration'], key=lambda x: x[0])
    xs = [pt[0] for pt in pts]
    ys = [pt[1] for pt in pts]

    # voltage regulator has two calibrations for read and set voltages
    # use the alt flag to switch between them
    if alt:
        xs = [pt[2] for pt in pts]

    if reverse:
        xs, ys = ys, xs

    f = interpolate.interp1d(xs, ys, fill_value="extrapolate")
    return np.around(f(val), decimals=4).astype(device['type'])


def clear_layout(layout):
    """ Recursively remove all Qt widgets from a layout """
    if layout is not None:
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
            else:
                clear_layout(item.layout())


class CouldNotConnectError(Exception):
    pass


class Daq(QObject):
    """ Class for collecting and organizing data from scans """
    sig_msg = pyqtSignal(str) # emit when ready to send command to server
    sig_new_pt = pyqtSignal(object, str) # emit when the scan completes a new point, string tells which type of scan
    sig_scan_started = pyqtSignal(str, str) # emits timestamps when scan starts
    sig_scan_finished = pyqtSignal(bool) # emits true if done, or false if there is still a horizontal scan to do
    sig_done = pyqtSignal() # emit when all scans have completed

    def __init__(self, devices):
        super().__init__()

        # local copy of device dict
        self._devices = devices

        self._v_scan_pts = None
        self._h_scan_pts = None

        self._vdata = None
        self._hdata = None

        if self._devices['vstepper']['scan'][0] is not None:
            self._v_scan_pts = [self._devices['vstepper']['scan'][0],
                                self._devices['vreg']['scan'][0]]

            # creates empty array with n rows and (time, pos, v, i) columns
            # n = # of vstepper points * # of vreg points
            self._vdata = np.empty(shape=(len(self._v_scan_pts[0]) * \
                                            len(self._v_scan_pts[1]), 1),
                                    dtype=[('time', object),
                                           ('pos', object),
                                           ('v', object),
                                           ('i', object)]
                                    )

            vsteps = len(self._v_scan_pts[1])
            for i, vtarget in enumerate(self._v_scan_pts[0]):
                for j, vtargetv in enumerate(self._v_scan_pts[1]):
                    self._vdata[i * vsteps + j]['pos'] = calibrate(vtarget, self._devices['vstepper'])
                    # convert set vals -> kV instead of read vals -> kV, so use alt flag
                    self._vdata[i * vsteps + j]['v'] = calibrate(vtargetv, self._devices['vreg'], alt=True)

            # initialize currents to nan so that the gui can draw null bins on 2d histogram
            self._vdata['i'] = np.nan

        # repeat above block for hstepper
        if self._devices['hstepper']['scan'][0] is not None:
            self._h_scan_pts = [self._devices['hstepper']['scan'][0],
                                self._devices['vreg']['scan'][1]]

            self._hdata = np.empty(shape=(len(self._h_scan_pts[0]) * \
                                            len(self._h_scan_pts[1]), 1),
                                    dtype=[('time', object),
                                           ('pos', object),
                                           ('v', object),
                                           ('i', object)]
                                    )

            hsteps = len(self._h_scan_pts[1])
            for i, htarget in enumerate(self._h_scan_pts[0]):
                for j, htargetv in enumerate(self._h_scan_pts[1]):
                    self._hdata[i * hsteps + j]['pos'] = calibrate(htarget, self._devices['hstepper'])
                    self._hdata[i * hsteps + j]['v'] = calibrate(htargetv, self._devices['vreg'], alt=True)

            # initialize currents to nan so that the gui can draw null bins on 2d histogram
            self._hdata['i'] = np.nan

        self._terminate = False

    def safe_move(self, vtarget, htarget, voltarget):
        """ Sends set command and waits until devices have reached set point """

        stepper = 'vstepper' if  vtarget is not None else 'hstepper'
        target = vtarget if vtarget is not None else htarget

        prev_val = self._devices[stepper]['value']
        # stepper motor is traditionally slower so try that first
        while not close_enough(self._devices[stepper]['value'], int(target), epsilon=1):
            # server expects setall <v stepper value> <h stepper value> <vreg value>s
            # only want to send command if we are stuck
            # for exiting mid-scan
            if self._terminate:
                return

            if self._devices[stepper]['value'] == prev_val:
                if vtarget is not None:
                    vtarget = int(vtarget)
                if htarget is not None:
                    htarget = int(htarget)

                msg = 'setall {} {} {}'.format(vtarget, htarget, voltarget)
                self.sig_msg.emit(msg)
            time.sleep(0.5)

        # now send some more commands if we are not at voltage target
        if voltarget is None:
            return

        prev_val = self._devices['vreg']['value']
        # convert set voltage to kV, then kV to read voltage
        calib_val = calibrate(
            calibrate(voltarget, self._devices['vreg'], alt=True),
            self._devices['vreg'], reverse=True)

        while not close_enough(self._devices['vreg']['value'], calib_val, epsilon=0.01):
            # for exiting mid-scan
            if self._terminate:
                return

            if self._devices['vreg']['value'] == prev_val:
                msg = 'setall {} {} {}'.format(None, None, voltarget)
                self.sig_msg.emit(msg)
            time.sleep(0.5)

    def scan(self, stepper, stepper_pts, vreg_pts):

        # make sure the other stepper is parked
        other = 'hstepper' if stepper == 'vstepper' else 'vstepper'
        other_prefix = 'hmove ' if stepper == 'vstepper' else 'vmove '
        #TODO
        #self.sig_msg.emit(other_prefix + 'SL SP')
        #while self._devices[other]['flag'] != 'MAX':
        #    time.sleep(0.1)

        vtarget, htarget = (stepper_pts[0], None) if stepper == 'vstepper' else (None, stepper_pts[0])
        targetv = vreg_pts[0]
        stepsv = len(vreg_pts)

        self.safe_move(vtarget, htarget, targetv)

        time.sleep(1)

        timestamp = dt.datetime.strftime(dt.datetime.now(), '%Y-%m-%d %H:%M:%S')
        kind = 'Vertical' if stepper == 'vstepper' else 'Horizontal'
        self.sig_scan_started.emit(timestamp, kind)

        # start scan
        for i, target in enumerate(stepper_pts):
            vtarget, htarget = (target, None) if stepper == 'vstepper' else (None, target)
            self.safe_move(vtarget, htarget, None)

            # for exiting mid-scan, safe_move is broken out-of and lands here
            if self._terminate:
                break

            # loop over voltages once we are at the correct position
            for j, targetv in enumerate(vreg_pts):
                self.safe_move(vtarget, htarget, targetv)

                if self._terminate:
                    break

                # once we are at the target values, we find average current
                t = dt.datetime.strftime(dt.datetime.now(), '%Y-%m-%d %H:%M:%S.%f')
                currents = []
                for k in range(50):
                    currents.append(self._devices['pico']['value'])
                    time.sleep(0.001)

                current = np.mean(currents)
                idx = i * stepsv + j
                data_frame = self._vdata if stepper == 'vstepper' else self._hdata
                data_frame[idx]['i'] = current
                data_frame[idx]['time'] = t
                pt = data_frame[idx][0]
                self.sig_new_pt.emit(pt, kind)

        if not self._terminate:
            # move stepper to parked position -- calibration point 1 is tuple with (steps, mm), use steps
            # also zero the voltage
            target = self._devices[stepper]['calibration'][0][0]
            self.safe_move(target, None, 0) if stepper == 'vstepper' else \
                self.safe_move(None, target, 0)

    def run(self):
        # do vertical scan
        if self._v_scan_pts is not None:
            self.scan('vstepper', self._v_scan_pts[0], self._v_scan_pts[1])

        # do horizontal scan
        if self._h_scan_pts is not None:
            # if we are here, let gui know we have finished vertical scan but
            # are doing a horizontal scan
            if self._v_scan_pts is not None:
                # don't emit unless we have already done the vertical scan
                self.sig_scan_finished.emit(False)

            self.scan('hstepper', self._h_scan_pts[0], self._h_scan_pts[1])

        self._terminate = True
        self.sig_done.emit()

        # now we are done with both scans
        self.sig_scan_finished.emit(True)

        # might need this line to tell Qt that this function is finished
        return

    @property
    def vdata(self):
        return self._vdata

    @property
    def hdata(self):
        return self._hdata

    def save_data(self):
        pass

    def send_data(self):
        pass

    def stop(self):
        pass

    def terminate(self):
        self._terminate = True


class Comm(QObject):

    sig_poll_rate = pyqtSignal(float)
    sig_data = pyqtSignal(bytes)
    sig_done = pyqtSignal() # emit if shutting down because of error

    def __init__(self, server_ip, server_port):
        super().__init__()

        self._ip = server_ip
        self._port = server_port

        self._buffer = 1024

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(1)

        self._polling_rate = 60 # Hz, 2 x the receive rate

        self._command_queue = queue.Queue()

        # try to connect to the server
        try:
            self._socket.connect((self._ip, self._port))
        except socket.error:
            raise CouldNotConnectError
            return

        self._terminate = False

    @property
    def server_ip(self):
        return self._ip

    @property
    def port(self):
        return self._port

    def poll(self):
        # send poll command or read from server if there is data
        recv = True # true if OK to send new info to server, false if waiting
        pollcount = 0
        sleep_time = 1. / self._polling_rate
        while not self._terminate:
            loop_start = timeit.default_timer()

            if recv:
                if pollcount == 0:
                    start_time = timeit.default_timer()

                # send any messages in the queue to the server
                if not self._command_queue.empty():
                    try:
                        cmd = self._command_queue.get_nowait()
                    except socket.timeout:
                        print('timeout')
                    self._socket.send(cmd.encode())
                    time.sleep(0.05)
                    recv = True
                    continue

                # otherwise just send a poll request
                else:
                    try:
                        self._socket.send(b'poll')
                    except socket.timeout:
                        print('timeout')

            ready = select.select([self._socket], [], [], 0)[0]
            if ready:
                data = self._socket.recv(self._buffer)
                if not data:
                    self.sig_done.emit()
                    self._terminate = True
                    break

                self.sig_data.emit(data)
                recv = True
                pollcount += 1
                if pollcount == 5:
                    rate = 5./ (timeit.default_timer() - start_time)
                    self.sig_poll_rate.emit(rate)
                    pollcount = 0
            else:
                recv = False

            # this line allows the thread to check its queued pyqt signals
            app.processEvents()

            net_sleep = sleep_time - (timeit.default_timer() - loop_start)
            if net_sleep > 0:
                time.sleep(net_sleep)

    @pyqtSlot(str)
    def add_message_to_queue(self, cmd):
        self._command_queue.put(cmd)

    def terminate(self):
        self._terminate = True


class DeviceManager(QObject):
    """ Class containing devices to be updated asynchronously from the gui """
    def __init__(self):
        super().__init__()

        device_name_list = ['pico', 'vstepper', 'hstepper', 'vreg']
        self._devices = dict()

        for name in device_name_list:
            self._devices[name] = {
                'value': 0.0, # real value returned from the server
                'type': float,
                'deque': deque(maxlen=10),
                'hasErr': False,
                'label': None,
                'name': '',
                'unit': '',
                'status': '',
                'fmt': '{0:.2f}',
                'calibration': [(None, None), (None, None)], # Linear interpolate between these points if set
                'scan': None # holds list of sets of points to scan
            }

        # stepper motors have special flags to indicate max/min positions
        self._devices['vstepper']['flag'] = ''
        self._devices['hstepper']['flag'] = ''

        self._devices['pico']['name'] = 'Current'
        self._devices['vstepper']['name'] = 'Vertical'
        self._devices['hstepper']['name'] = 'Horizontal'
        self._devices['vreg']['name'] = 'Voltage'

        self._devices['pico']['unit'] = 'A'
        self._devices['vstepper']['unit'] = 'steps'
        self._devices['hstepper']['unit'] = 'steps'
        self._devices['vreg']['unit'] = 'V'

        # exceptions to default values
        self._devices['pico']['fmt'] = '{0:.4e}'
        self._devices['vstepper']['status'] = 'Not calibrated'
        self._devices['hstepper']['status'] = 'Not calibrated'
        self._devices['vstepper']['type'] = int
        self._devices['hstepper']['type'] = int
        self._devices['vstepper']['fmt'] = '{:d}'
        self._devices['hstepper']['fmt'] = '{:d}'
        self._devices['vreg']['status'] = 'Not calibrated'
        self._devices['vreg']['calibration'] = []

    def on_data(self, data):
        data = data.decode("utf-8").split(' ')
        cur = float(data[0]) if data[0] != 'ERR' else 'ERR'
        vol = float(data[3]) if data[3] != 'ERR' else 'ERR'

        # stepper motors have custom error messages at limit switches
        vdata = data[1].split(',')
        hdata = data[2].split(',')

        ver = float(vdata[0]) if data[1] != 'ERR' else 'ERR'
        hor = float(hdata[0]) if data[2] != 'ERR' else 'ERR'

        if not isinstance(vdata, str):
            try:
                self._devices['vstepper']['flag'] = vdata[1]
            except IndexError:
                # no flag, so reset
                self._devices['vstepper']['flag'] = ''

        if not isinstance(hdata, str):
            try:
                self._devices['hstepper']['flag'] = hdata[1]
            except IndexError:
                # no flag, so reset
                self._devices['hstepper']['flag'] = ''

        self._devices['pico']['value'] = cur
        self._devices['vstepper']['value'] = ver
        self._devices['hstepper']['value'] = hor
        self._devices['vreg']['value'] = vol

        for device_name, info in self._devices.items():
            if info['value'] == 'ERR':
                info['hasErr'] = True
            else:
                info['hasErr'] = False
                info['deque'].append(
                        info['value']
                    )

    @property
    def devices(self):
        return self._devices


class DaqView:
    """ Handles interaction between GUI & server """
    def __init__(self):

        self._window = MainWindow.MainWindow()

        self._window.btnConnect.clicked.connect(self.connect_to_server)
        self._window.ui.btnExit.triggered.connect(self.exit)

        # load/save session
        self._window.ui.btnSaveSession.triggered.connect(self.save_session)
        self._window.ui.btnLoadSession.triggered.connect(self.load_session)

        # stepper manual controls & test buttons
        self._window.ui.btnRetract.clicked.connect(lambda: self.stepper_com('SL SP'))
        self._window.ui.btnExtend.clicked.connect(lambda: self.stepper_com('SL - SP'))
        self._window.ui.btnStop.clicked.connect(lambda: self.stepper_com('\x1b')) # esc key

        # calibration buttons
        self._window.ui.btnStartVCalib.clicked.connect(self.start_vstepper_calibration)
        self._window.ui.btnStartHCalib.clicked.connect(self.start_hstepper_calibration)
        self._window.ui.btnSetVreg.clicked.connect(self.set_vreg)
        self._window.ui.btnAddVregPoint.clicked.connect(self.add_vreg_calibration_point)

        # calibration page textboxes. Use eval to connect them all to the same function
        txtlist = ['txtVCalibUpper', 'txtVCalibLower', 'txtHCalibUpper', 'txtHCalibLower']
        for txt in txtlist:
            eval('self._window.ui.{}.textChanged.connect(self.on_calibration_textbox_changed)'.format(txt))

        self._window.ui.txtSetVreg.textChanged.connect(self.on_vreg_calibration_textbox_change)
        self._window.ui.txtVregCalib.textChanged.connect(self.on_vreg_calibration_textbox_change)

        # scan buttons
        self._window.ui.btnChooseFile.clicked.connect(self.choose_file)
        self._window.ui.btnStartStopScan.clicked.connect(self.scan)
        self._window.ui.rbVScan.toggled.connect(self.on_scan_rb_changed)
        self._window.ui.rbHScan.toggled.connect(self.on_scan_rb_changed)

        # testing buttons
        self._window.ui.btnVregTest.clicked.connect(self.test_vreg)

        # scan page textboxes all call the same checking function
        txtlist = ['txtVMinPos', 'txtVMaxPos', 'txtVStepPos', 'txtVMinV', 'txtVMaxV', 'txtVStepV',
                   'txtHMinPos', 'txtHMaxPos', 'txtHStepPos', 'txtHMinV', 'txtHMaxV', 'txtHStepV']

        for txt in txtlist:
            eval('self._window.ui.{}.textChanged.connect(self.on_scan_textbox_change)'.format(txt))

        self._window.ui.rbVScanStatus.toggled.connect(self.on_scan_status_rb_changed)

        # calibration variables
        self._vercalib = False
        self._horcalib = False
        self._vregcalib = False
        self._vreg_layouts = []  # list of Qt layouts created for each calibration point

        # device manager class allows devices to be updated independently of the gui thread
        self._dm = DeviceManager()

        # manually assign labels & properties to each device
        self._dm.devices['pico']['label'] = self._window.lblCur
        self._dm.devices['vstepper']['label'] = self._window.lblVer
        self._dm.devices['hstepper']['label'] = self._window.lblHor
        self._dm.devices['vreg']['label'] = self._window.lblV

        # testing block
        # self._dm.devices['vstepper']['calibration'] = [(400000, 20), (0, -20)]
        # self._dm.devices['hstepper']['calibration'] = [(50000, 20), (-50000, -20)]
        # self._vercalib = True
        # self._horcalib = True

        self.check_calibration()

        self._com_thread = QThread()
        self._dm_thread = QThread()

        self._dm.moveToThread(self._dm_thread)
        self._dm_thread.start()

        # default objects for saving scans
        self._scanfile = ''

        self.update_display_values()

    def test_vreg(self):
        val = np.random.uniform(-1, 1, 1)[0]
        self._comm.add_message_to_queue('vset vset {0:.2f}'.format(val))
        print('Setting voltage regulator to {0:.2f}'.format(val))

    def connect_to_server(self):
        if self._window.btnConnect.text() == 'Stop':
            self.shutdown_communication()
            return

        try:
            server_ip = self._window.txtIp.text()
            socket.inet_aton(server_ip)
        except socket.error:
            self._window.lblServerMsg.show()
            self._window.lblServerMsg.setText('Invalid IP entered!')
            return

        try:
            server_port = int(self._window.txtPort.text())
        except ValueError:
            self._window.lblServerMsg.show()
            self._window.lblServerMsg.setText('Invalid port entered!')
            return

        try:
            self._comm = Comm(server_ip, server_port)
        except CouldNotConnectError:
            self._window.lblServerMsg.show()
            self._window.lblServerMsg.setText('Could not connect to server!')
            return

        self._comm.sig_poll_rate.connect(self.on_poll_rate)
        self._comm.sig_data.connect(self._dm.on_data)
        self._comm.sig_data.connect(self.update_display_values)
        self._comm.sig_done.connect(self.shutdown_communication)
        self._comm.moveToThread(self._com_thread)
        self._com_thread.started.connect(self._comm.poll)
        self._com_thread.start()
        self._window.lblServerMsg.hide()

        self._window.statusBar.showMessage(
            'Server connection at {}:{} started.'.format(
                self._comm.server_ip, self._comm.port))

        self._window.btnConnect.setText('Stop')
        self._window.tabCalib.setEnabled(True)

    def shutdown_communication(self):
        try:
            self._daq.terminate()
        except AttributeError:
            # daq object may not have been created yet
            pass

        # try block for shutdown calibration should go here
        try:
            pass
            # self._calibration_thread quit somehow
        except AttributeError:
            pass

        try:
            self._comm.terminate()
            self._com_thread.quit()
            self._window.statusBar.showMessage(
                'Server connection at {}:{} closed.'.format(
                    self._comm.server_ip, self._comm.port))
        except AttributeError:
            # comm object may not have been created yet
            pass

        # reset gui elements
        self._window.btnConnect.setText('Connect')
        self._window.tabCalib.setEnabled(False)
        self._window.tabScan.setEnabled(False)

        # reset devices
        self._dm.devices['vstepper']['calibration'] = [(None, None), (None, None)]
        self._dm.devices['hstepper']['calibration'] = [(None, None), (None, None)]
        self._dm.devices['vreg']['calibration'] = []

        self.check_calibration()

    def on_poll_rate(self, rate):
        self._window.lblPollRate.setText('Polling rate: {0:.2f} Hz'.format(rate))

    def update_display_values(self):
        """ Updates the GUI with the latest device values """
        for device_name, info in self._dm.devices.items():
            if info['hasErr']:
                info['label'].setText('{0}: Error!'.format(info['name']))
            else:
                # <Device name>: <value> <unit> <optional message>
                info['label'].setText('{0}: {1} {2} {3}'.format(
                        info['name'], info['fmt'].format(
                            calibrate(info['value'],
                                self._dm.devices[device_name])),
                        info['unit'], '(%s)' % (info['status']) if \
                            info['status'] != '' else ''
                    )
                )

    # stepper buttons on home page call this function with fixed commands
    def stepper_com(self, cmd):
        if self._window.ui.rbVMove.isChecked():
            msg = 'vmove '
        else:
            msg = 'hmove '

        self._comm.add_message_to_queue(msg + cmd)

    def check_calibration(self):
        """ Determines if certain devices are calibrated or not """
        #if None not in flatten(self._dm.devices['vreg']['calibration']):
        if len(self._dm.devices['vreg']['calibration']) > 1:
            self._dm.devices['vreg']['status'] = ''
            self._dm.devices['vreg']['unit'] = 'kV'
            self._vregcalib = True
        else:
            self._dm.devices['vreg']['status'] = 'Not calibrated'
            self._dm.devices['vreg']['unit'] = 'V'
            self._vregcalib = False

        # are both vertical points set?
        if None not in flatten(self._dm.devices['vstepper']['calibration']):
            self._dm.devices['vstepper']['status'] = ''
            self._dm.devices['vstepper']['unit'] = 'mm'
            self._dm.devices['vstepper']['fmt'] = '{0:.2f}'
            self._dm.devices['vstepper']['type'] = float
            self._vercalib = True
        else:
            self._dm.devices['vstepper']['status'] = 'Not calibrated'
            self._dm.devices['vstepper']['unit'] = 'steps'
            self._dm.devices['vstepper']['fmt'] = '{:d}'
            self._dm.devices['vstepper']['type'] = int
            self._vercalib = False

        # same for horizontal points
        if None not in flatten(self._dm.devices['hstepper']['calibration']):
            self._dm.devices['hstepper']['status'] = ''
            self._dm.devices['hstepper']['unit'] = 'mm'
            self._dm.devices['hstepper']['fmt'] = '{0:.2f}'
            self._dm.devices['hstepper']['type'] = float
            self._horcalib = True
        else:
            self._dm.devices['hstepper']['status'] = 'Not calibrated'
            self._dm.devices['hstepper']['unit'] = 'steps'
            self._dm.devices['hstepper']['fmt'] = '{:d}'
            self._dm.devices['hstepper']['type'] = int
            self._horcalib = False

        if self._vregcalib and (self._vercalib or self._horcalib):
            self._window.tabScan.setEnabled(True)

            if not self._vercalib:
                self._window.ui.rbVScan.setEnabled(False)
                self._window.ui.rbVScanStatus.setEnabled(False)
                self._window.ui.rbHScan.setChecked(True)
                self._window.ui.rbHScanStatus.setChecked(True)
                self._window.ui.rbBothScan.setEnabled(False)
                return

            if not self._horcalib:
                self._window.ui.rbHScan.setEnabled(False)
                self._window.ui.rbHScanStatus.setEnabled(False)
                self._window.ui.rbVScan.setChecked(True)
                self._window.ui.rbVScanStatus.setChecked(True)
                self._window.ui.rbBothScan.setEnabled(False)
                return

            # if both calibrated, we can do a 2-axis scan
            self._window.ui.rbVScan.setEnabled(True)
            self._window.ui.rbHScan.setEnabled(True)
            self._window.ui.rbVScanStatus.setEnabled(True)
            self._window.ui.rbHScanStatus.setEnabled(True)
            self._window.on_scan_rb_changed()
            self._window.ui.rbBothScan.setEnabled(True)

    def save_session(self):
        """ Write window settings and device calibrations to a file """
        calibration_dict = {
            'vstepper': self._dm.devices['vstepper']['calibration'],
            'hstepper': self._dm.devices['hstepper']['calibration'],
            'vreg': self._dm.devices['vreg']['calibration'],
        }

        save_data = {
            'ui': self._window.session_properties,
            'devices': calibration_dict
        }

        with open('session.cfg', 'w') as f:
            json.dump(save_data, f, sort_keys=True, indent=4, separators=(', ', ': '))

    def load_session(self):
        data_dict = {}
        try:
            with open('session.cfg', 'r') as f:
                data_dict = json.loads(f.read())
        except IOError:
            # file does not exist
            return

        self._window.apply_session_properties(data_dict['ui'])
        for device_name, calibration in data_dict['devices'].items():
            # json doesn't have tuples??
            _calibration = [tuple(pt) for pt in calibration]
            self._dm.devices[device_name]['calibration'] = _calibration

        # this function also calls self.check_calibration()
        self.update_vreg_calibration()

    ############################
    # Calibration page functions
    ############################

    def on_calibration_textbox_changed(self):
        vertical_controls = {
            'upper': self._window.ui.txtVCalibUpper,
            'lower': self._window.ui.txtVCalibLower,
            'error': self._window.ui.lblVCalibError,
        }

        horizontal_controls = {
            'upper': self._window.ui.txtHCalibUpper,
            'lower': self._window.ui.txtHCalibLower,
            'error': self._window.ui.lblHCalibError,
        }

        for controls in [vertical_controls, horizontal_controls]:
            # only display error message if both textboxes are filled in
            if '' in [controls['upper'].text().strip(), controls['lower'].text().strip()]:
                return

            try:
                upper = float(controls['upper'].text().strip())
                lower = float(controls['lower'].text().strip())
            except ValueError:
                controls['error'].setText("Bad values entered for limits.")
                controls['error'].show()
                return

            if upper == lower:
                controls['error'].setText("Limits must be different.")
                controls['error'].show()
                return

            if upper < lower:
                controls['error'].setText("Upper limit must be greater than lower limit.")
                controls['error'].show()
                return

            # we pass all the checks, so hide the error label
            controls['error'].hide()

    def start_vstepper_calibration(self):
        try:
            upper = float(self._window.ui.txtVCalibUpper.text().strip())
            lower = float(self._window.ui.txtVCalibLower.text().strip())
        except ValueError:
            return

        if upper == lower:
            return

        if upper < lower:
            return

        self._calibration_thread = threading.Thread(
                target=self.set_stepper_calibration,
                args=('vstepper', upper, lower)
            )
        self._calibration_thread.start()
        self._window.tabCalib.setEnabled(False)
        self._window.tabScan.setEnabled(False)
        self._window.ui.gbSteppers.setEnabled(False)

    def start_hstepper_calibration(self):
        try:
            upper = float(self._window.ui.txtVCalibUpper.text().strip())
            lower = float(self._window.ui.txtVCalibLower.text().strip())
        except ValueError:
            return

        if upper == lower:
            return

        if upper < lower:
            return

        self._calibration_thread = threading.Thread(
                target=self.set_stepper_calibration,
                args=('hstepper', upper, lower)
            )
        self._calibration_thread.start()
        self._window.tabCalib.setEnabled(False)
        self._window.tabScan.setEnabled(False)
        self._window.ui.gbSteppers.setEnabled(False)

    def set_stepper_calibration(self, stepper, upper, lower):
        # send command to extend stepper
        # wait until stepper isnt moving anymore
        # set first point
        # send command to retract stepper
        # set second point

        waittime = 0.1 # TODO maybe this should depend on the polling rate?
        prefix = 'vmove ' if stepper == 'vstepper' else 'hmove '

        lower_steps = None
        upper_steps = None

        # extend stepper <=> move in negative direction
        msg = prefix + 'SL - SP'
        self._comm.add_message_to_queue(msg)
        while self._dm.devices[stepper]['flag'] != 'MIN':
            time.sleep(waittime)
        lower_steps = self._dm.devices[stepper]['value']

        # retract stepper <=> move in positive direction
        msg = prefix + 'SL SP'
        self._comm.add_message_to_queue(msg)
        while self._dm.devices[stepper]['flag'] != 'MAX':
            time.sleep(waittime)
        upper_steps = self._dm.devices[stepper]['value']

        self._dm.devices[stepper]['calibration'] = \
            [(upper_steps, upper), (lower_steps, lower)]

        self.check_calibration()
        self._window.tabCalib.setEnabled(True)
        self._window.tabScan.setEnabled(True)
        self._window.ui.gbSteppers.setEnabled(True)

    def on_vreg_calibration_textbox_change(self):
        # don't show error if user hasn't entered values
        if '' in [self._window.ui.txtSetVreg.text().strip(), \
                self._window.ui.txtVregCalib.text().strip()]:
            self._window.ui.lblVolCalibError.hide()
            return

        try:
            val = float(self._window.ui.txtSetVreg.text())
        except ValueError:
            self._window.ui.lblVolCalibError.setText("Bad input for SET value.")
            self._window.ui.lblVolCalibError.show()
            return

        try:
            val_in_kv = float(self._window.ui.txtVregCalib.text())
        except ValueError:
            self._window.ui.lblVolCalibError.setText("Bad input for kV value.")
            self._window.ui.lblVolCalibError.show()
            return

        self._window.ui.lblVolCalibError.hide()

    def set_vreg(self):
        # read value from text box
        try:
            val = float(self._window.ui.txtSetVreg.text())
        except ValueError:
            return

        # set the voltage regulator on the server
        self._comm.add_message_to_queue('vset vset {0:.2f}'.format(val))

    def add_vreg_calibration_point(self):
        # read value from text box
        try:
            val = float(self._window.ui.txtSetVreg.text())
        except ValueError:
            return

        # see what the user entered in the conversion text box
        try:
            val_in_kv = float(self._window.ui.txtVregCalib.text())
        except ValueError:
            return

        # make sure this calibration point is unique
        read_val = float('{0:.2f}'.format(self._dm.devices['vreg']['value']))
        if read_val in [pt[0] for pt in self._dm.devices['vreg']['calibration']]:
            self._window.ui.lblVolCalibError.setText("There is already a calibation point at the current value.")
            self._window.ui.lblVolCalibError.show()
            return

        self._window.ui.lblVolCalibError.hide()

        # otherwise add the point
        # format is: (what server reads, what user measures, what user sets)
        self._dm.devices['vreg']['calibration'].append((read_val, val_in_kv, val))

        # update the gui
        self.update_vreg_calibration()

    def remove_vreg_calibration_point(self, idx):
        del self._dm.devices['vreg']['calibration'][idx]
        self.update_vreg_calibration()

    def update_vreg_calibration(self):
        self.check_calibration()
        # clear all layouts
        for layout in self._vreg_layouts:
            clear_layout(layout)
            self._window.ui.gbVolCalib.layout().removeItem(layout)

        # add current set of calibration points
        for i, point in enumerate(self._dm.devices['vreg']['calibration']):

            lbl = QLabel('{}. {} V (read) = {} kV = {} V (set)'.format(
                i + 1, point[0], point[1], point[2]))
            btn = QPushButton('Delete')
            layout = QHBoxLayout()

            btn.clicked.connect(lambda: self.remove_vreg_calibration_point(i))

            layout.addWidget(lbl)
            layout.addStretch()
            layout.addWidget(btn)

            self._window.ui.gbVolCalib.layout().insertLayout(6 + i, layout)

            self._vreg_layouts.append(layout)

    def set_vreg_calibration(self):
        # Don't know what to do here yet.
        self._dm.devices['vreg']['calibration'] = \
            [(10.0, 1.5), (-10.0, -1.5)]

    #####################
    # Scan page functions
    #####################

    def choose_file(self):
        """ Called when user presses choose output file button """
        dlg = QFileDialog(self._window, 'Choose Data File', '' , 'CSV Files (*.csv)')
        if dlg.exec_() == QDialog.Accepted:
            fname = dlg.selectedFiles()[0]
            if fname[-4:] != '.csv':
                fname += '.csv'

            self._scanfile = fname
            #self._window.ui.lblSaveFile.setText('Saving output to:\n{}'.format(fname))

            # the start/stop scan button is enabled if the textboxes are valid
            # and the file is set
            self.on_scan_rb_changed()

    def on_scan_textbox_change(self):
        """ Function that calculates the number of scan points """
        self._window.ui.lblScanPoints.setText('Total points: --')
        self._window.ui.btnStartStopScan.setEnabled(False)

        v_points = 0
        h_points = 0
        self._dm.devices['vstepper']['scan'] = [None]
        self._dm.devices['hstepper']['scan'] = [None]
        self._dm.devices['vreg']['scan'] = [None, None]

        verr = False
        herr = False

        # calculate number of vectical points
        if self._window.ui.rbVScan.isChecked() or self._window.ui.rbBothScan.isChecked():
            # try to parse user input for all vertical textboxes
            txtnames = ['VMinPos', 'VMaxPos', 'VStepPos', 'VMinV', 'VMaxV', 'VStepV']
            ns = {} # local variables created with exec go in this "namespace".
                    # Otherwise exec creates globals in python 3 which could be bad
            try:
                # uh oh, exec ahead!!! Panic!!!!
                for txt in txtnames:
                    exec('{} = float(self._window.ui.txt{}.text())'.format(
                            txt.lower().split('pos')[0], txt), {'self':self}, ns)
                    # this creates vmin, vmax, vstep, vminv, vmaxv, vstepv variables in the namespace
            except ValueError:
                # only show error message if user has entered everything, so check for blank text fields
                # exec & eval in the same function??? Watch out!!!
                txtlist = [eval('self._window.ui.txt{}.text()'.format(_), {'self':self}) for _ in txtnames]
                if '' not in txtlist:
                    self._window.ui.lblVScanError.setText('Bad input.')
                    self._window.ui.lblVScanError.show()
                verr = True

            if not verr:
                # user input should be sequential, and having a step of 0 will cause a divide by zero error
                if (ns['vmin'] > ns['vmax']) or (ns['vminv'] > ns['vmaxv']) or \
                        ns['vmin'] == ns['vmax'] or ns['vminv'] == ns['vmaxv'] or \
                        ns['vstep'] == 0 or ns['vstepv'] == 0:
                    self._window.ui.lblVScanError.setText('Bad values entered.')
                    self._window.ui.lblVScanError.show()
                    verr = True

                if self._vercalib:
                    if ns['vmin'] < self._dm.devices['vstepper']['calibration'][1][1] or \
                            ns['vmax'] > self._dm.devices['vstepper']['calibration'][0][1]:

                        self._window.ui.lblVScanError.setText('Values exceed device limits.')
                        self._window.ui.lblVScanError.show()
                        verr = True

            if not verr:
                self._window.ui.lblVScanError.hide()

                # we create the list of vertical points, undoing the user-entered values
                steprange = arange_inclusive(ns['vmin'], ns['vmax'], ns['vstep'])
                volrange = arange_inclusive(ns['vminv'], ns['vmaxv'], ns['vstepv'])
                self._dm.devices['vstepper']['scan'] = \
                    [calibrate(steprange, self._dm.devices['vstepper'], reverse=True)]
                # alt flag indicates to use the set voltage instead of the read voltage
                self._dm.devices['vreg']['scan'][0] = \
                    calibrate(volrange, self._dm.devices['vreg'], reverse=True, alt=True)

                v_points = len(self._dm.devices['vstepper']['scan'][0]) * \
                            len(self._dm.devices['vreg']['scan'][0])

        # calculate number of horizontal points. Same as above block.
        # I don't like duplicating the code, but there doesn't seem to be a good
        # way to make it generalizable since all the Qt control names are different
        if self._window.ui.rbHScan.isChecked() or self._window.ui.rbBothScan.isChecked():
            # try to parse user input for all vertical textboxes
            txtnames = ['HMinPos', 'HMaxPos', 'HStepPos', 'HMinV', 'HMaxV', 'HStepV']
            ns = {}  # local variables created with exec go here
            try:
                for txt in txtnames:
                    exec('{} = float(self._window.ui.txt{}.text())'.format(
                            txt.lower().split('pos')[0], txt), {'self':self}, ns)
            except ValueError:
                # only show error message if user has entered everything
                txtlist = [eval('self._window.ui.txt{}.text()'.format(_), {'self':self}) for _ in txtnames]
                if '' not in txtlist:
                    self._window.ui.lblHScanError.setText('Bad input.')
                    self._window.ui.lblHScanError.show()
                herr = True

            if not herr:
                # user input should be sequential, and having a step of 0 will cause a divide by zero error
                if (ns['hmin'] > ns['hmax']) or (ns['hminv'] > ns['hmaxv']) or \
                        ns['hmin'] == ns['hmax'] or ns['hminv'] == ns['hmaxv'] or \
                        ns['hstep'] == 0 or ns['hstepv'] == 0:
                    self._window.ui.lblHScanError.setText('Bad values entered.')
                    self._window.ui.lblHScanError.show()
                    herr = True

                if self._horcalib:
                    if ns['hmin'] < self._dm.devices['hstepper']['calibration'][1][1] or \
                            ns['hmax'] > self._dm.devices['hstepper']['calibration'][0][1]:

                        self._window.ui.lblHScanError.setText('Values exceed device limits.')
                        self._window.ui.lblHScanError.show()
                        herr = True

            if not herr:
                self._window.ui.lblHScanError.hide()

                # create list of horizontal points
                steprange = arange_inclusive(ns['hmin'], ns['hmax'], ns['hstep'])
                volrange = arange_inclusive(ns['hminv'], ns['hmaxv'], ns['hstepv'])
                self._dm.devices['hstepper']['scan'] = \
                    [calibrate(steprange, self._dm.devices['hstepper'], reverse=True)]
                self._dm.devices['vreg']['scan'][1] = \
                    calibrate(volrange, self._dm.devices['vreg'], reverse=True, alt=True)

                h_points = len(self._dm.devices['hstepper']['scan'][0]) * \
                            len(self._dm.devices['vreg']['scan'][1])

        if not (verr or herr):
            # if we made it here, then we can update the # of points label
            self._window.ui.lblScanPoints.setText(
                    'Total points: {}'.format(h_points + v_points))

            # and if the user has selected a valid file, then they may start a scan
            if self._scanfile != '':
                self._window.ui.btnStartStopScan.setEnabled(True)

    def scan(self):
        """ Called when user presses the start scan button. Calculates the
            points to scan from the textboxes, and creates a Daq object
            which runs in its own thread
        """

        if self._window.ui.btnStartStopScan.text() == 'Stop Scan':
            self._window.ui.btnStartStopScan.setText('Start Scan')
            self._daq.terminate()
            self.stepper_com('\x1b')
            return

        self._window.ui.btnStartStopScan.setText('Stop Scan')

        # we call this again because the user can re-calibrate between scans
        # without changing the textboxes, so the scan points wouldn't be set
        # correctly without it
        self.on_scan_textbox_change()

        # create a new thead & daq instance
        self._scan_thread = QThread()
        self._daq = Daq(self._dm.devices)

        # connect Daq object signals
        self._daq.sig_msg.connect(self._comm.add_message_to_queue)
        self._daq.sig_scan_started.connect(self.on_scan_start)
        self._daq.sig_scan_finished.connect(self.on_one_scan_finished)
        self._daq.sig_done.connect(self.on_scan_finished)
        self._daq.sig_new_pt.connect(self.on_scan_pt)

        self._daq.moveToThread(self._scan_thread)
        self._scan_thread.started.connect(self._daq.run)
        self._scan_thread.finished.connect(self.shutdown_scan)
        self._scan_thread.start()

        # disable controls that could interfere with the scan
        self._window.tabCalib.setEnabled(False)
        self._window.enable_scan_controls(False)
        self._window.ui.gbSteppers.setEnabled(False)

    def on_scan_rb_changed(self):
        """ Update the total number of points when user changes scan selection """
        self.on_scan_textbox_change()

        if self._window.ui.rbBothScan.isChecked():
            if self._scanfile != '':
                _fname = self._scanfile[:-4]
                self._window.ui.lblSaveFile.setText('Saving output to:\n{}_v.csv\n{}_h.csv'.format(_fname, _fname))
        else:
            if self._scanfile != '':
                self._window.ui.lblSaveFile.setText('Saving output to:\n{}'.format(self._scanfile))

    def on_scan_status_rb_changed(self):
        """ Update the plot when the user switches views """
        if self._window.ui.rbVScanStatus.isChecked():
            try:
                self._window.draw_scan_hist(self._daq.vdata)
            except AttributeError:
                self._window.draw_scan_hist(None)
        else:
            try:
                self._window.draw_scan_hist(self._daq.hdata)
            except AttributeError:
                self._window.draw_scan_hist(None)

    def on_scan_start(self, time, kind):
        # create file preamble
        if kind == 'Vertical':
            pts = len(self._dm.devices['vstepper']['scan'][0]) * \
                        len(self._dm.devices['vreg']['scan'][0])

            # since we are here, switch the user view to the vertical status page
            self._window.ui.rbVScanStatus.setChecked(True)
        else:
            pts = len(self._dm.devices['hstepper']['scan'][0]) * \
                        len(self._dm.devices['vreg']['scan'][1])

        preamble = "# Emittance scan results\n" \
                   "# {} scan\n" \
                   "# Time initiated: {}\n" \
                   "# Number of points: {}\n" \
                   "time,pos,v,i\n".format(kind, time, str(pts))

        _file = self._scanfile
        if self._window.ui.rbBothScan.isChecked():
            # if we are doing 2 scans, then we need file_v.csv and file_h.csv
            _file = self._scanfile[:-4] + '_{}.csv'.format(kind[0].lower())

        with open(_file, 'w') as f:
            f.write(preamble)

    def on_scan_pt(self, pt, kind):
        """ update the file and plot label when a new point comes in """
        if self._window.ui.rbVScanStatus.isChecked():
            self._window.draw_scan_hist(self._daq.vdata)
        else:
            self._window.draw_scan_hist(self._daq.hdata)

        _file = self._scanfile
        if self._window.ui.rbBothScan.isChecked():
            # if we are doing 2 scans, then we need file_v.csv and file_h.csv
            _file = self._scanfile[:-4] + '_{}.csv'.format(kind[0].lower())

        # append data to text file
        with open(_file, 'a') as f:
            f.write(','.join([str(_) for _ in pt]))
            f.write('\n')

    def on_one_scan_finished(self, final):
        """ Function called after first scan (vertical) is finished.
            Only relevant during "both" scans
        """
        if not final:
            self._window.ui.rbHScanStatus.setChecked(True)

    def on_scan_finished(self):
        """ Clean up after scan, and tell the thread to quit
            This function is called when the stepper has returned to
            its original position.
        """
        self._daq.deleteLater()
        self._scan_thread.quit()
        self._scan_thread.wait()

    def shutdown_scan(self):
        """ Safely re-enable the gui once the thread is done. This is called
            when the QThread holding the DAQ object emits its finished signal.
        """
        self._window.ui.btnStartStopScan.setText('Start Scan')
        self._window.tabCalib.setEnabled(True)
        self._window.enable_scan_controls(True)
        self._window.ui.gbSteppers.setEnabled(True)

        # call this to make sure the proper controls stay disabled
        self.check_calibration()

    def run(self):
        self._window.show()

    def exit(self):
        self.shutdown_communication()
        self._window.close()


if __name__ == '__main__':
    app = QApplication([])

    dq = DaqView()
    dq.run()
    sys.exit(app.exec_())
