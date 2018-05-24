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
                            QPushButton, QLabel, QHBoxLayout, QLineEdit

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


class Calibrator(QObject):
    """ Wrapper class for stepper calibration functions """
    sig_msg = pyqtSignal(str) # emit when ready to send command to server
    sig_done = pyqtSignal(str)

    def __init__(self, stepper, lower, upper, devices):
        super().__init__()
        self._stepper = stepper
        self._devices = devices

        # upper_steps, upper, lower_steps, lower
        self._calibration_data = [None, upper, None, lower]

    @property
    def calibration_data(self):
        return self._calibration_data

    def calibrate(self):
        # send command to extend stepper
        # wait until stepper isnt moving anymore
        # set first point
        # send command to retract stepper
        # set second point

        waittime = 0.1 # TODO maybe this should depend on the polling rate?
        prefix = 'vmove ' if self._stepper == 'vstepper' else 'hmove '

        # extend stepper <=> move in negative direction
        msg = prefix + 'SL - SP'
        self.sig_msg.emit(msg)
        while self._devices[self._stepper]['flag'] != 'MIN':
            time.sleep(waittime)
        self._calibration_data[2] = self._devices[self._stepper]['value']

        # retract stepper <=> move in positive direction
        msg = prefix + 'SL SP'
        self.sig_msg.emit(msg)
        while self._devices[self._stepper]['flag'] != 'MAX':
            time.sleep(waittime)
        self._calibration_data[0] = self._devices[self._stepper]['value']

        self.sig_done.emit(self._stepper)

class Daq(QObject):
    """ Class for collecting and organizing data from scans """
    sig_msg = pyqtSignal(str) # emit when ready to send command to server
    sig_new_pt = pyqtSignal(object, str) # emit when the scan completes a new point, string tells which type of scan
    sig_scan_started = pyqtSignal(str, str) # emits timestamps when scan starts
    sig_scan_finished = pyqtSignal(bool) # emits true if done, or false if there is still a horizontal scan to do
    sig_done = pyqtSignal() # emit when all scans have completed

    def __init__(self, devices, com_wait=0.5):
        super().__init__()

        self._devices = devices  # local copy of device dict
        self._com_wait = com_wait  # adjust wait time between sending set commands

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
                                           ('i', object),
                                           ('i_rms', object)]
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
                                           ('i', object),
                                           ('i_rms', object)]
                                    )

            hsteps = len(self._h_scan_pts[1])
            for i, htarget in enumerate(self._h_scan_pts[0]):
                for j, htargetv in enumerate(self._h_scan_pts[1]):
                    self._hdata[i * hsteps + j]['pos'] = calibrate(htarget, self._devices['hstepper'])
                    self._hdata[i * hsteps + j]['v'] = calibrate(htargetv, self._devices['vreg'], alt=True)

            # initialize currents to nan so that the gui can draw null bins on 2d histogram
            self._hdata['i'] = np.nan

        self._terminate = False

        # keep track of current point and number of total points to give status updates
        self._current_scan_direction = 'Vertical'
        self._total_points = 0
        if self._vdata is not None:
            self._total_points += len(self._vdata)
        if self._hdata is not None:
            self._total_points += len(self._hdata)

        self._point_times = []
        self._prev_point_time = None
        self._current_point_count = 0

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
            time.sleep(self._com_wait)

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
            time.sleep(self._com_wait)

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
                t = dt.datetime.now()
                currents = []
                for k in range(50):
                    currents.append(self._devices['pico']['value'])
                    time.sleep(0.001)

                current = np.mean(currents)
                current_rms = np.std(currents)
                idx = i * stepsv + j
                data_frame = self._vdata if stepper == 'vstepper' else self._hdata
                data_frame[idx]['i'] = current
                data_frame[idx]['i_rms'] = current_rms
                data_frame[idx]['time'] = dt.datetime.strftime(t, '%Y-%m-%d %H:%M:%S.%f')
                pt = data_frame[idx][0]
                self.sig_new_pt.emit(pt, kind)

                # status update
                self._current_point_count += 1
                if self._prev_point_time is not None:
                    time_delta = (t - self._prev_point_time).total_seconds()
                    self._point_times.append(time_delta)
                self._prev_point_time = t

        if not self._terminate:
            # move stepper to parked position -- calibration point 1 is tuple with (steps, mm), use steps
            # also zero the voltage
            target = self._devices[stepper]['calibration'][0][0]
            self.safe_move(target, None, 0) if stepper == 'vstepper' else \
                self.safe_move(None, target, 0)

    def run(self):
        # do vertical scan
        if self._v_scan_pts is not None:
            self._current_scan_direction = 'Vertical'
            self.scan('vstepper', self._v_scan_pts[0], self._v_scan_pts[1])

        # do horizontal scan
        if self._h_scan_pts is not None:
            # if we are here, let gui know we have finished vertical scan but
            # are doing a horizontal scan
            if self._v_scan_pts is not None:
                # don't emit unless we have already done the vertical scan
                self.sig_scan_finished.emit(False)

            self._current_scan_direction = 'Horizontal'
            self._prev_point_time = None
            self.scan('hstepper', self._h_scan_pts[0], self._h_scan_pts[1])

        self._terminate = True
        self.sig_done.emit()

        # now we are done with both scans
        self.sig_scan_finished.emit(True)

        # might need this line to tell Qt that this function is finished
        return

    def time_remaining(self):
        """ Estimate time left in scan by average time between points and
            number of points remainig (in seconds).
        """
        long_cutoff = self._com_wait * 1.5  # seconds (might need to be tuned,
                                            # and actually depends on how fast
                                            # the stepper moves, but this is
                                            # a much simpler calculation)


        v_scan_offset = 0  # index offset for self._point_times if 'both' scan
        if self._v_scan_pts is not None:
            v_scan_offset = len(self._vdata)

        scan_pts = None
        if self._current_scan_direction == 'Vertical':
            # estimate for vertical scan
            scan_pts = self._v_scan_pts
            slice_min = None
            slice_max = v_scan_offset
            pts_remaining = v_scan_offset - self._current_point_count
        else:
            scan_pts = self._h_scan_pts
            slice_min = v_scan_offset
            slice_max = None
            pts_remaining = self._total_points - self._current_point_count

        # only select points that are associated with the current scan
        numpy_point_times = np.asarray(self._point_times[slice_min:slice_max])

        # estimate the number of remaining long points (i.e. when stepper has to move)
        # this is just number of stepper points, -1 due to starting
        long_rate_estimate = (len(scan_pts[0]) - 1) / self.current_scan_total_points()
        fast_rate_estimate = 1. - long_rate_estimate

        # average time of long delay
        long_average = np.mean(numpy_point_times[np.where(numpy_point_times > long_cutoff)])
        if np.isnan(long_average):
            # we have no long times so we might as well guess
            long_average = long_cutoff * 2.  # a better estimate would use stepper speed

        # average time of fast delay
        fast_average = np.mean(numpy_point_times[np.where(numpy_point_times < long_cutoff)])
        if np.isnan(fast_average):
            fast_average = long_cutoff

        # time remaining prediction is weighted average of fast & long points
        weighted_average = long_rate_estimate * long_average + \
            fast_rate_estimate * fast_average

        # number of vertical points left times weighted average estimator
        return pts_remaining * weighted_average

    @property
    def current_point_count(self):
        return self._current_point_count

    def current_scan_point_count(self):
        v_offset = 0
        if self._v_scan_pts is not None:
            v_offset += len(self._vdata)

        if self._current_scan_direction == 'Vertical':
            return self._current_point_count
        else:
            return self._current_point_count - v_offset

    @property
    def current_scan_direction(self):
        return self._current_scan_direction

    @property
    def total_points(self):
        return self._total_points

    def current_scan_total_points(self):
        if self._current_scan_direction == 'Vertical':
            return len(self._vdata)
        else:
            return len(self._hdata)

    @property
    def vdata(self):
        return self._vdata

    @property
    def hdata(self):
        return self._hdata

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
        recv = True  # true if OK to send new info to server, false if waiting
        set_last = False  # flag to indicate if the last command was a set command instead of a poll
        pollcount = 0
        sleep_time = 1. / self._polling_rate
        while not self._terminate:
            loop_start = timeit.default_timer()

            if recv:
                if pollcount == 0:
                    start_time = timeit.default_timer()

                # send any messages in the queue to the server
                if not self._command_queue.empty() and not set_last:
                    try:
                        cmd = self._command_queue.get_nowait()
                    except socket.timeout:
                        print('timeout')
                        continue
                    self._socket.send(cmd.encode())
                    time.sleep(0.05)
                    recv = True
                    set_last = True
                    continue

                # otherwise just send a poll request
                else:
                    try:
                        self._socket.send(b'poll')
                    except socket.timeout:
                        print('timeout')
                    set_last = False

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

    def clear_queue(self):
        """ Thread-safe removal of all pending commands in queue """
        with self._command_queue.mutex:
            self._command_queue.queue.clear()
            self._command_queue.all_tasks_done.notify_all()
            self._command_queue.unfinished_tasks = 0

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
                'deque': deque(maxlen=500),
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

        # default HV configuration
        self._devices['vreg']['min'] = -1.5
        self._devices['vreg']['max'] = 1.5

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
                        (time.time(), calibrate(info['value'], info))
                    )

    @property
    def devices(self):
        return self._devices


class DaqView:
    """ Handles interaction between GUI & server """
    def __init__(self):
        self._connected_to_server = False

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
        self._window.ui.btnSetVregLimits.clicked.connect(self.set_vreg_limits)
        self._window.ui.txtVregMin.textChanged.connect(self.on_vreg_limit_text_changed)
        self._window.ui.txtVregMax.textChanged.connect(self.on_vreg_limit_text_changed)

        # calibration buttons
        self._window.ui.btnStartVCalib.clicked.connect(lambda: self.start_stepper_calibration('vstepper'))
        self._window.ui.btnStartHCalib.clicked.connect(lambda: self.start_stepper_calibration('hstepper'))
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
        self._window.ui.chkSaveImage.toggled.connect(self.on_save_image_checkbox_changed)
        self._window.ui.btnAddField.clicked.connect(self.add_metadata_field)
        self._window.ui.btnStartScan.clicked.connect(self.scan)
        self._window.ui.btnStopScan.clicked.connect(self.stop_scan)
        self._window.ui.rbVScan.toggled.connect(self.on_scan_rb_changed)
        self._window.ui.rbHScan.toggled.connect(self.on_scan_rb_changed)

        # testing buttons
        self._window.ui.btnVregTest.clicked.connect(self.test_vreg)

        # scan page textboxes all call the same checking function
        txtlist = [
            'txtVMinPos', 'txtVMaxPos', 'txtVStepPos', 'txtVMinV', 'txtVMaxV', 'txtVStepV',
            'txtHMinPos', 'txtHMaxPos', 'txtHStepPos', 'txtHMinV', 'txtHMaxV', 'txtHStepV'
        ]

        for txt in txtlist:
            eval('self._window.ui.{}.textChanged.connect(self.on_scan_textbox_change)'.format(txt))

        self._window.ui.rbVScanStatus.toggled.connect(self.on_scan_status_rb_changed)

        # calibration variables
        self._vercalib = False
        self._horcalib = False
        self._vregcalib = False
        self._vreg_layouts = []  # list of Qt layouts created for each calibration point

        # scan variables
        self._metadata = {}

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

        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self.update_display_values)

    def test_vreg(self):
        val = np.random.uniform(-1, 1)
        self._comm.add_message_to_queue('vset vset {0:.2f}'.format(val))
        print('Setting voltage regulator to {0:.2f}'.format(val))

    def on_vreg_limit_text_changed(self):
        self._window.ui.txtVregMin.setStyleSheet('')
        self._window.ui.txtVregMax.setStyleSheet('')

    def set_vreg_limits(self):
        try:
            _min = float(self._window.ui.txtVregMin.text().strip())
        except ValueError:
            self._window.ui.txtVregMin.setStyleSheet('background-color: #FFBBBB')
            return

        try:
            _max = float(self._window.ui.txtVregMax.text().strip())
        except ValueError:
            self._window.ui.txtVregMax.setStyleSheet('background-color: #FFBBBB')
            return

        self._window.ui.txtVregMin.setStyleSheet('background-color: #BBFFBB')
        self._window.ui.txtVregMax.setStyleSheet('background-color: #BBFFBB')

        self._dm.devices['vreg']['min'] = _min
        self._dm.devices['vreg']['max'] = _max

        # if user has set scan values already, update them with the new limits
        self.on_scan_textbox_change()

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
        #self._comm.sig_data.connect(self.update_display_values)
        self._comm.sig_done.connect(self.shutdown_communication)
        self._comm.moveToThread(self._com_thread)
        self._com_thread.started.connect(self._comm.poll)
        self._com_thread.start()
        self._update_timer.start(50)
        self._window.lblServerMsg.hide()

        self._window.statusBar.showMessage(
            'Server connection at {}:{} started.'.format(
                self._comm.server_ip, self._comm.port))

        self._window.btnConnect.setText('Stop')
        self._connected_to_server = True
        self._window.tabCalib.setEnabled(True)
        self._window.ui.lblServerStatus.setStyleSheet('color: green')
        self._window.ui.lblServerStatus.setText('Connected')

        self.check_calibration()  # case where user is reconnecting

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
        self._connected_to_server = False
        self._update_timer.stop()

        self._window.ui.lblServerStatus.setStyleSheet('color: red')
        self._window.ui.lblServerStatus.setText('Not connected')

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
                self._window.update_plot(device_name, info['deque'])

        app.processEvents()

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
            self._window._vreg_settings['unit'] = 'kV'
        else:
            self._dm.devices['vreg']['status'] = 'Not calibrated'
            self._dm.devices['vreg']['unit'] = 'V'
            self._window._vreg_settings['unit'] = 'V'
            self._vregcalib = False

        # are both vertical points set?
        if None not in flatten(self._dm.devices['vstepper']['calibration']):
            self._dm.devices['vstepper']['status'] = ''
            self._dm.devices['vstepper']['unit'] = 'mm'
            self._dm.devices['vstepper']['fmt'] = '{0:.2f}'
            self._dm.devices['vstepper']['type'] = float
            self._window._vstepper_settings['unit'] = 'mm'
            self._vercalib = True
        else:
            self._dm.devices['vstepper']['status'] = 'Not calibrated'
            self._dm.devices['vstepper']['unit'] = 'steps'
            self._dm.devices['vstepper']['fmt'] = '{:d}'
            self._dm.devices['vstepper']['type'] = int
            self._window._vstepper_settings['unit'] = 'Steps'
            self._vercalib = False

        # same for horizontal points
        if None not in flatten(self._dm.devices['hstepper']['calibration']):
            self._dm.devices['hstepper']['status'] = ''
            self._dm.devices['hstepper']['unit'] = 'mm'
            self._dm.devices['hstepper']['fmt'] = '{0:.2f}'
            self._dm.devices['hstepper']['type'] = float
            self._window._hstepper_settings['unit'] = 'mm'
            self._horcalib = True
        else:
            self._dm.devices['hstepper']['status'] = 'Not calibrated'
            self._dm.devices['hstepper']['unit'] = 'steps'
            self._dm.devices['hstepper']['fmt'] = '{:d}'
            self._dm.devices['hstepper']['type'] = int
            self._window._hstepper_settings['unit'] = 'Steps'
            self._horcalib = False

        self._window.update_plot_settings()

        if self._connected_to_server and self._vregcalib and \
                (self._vercalib or self._horcalib):
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
        else:
            self._window.tabScan.setEnabled(False)

    def save_session(self):
        """ Write window settings and device calibrations to a file """
        calibration_dict = {
            'vstepper': self._dm.devices['vstepper']['calibration'],
            'hstepper': self._dm.devices['hstepper']['calibration'],
            'vreg': self._dm.devices['vreg']['calibration'],
        }

        metadata_dict = {}
        for name, info in self._metadata.items():
            metadata_dict[name] = {
                'field': name,
                'value': info['value'],
                'mandatory': info['mandatory'],
                'order': info['order']
            }

        save_data = {
            'ui': self._window.session_properties,
            'devices': calibration_dict,
            'vreg': {'min': self._dm.devices['vreg']['min'],
                     'max': self._dm.devices['vreg']['max']},
            'metadata': metadata_dict,
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

        self._dm.devices['vreg']['min'] = data_dict['vreg']['min']
        self._dm.devices['vreg']['max'] = data_dict['vreg']['max']

        for metadata_name, info in data_dict['metadata'].items():
            self.add_metadata_field(info)

        # this function also calls self.check_calibration()
        self.update_vreg_calibration()
        self.on_scan_textbox_change()

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

    def start_stepper_calibration(self, device_name):
        """ Check user entry and launch stepper calibration thread """
        vertical_settings = {
            'upper': self._window.ui.txtVCalibUpper,
            'lower': self._window.ui.txtVCalibLower,
            'device': 'vstepper'
        }

        horizontal_settings = {
            'upper': self._window.ui.txtHCalibUpper,
            'lower': self._window.ui.txtHCalibLower,
            'device': 'hstepper'
        }

        settings = vertical_settings if device_name == 'vstepper' else horizontal_settings

        try:
            upper = float(settings['upper'].text().strip())
            lower = float(settings['lower'].text().strip())
        except ValueError:
            return

        if upper == lower:
            return

        if upper < lower:
            return

        self._calibrator = Calibrator(settings['device'], lower, upper, self._dm.devices)
        self._calibrator.sig_msg.connect(self._comm.add_message_to_queue)
        self._calibrator.sig_done.connect(self.on_calibration_done)

        self._calibration_thread = QThread()
        self._calibrator.moveToThread(self._calibration_thread)
        self._calibration_thread.started.connect(self._calibrator.calibrate)
        self._calibration_thread.finished.connect(self.on_calibration_finished)
        self._calibration_thread.start()

        self._window.tabCalib.setEnabled(False)
        self._window.tabScan.setEnabled(False)
        self._window.ui.gbVCalib.setEnabled(False)
        self._window.ui.gbHCalib.setEnabled(False)

    def on_calibration_done(self, stepper):
        """ Clean up everything associated with calibration thread
            and set the new calibration in the device manager.
        """
        upper_steps, upper, lower_steps, lower = self._calibrator.calibration_data
        self._calibrator.deleteLater()
        self._calibration_thread.quit()
        self._calibration_thread.wait()

        self._dm.devices[stepper]['calibration'] = \
            [(upper_steps, upper), (lower_steps, lower)]

        _range = upper - lower
        if stepper == 'vstepper':
            self._window._vstepper_settings['y']['min'] = lower - 0.1 * _range
            self._window._vstepper_settings['y']['max'] = upper + 0.1 * _range
            self._window._vstepper_settings['y']['mode'] = 'manual'
        else:
            self._window._hstepper_settings['y']['min'] = lower - 0.1 * _range
            self._window._hstepper_settings['y']['max'] = upper + 0.1 * _range
            self._window._hstepper_settings['y']['mode'] = 'manual'

        self._window.update_plot_settings()

    def on_calibration_finished(self):
        """ This function is called once the thread has been safely deleted """
        self.check_calibration()
        self._window.tabCalib.setEnabled(True)

        self._window.ui.gbVCalib.setEnabled(True)
        self._window.ui.gbHCalib.setEnabled(True)

        self._window.on_calib_rb_changed()

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

        self._vreg_layouts = []

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
            #self.check_scan_valid()
            self.on_scan_rb_changed()

    def on_save_image_checkbox_changed(self):
        self.on_scan_rb_changed()

    def add_metadata_field(self, info_dict={}):
        # if nothing is passed as an argument, create field from gui
        if not info_dict:
            field = self._window.ui.txtFieldName.text().strip()
            if field == '':
                self._window.ui.lblMetadataError.setText('Invalid field name.')
                self._window.ui.lblMetadataError.show()
                return
            elif field in self._metadata.keys():
                self._window.ui.lblMetadataError.setText('Field name already exists.')
                self._window.ui.lblMetadataError.show()
                return

            # scan won't start unless this field is nonempty
            mandatory = self._window.ui.chkFieldMandatory.isChecked()
            order = len(self._metadata.keys())
            value = ''
        else:
            # otherwise create field from the dictionary info
            field = info_dict['field']
            value = info_dict['value']
            mandatory = info_dict['mandatory']
            order = info_dict['order']

        self._window.ui.lblMetadataError.hide()

        lbl = QLabel('{}:'.format(field))
        if mandatory:
            lbl.setStyleSheet('font-weight: bold')
        txt = QLineEdit()
        txt.setText(value)
        txt.textChanged.connect(lambda: self.on_metadata_text_changed(field))
        btnDel = QPushButton('Delete')
        btnDel.clicked.connect(lambda: self.delete_metadata(field))

        btnMoveUp = QPushButton('+')
        btnMoveUp.clicked.connect(lambda: self.move_metadata(field, direction='up'))

        btnMoveDown = QPushButton('-')
        btnMoveDown.clicked.connect(lambda: self.move_metadata(field, direction='down'))

        # create new control for the gui
        self._metadata[field] = {
            'value': value,
            'order': order,
            'mandatory': mandatory,
            'label': lbl,
            'text': txt,
            'delete_button': btnDel,
            'move_up_button': btnMoveUp,
            'move_down_button': btnMoveDown,
        }

        self.update_metadata()
        self.on_scan_rb_changed()

    def on_metadata_text_changed(self, field):
        self._metadata[field]['value'] = self._metadata[field]['text'].text()
        self.on_scan_rb_changed() # this checks if the scan is valid

    def update_metadata(self):
        clear_layout(self._window.ui.layoutMetadata)

        sorted_fields = sorted([(name, info) for name, info in self._metadata.items()],
            key=lambda x: x[1]['order'])

        # add controls to form
        for field, info in sorted_fields:
            self._window.ui.layoutMetadata.addWidget(info['label'], info['order'], 0)
            self._window.ui.layoutMetadata.addWidget(info['text'], info['order'], 1)

            if info['order'] == 0:
                info['move_up_button'].setEnabled(False)
            else:
                info['move_up_button'].setEnabled(True)

            self._window.ui.layoutMetadata.addWidget(info['move_up_button'], info['order'], 2)

            if info['order'] == sorted_fields[-1][1]['order']:
                # last index
                info['move_down_button'].setEnabled(False)
            else:
                info['move_down_button'].setEnabled(True)

            self._window.ui.layoutMetadata.addWidget(info['move_down_button'], info['order'], 3)

            self._window.ui.layoutMetadata.addWidget(info['delete_button'], info['order'], 4)

    def move_metadata(self, field_name, direction):
        old_index = self._metadata[field_name]['order']
        new_index = old_index - 1 if direction == 'up' else old_index + 1

        shiftleft = False
        if new_index > old_index:
            shiftleft = True

        for field, info in self._metadata.items():
            if shiftleft:
                # shift all fields between old_index and new_index by -1
                if info['order'] > old_index and info['order'] <= new_index:
                    self._metadata[field]['order'] -= 1

            else:
                # shift all fields between old_index and new_index by +1
                if info['order'] < old_index and info['order'] >= new_index:
                    self._metadata[field]['order'] += 1

        self._metadata[field_name]['order'] = new_index

        self.update_metadata()

    def delete_metadata(self, field_name):
        del self._metadata[field_name]

        # re-set ordering of all fields
        sorted_fields = sorted([(name, info) for name, info in self._metadata.items()],
            key=lambda x: x[1]['order'])

        new_index = 0
        for field_name, info in sorted_fields:
            self._metadata[field_name]['order'] = new_index
            new_index += 1

        self.update_metadata()
        self.on_scan_rb_changed()

    def on_scan_textbox_change(self):
        """ Function that calculates the number of scan points """
        self._window.ui.lblScanPoints.setText('Total points: --')
        self._window.ui.btnStartScan.setEnabled(False)
        self._window.ui.btnStopScan.setEnabled(False)

        self._dm.devices['vstepper']['scan'] = [None]
        self._dm.devices['hstepper']['scan'] = [None]
        self._dm.devices['vreg']['scan'] = [None, None]

        vertical_settings = {
            'txts': ['VMinPos', 'VMaxPos', 'VStepPos', 'VMinV', 'VMaxV', 'VStepV'],
            'rb': self._window.ui.rbVScan,
            'error': self._window.ui.lblVScanError,
            'error_bool': False,
            'calib_state': self._vercalib,
            'device': 'vstepper',
            'vreg_index': 0,
            'points': 0
        }

        horizontal_settings = {
            'txts': ['HMinPos', 'HMaxPos', 'HStepPos', 'HMinV', 'HMaxV', 'HStepV'],
            'rb': self._window.ui.rbHScan,
            'error': self._window.ui.lblHScanError,
            'error_bool': False,
            'calib_state': self._horcalib,
            'device': 'hstepper',
            'vreg_index': 1,
            'points': 0
        }

        for settings in [vertical_settings, horizontal_settings]:
            if settings['rb'].isChecked() or self._window.ui.rbBothScan.isChecked():
                # try to parse user input for all vertical textboxes
                ns = {} # local variables created with exec go in this "namespace".
                        # Otherwise exec creates globals in python 3 which could be bad
                try:
                    # uh oh, exec ahead!!! Panic!!!!
                    for txt in settings['txts']:
                        exec('{} = float(self._window.ui.txt{}.text())'.format(
                                '_' + txt.lower().split('pos')[0][1:], txt), {'self':self}, ns)
                        # this creates _min, _max, _step, _minv, _maxv, _stepv variables in the namespace
                except ValueError:
                    # only show error message if user has entered everything, so check for blank text fields
                    # exec & eval in the same function??? Watch out!!!
                    txtlist = [eval('self._window.ui.txt{}.text()'.format(_), {'self':self}) for _ in settings['txts']]
                    if '' not in txtlist:
                        settings['error'].setText('Bad input.')
                        settings['error'].show()
                    settings['error_bool'] = True

                if not settings['error_bool']:
                    # user input should be sequential, and having a step of 0 will cause a divide by zero error
                    if (ns['_min'] > ns['_max']) or (ns['_minv'] > ns['_maxv']) or \
                            ns['_min'] == ns['_max'] or ns['_minv'] == ns['_maxv'] or \
                            ns['_step'] == 0 or ns['_stepv'] == 0:
                        settings['error'].setText('Bad values entered.')
                        settings['error'].show()
                        settings['error_bool'] = True

                    if settings['calib_state']:
                        # stepper motors are limited by their endpoint calibration
                        # vreg can have non-endpoint callibration points
                        # but has a physical limit
                        if ns['_min'] < self._dm.devices[settings['device']]['calibration'][1][1] or \
                                ns['_max'] > self._dm.devices[settings['device']]['calibration'][0][1] or \
                                ns['_minv'] < self._dm.devices['vreg']['min'] or \
                                ns['_maxv'] > self._dm.devices['vreg']['max']:

                            settings['error'].setText('Values exceed device limits.')
                            settings['error'].show()
                            settings['error_bool'] = True

                if not settings['error_bool']:
                    settings['error'].hide()

                    # we create the list of vertical points, undoing the user-entered values
                    steprange = arange_inclusive(ns['_min'], ns['_max'], ns['_step'])
                    volrange = arange_inclusive(ns['_minv'], ns['_maxv'], ns['_stepv'])
                    self._dm.devices[settings['device']]['scan'] = \
                        [calibrate(steprange, self._dm.devices[settings['device']], reverse=True)]
                    # alt flag indicates to use the set voltage instead of the read voltage
                    self._dm.devices['vreg']['scan'][settings['vreg_index']] = \
                        calibrate(volrange, self._dm.devices['vreg'], reverse=True, alt=True)

                    settings['points'] = len(self._dm.devices[settings['device']]['scan'][0]) * \
                        len(self._dm.devices['vreg']['scan'][settings['vreg_index']])

        if not (vertical_settings['error_bool'] or horizontal_settings['error_bool']):
            # if we made it here, then we can update the # of points label
            self._window.ui.lblScanPoints.setText(
                    'Total points: {}'.format(
                        vertical_settings['points'] + horizontal_settings['points']))

            if self.check_scan_valid():
                self._window.ui.btnStartScan.setEnabled(True)

    def stop_scan(self):
        self._comm.clear_queue()
        self._daq.terminate()
        self.stepper_com('\x1b')

    def check_scan_valid(self):
        # check if we have a file name
        if self._scanfile == '':
            return False

        # check if we have filled in all mandatory metadata fields
        for field_name, info in self._metadata.items():
            if info['mandatory']:
                if info['value'] == '':
                    return False

        return True

    def on_scan_rb_changed(self):
        """ Update the total number of points when user changes scan selection """
        self.on_scan_textbox_change()

        text = ''

        if self._scanfile == '':
            return

        _fname = self._scanfile[:-4]

        if self._window.ui.rbBothScan.isChecked():
            text = 'Saving output to:\n{0}_v.csv\n{0}_h.csv'.format(_fname)
            if self._window.ui.chkSaveImage.isChecked():
                text += '\nSaving images to:\n{0}_v.png\n{0}_h.png'.format(_fname)

        else:
            text = 'Saving output to:\n{}'.format(self._scanfile)
            if self._window.ui.chkSaveImage.isChecked():
                text += '\nSaving an image to:\n{}.png'.format(_fname)

        self._window.ui.lblSaveFile.setText(text)

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

    def scan(self):
        """ Called when user presses the start scan button. Calculates the
            points to scan from the textboxes, and creates a Daq object
            which runs in its own thread
        """

        # we call this again because the user can re-calibrate between scans
        # without changing the textboxes, so the scan points wouldn't be set
        # correctly without it
        self.on_scan_textbox_change()

        self._window.ui.btnStartScan.setEnabled(False)
        self._window.ui.btnStopScan.setEnabled(True)

        # if the user has modified the COM delay setting
        com_delay = 0.5
        try:
            com_delay = float(self._window.ui.txtScanComDelay.text().strip())
        except:
            pass

        # create a new thead & daq instance
        self._scan_thread = QThread()
        self._daq = Daq(self._dm.devices, com_wait=com_delay)

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
        self._window.ui.gbVCalib.setEnabled(False)
        self._window.ui.gbHCalib.setEnabled(False)

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
                   "#\n" \
                   "# User-defined metadata\n".format(kind, time, str(pts))

        for field, info in self._metadata.items():
            preamble += "# {}: {}\n".format(field, info['value'])

        preamble += "#\ntime,pos,v,i,i_rms\n"

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

        # status updates
        time_remaining = self._daq.time_remaining()
        if not np.isnan(time_remaining) and self._daq.current_scan_point_count() > 5:
            m, s = divmod(time_remaining, 60)
            if m > 0:
                self._window.ui.lblScanTime.setText('Est. time remaining: {:d} minute{} and {:d} seconds'.format(
                    int(m), 's' if int(m) > 1 else '', int(s)))
            else:
                self._window.ui.lblScanTime.setText('Est. time remaining: {:d} second{}'.format(
                    int(s), 's' if int(s) != 1 else ''))
        else:
            self._window.ui.lblScanTime.setText('Est. time remaining: --')

        self._window.ui.lblScanProgress.setText('Current point: {}/{} ({:.0%})'.format(
            self._daq.current_point_count, self._daq.total_points,
            self._daq.current_point_count / self._daq._total_points))

        if self._daq.current_point_count == self._daq.total_points and self._window.ui.chkSaveImage.isChecked():
            self.save_scan_image()

    def save_scan_image(self):
        both = self._window.ui.rbBothScan.isChecked()

        if self._window.ui.rbVScan.isChecked() or both:
            img = self._window.make_histogram(self._daq.vdata, 500, 500)
            _file = self._scanfile[:-4]
            if both:
                _file += '_v'
            self._window.px.save(_file + '.png', 'png')

        if self._window.ui.rbHScan.isChecked() or both:
            img = self._window.make_histogram(self._daq.hdata, 500, 500)
            _file = self._scanfile[:-4]
            if both:
                _file += '_h'
            self._window.px.save(_file + '.png', 'png')

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
        self._window.ui.btnStartScan.setEnabled(True)
        self._window.ui.btnStopScan.setEnabled(False)

        self._window.tabCalib.setEnabled(True)
        self._window.on_calib_rb_changed()
        self._window.enable_scan_controls(True)

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
