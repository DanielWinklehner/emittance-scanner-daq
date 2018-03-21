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
from collections import deque

import numpy as np
import datetime as dt

from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QFileDialog, QDialog

from gui import MainWindow

# need this to be a global function
def calibrate(val, dev, reverse=False):
    ''' Converts a value to a device's units based on its calibration '''
    # python trickery to flatten a list of tuples
    if None in list(sum(dev['calibration'], ())):
        # if calibration not set, clean up argument & return
        if isinstance(val, np.ndarray):
            # we passed in a numpy array, so use numpy syntax
            return np.around(val, decimals=4).astype(dev['type'])
        else:
            # we passed in a single value, so use default python syntax
            return dev['type'](val)

    pt1 = dev['calibration'][0]
    pt2 = dev['calibration'][1]

    # find slope
    m = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])

    # return linear extrapolation
    if reverse:
        # if true, assume the user passed a calibrated value and wants the
        # device value back, make sure it is the right data type
        if isinstance(val, np.ndarray):
            return np.around(pt1[0] + (val - pt1[1]) / m, decimals=4).astype(dev['type'])
        else:
            return dev['type'](pt1[0] + (val - pt1[1]) / m)
    else:
        # return the calibrated value, has to be a float
        return m * (val - pt1[0]) + pt1[1]


class CouldNotConnectError(Exception):
    pass


class Daq(QObject):
    ''' Class for collecting and organizing data from scans '''

    sig_msg = pyqtSignal(str) # emit when ready to send command to server
    sig_new_pt = pyqtSignal(object) # emit when the scan completes a new point
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
                    self._vdata[i * vsteps + j]['v'] = calibrate(vtargetv, self._devices['vreg'])

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
                    self._hdata[i * hsteps + j]['v'] = calibrate(htargetv, self._devices['vreg'])

            # initialize currents to nan so that the gui can draw null bins on 2d histogram
            self._hdata['i'] = np.nan

        self._terminate = False

    def close_enough(self, val1, val2):
        ''' Really dumb function for comparing floats '''
        epsilon = 1e-6
        return abs(val1 - val2) < epsilon

    def safe_move(self, vtarget, htarget, voltarget):
        ''' Sends set command and waits until devices have reached set point '''

        stepper = 'vstepper' if  vtarget is not None else 'hstepper'
        target = vtarget if vtarget is not None else htarget

        prev_val = self._devices[stepper]['value']
        # stepper motor is traditionally slower so try that first
        while not self.close_enough(self._devices[stepper]['value'], target):
            # server expects setall <v stepper value> <h stepper value> <vreg value>s
            # only want to send command if we are stuck

            # for exiting mid-scan
            if self._terminate:
                return

            if self._devices[stepper]['value'] == prev_val:
                msg = 'setall {} {} {}'.format(vtarget, htarget, voltarget)
                self.sig_msg.emit(msg)
            time.sleep(0.1)

        # now send some more commands if we are not at voltage target
        if voltarget is None:
            return

        prev_val = self._devices['vreg']['value']
        while not self.close_enough(self._devices['vreg']['value'], voltarget):

            # for exiting mid-scan
            if self._terminate:
                return

            if self._devices['vreg']['value'] == prev_val:
                msg = 'setall {} {} {}'.format(None, None, voltarget)
                self.sig_msg.emit(msg)
            time.sleep(0.1)

    def scan(self, stepper, stepper_pts, vreg_pts):

        # make sure the other stepper is parked
        other = 'hstepper' if stepper == 'vstepper' else 'vstepper'
        _target = self._devices[other]['calibration'][0][0]
        if _target is not None:
            # if it isn't calibrated, then there's nothing we can do
            self.safe_move(_target, None, None) if other == 'vstepper' else \
                self.safe_move(None, _target, None)

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
                self.sig_new_pt.emit(pt)

        if not self._terminate:
            # move stepper to parked position -- calibration point 1 is tuple with (steps, mm), use steps
            target = self._devices[stepper]['calibration'][0][0]
            self.safe_move(target, None, None) if stepper == 'vstepper' else \
                self.safe_move(None, target, None)

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
                    time.sleep(0.01)
                    continue

                # otherwise just send a poll request
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
    ''' Class containing devices to be updated asynchronously from the gui '''
    def __init__(self):
        super().__init__()

        device_name_list = ['pico', 'vstepper', 'hstepper', 'vreg']
        self._devices = dict()

        for name in device_name_list:
            self._devices[name] = {'value': 0.0, # real value returned from the server
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

        self._devices['pico']['name'] = 'Current'
        self._devices['vstepper']['name'] = 'Vertical'
        self._devices['hstepper']['name'] = 'Horizontal'
        self._devices['vreg']['name'] = 'Voltage'

        self._devices['pico']['unit'] = 'A'
        self._devices['vstepper']['unit'] = 'steps'
        self._devices['hstepper']['unit'] = 'steps'
        self._devices['vreg']['unit'] = 'kV'

        # exceptions to default values
        self._devices['pico']['fmt'] = '{0:.4e}'
        self._devices['vstepper']['status'] = 'Not calibrated'
        self._devices['hstepper']['status'] = 'Not calibrated'
        self._devices['vstepper']['type'] = int
        self._devices['hstepper']['type'] = int

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
            # vdata is a list
            print(vdata)

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

class DaqView():
    ''' Handles interaction between GUI & server '''
    def __init__(self):

        self._window = MainWindow.MainWindow()

        self._window.btnConnect.clicked.connect(self.connect_to_server)
        self._window.ui.btnExit.triggered.connect(self.exit)

        # stepper manual controls
        self._window.ui.btnRetract.clicked.connect(lambda: self.stepper_com('SL SP'))
        self._window.ui.btnExtend.clicked.connect(lambda: self.stepper_com('SL - SP'))
        self._window.ui.btnStop.clicked.connect(lambda: self.stepper_com('\x1b')) # esc key

        # calibration buttons
        self._window.ui.btnStartVCalib.clicked.connect(self.start_vstepper_calibration)
        self._window.ui.btnStartHCalib.clicked.connect(self.start_hstepper_calibration)

        # scan buttons
        self._window.ui.btnChooseFile.clicked.connect(self.choose_file)
        self._window.ui.btnStartStopScan.clicked.connect(self.scan)

        # testing buttons
        self._window.ui.btnVregTest.clicked.connect(self.test_vreg)

        # scan page textboxes all call the same checking function, so we do this
        # with eval instead of writing out 12 separate lines
        txtlist = ['txtVMinPos', 'txtVMaxPos', 'txtVStepPos', 'txtVMinV', 'txtVMaxV', 'txtVStepV',
                   'txtHMinPos', 'txtHMaxPos', 'txtHStepPos', 'txtHMinV', 'txtHMaxV', 'txtHStepV']

        for txt in txtlist:
            eval('self._window.ui.{}.textChanged.connect(self.on_scan_textbox_change)'.format(txt))

        self._window.ui.rbVScanStatus.toggled.connect(self.on_scan_status_rb_changed)

        # calibration variables
        self._vercalib = False
        self._horcalib = False

        # device manager class allows devices to be updated independently of the gui thread
        self._dm = DeviceManager()

        # manually assign labels & properties to each device
        self._dm.devices['pico']['label'] = self._window.lblCur
        self._dm.devices['vstepper']['label'] = self._window.lblVer
        self._dm.devices['hstepper']['label'] = self._window.lblHor
        self._dm.devices['vreg']['label'] = self._window.lblV

        #self._dm.devices['vstepper']['calibration'] = [(50000, 20), (-50000, -20)]
        self.check_calibration()

        self._com_thread = QThread()
        self._dm_thread = QThread()

        self._dm.moveToThread(self._dm_thread)
        self._dm_thread.start()

        # default objects for saving scans
        self._scanfile = ''

        self.update_display_values()

    def test_vreg(self):
        val = np.random.normal(loc=0, scale=10)
        self._comm.add_message_to_queue('vset vset {0:.2f}'.format(val))
        print(val)

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
        self.check_calibration()

    def on_poll_rate(self, rate):
        self._window.lblPollRate.setText('Polling rate: {0:.2f} Hz'.format(rate))

    def update_display_values(self):
        ''' Updates the GUI with the latest device values '''
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
        ''' Determines if certain devices are calibrated or not '''
        # are both vertical points set?
        # python trickery to flatten a list of tuples
        if None not in list(sum(self._dm.devices['vstepper']['calibration'], ())):
            self._dm.devices['vstepper']['status'] = ''
            self._dm.devices['vstepper']['unit'] = 'mm'
            self._vercalib = True
        else:
            self._dm.devices['vstepper']['status'] = 'Not calibrated'
            self._dm.devices['vstepper']['unit'] = 'steps'
            self._vercalib = False

        # same for horizontal points
        if None not in list(sum(self._dm.devices['hstepper']['calibration'], ())):
            self._dm.devices['hstepper']['status'] = ''
            self._dm.devices['hstepper']['unit'] = 'mm'
            self._horcalib = True
        else:
            self._dm.devices['hstepper']['status'] = 'Not calibrated'
            self._dm.devices['hstepper']['unit'] = 'steps'
            self._horcalib = False

        if self._vercalib or self._horcalib:
            self._window.tabScan.setEnabled(True)

            if not self._vercalib:
                self._window.ui.rbVScan.setEnabled(False)
                self._window.ui.rbHScan.setChecked(True)
                self._window.ui.rbBothScan.setEnabled(False)
                return

            if not self._horcalib:
                self._window.ui.rbHScan.setEnabled(False)
                self._window.ui.rbVScan.setChecked(True)
                self._window.ui.rbBothScan.setEnabled(False)
                return

            # if both calibrated, we can do a 2-axis scan
            self._window.ui.rbVScan.setEnabled(True)
            self._window.ui.rbHScan.setEnabled(True)
            self._window.ui.rbBothScan.setEnabled(True)

    ############################
    # Calibration page functions
    ############################

    def start_vstepper_calibration(self):
        try:
            upper = float(self._window.ui.txtVCalibUpper.text().strip())
            lower = float(self._window.ui.txtVCalibLower.text().strip())
        except ValueError:
            # TODO: Meaningful error message
            return

        if upper == lower:
            # TODO: Meaningful error message
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
            upper = float(self._window.ui.txtHCalibUpper.text().strip())
            lower = float(self._window.ui.txtHCalibLower.text().strip())
        except ValueError:
            # TODO: Meaningful error message
            return

        if upper == lower:
            # TODO: Meaningful error message
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

        waittime = 1.0 # TODO maybe this should depend on the polling rate?
        prefix = 'vmove ' if stepper == 'vstepper' else 'hmove '

        upper_set = False
        lower_set = False

        lower_steps = None
        upper_steps = None

        # extend stepper <=> move in negative direction
        prev_pos = self._dm.devices[stepper]['value']
        msg = prefix + 'SL - SP'
        self._comm.add_message_to_queue(msg)
        while not lower_set:
            time.sleep(waittime)
            if self._dm.devices[stepper]['value'] != prev_pos:
                prev_pos = self._dm.devices[stepper]['value']
            else:
                lower_steps = self._dm.devices[stepper]['value']
                lower_set = True

        # retract stepper <=> move in positive direction
        prev_pos = self._dm.devices[stepper]['value']
        msg = prefix + 'SL SP'
        self._comm.add_message_to_queue(msg)
        while not upper_set:
            time.sleep(waittime)
            if self._dm.devices[stepper]['value'] != prev_pos:
                prev_pos = self._dm.devices[stepper]['value']
            else:
                upper_steps = self._dm.devices[stepper]['value']
                upper_set = True

        self._dm.devices[stepper]['calibration'] = \
            [(upper_steps, upper), (lower_steps, lower)]

        self.check_calibration()
        self._window.tabCalib.setEnabled(True)
        self._window.tabScan.setEnabled(True)
        self._window.ui.gbSteppers.setEnabled(True)

    def set_vreg_calibration(self):
        # Don't know what to do here yet.
        pass

    #####################
    # Scan page functions
    #####################

    def choose_file(self):
        ''' Called when user presses choose output file button '''
        dlg = QFileDialog(self._window, 'Choose Data File', '' , 'CSV Files (*.csv)')
        if dlg.exec_() == QDialog.Accepted:
            fname = dlg.selectedFiles()[0]
            if fname[-4:] != '.csv':
                fname += '.csv'

            self._scanfile = fname
            self._window.ui.lblSaveFile.setText('Saving output to:\n{}'.format(fname))

            # the start/stop scan button is enabled if the textboxes are valid
            # and the file is set
            self.on_scan_textbox_change()


    def on_scan_textbox_change(self):
        ''' Function that calculates the number of scan points '''
        self._window.ui.lblScanPoints.setText('Total points: --')
        self._window.ui.btnStartStopScan.setEnabled(False)

        v_points = 0
        h_points = 0
        self._dm.devices['vstepper']['scan'] = [None]
        self._dm.devices['hstepper']['scan'] = [None]
        self._dm.devices['vreg']['scan'] = [None, None]

        # calculate number of vectical points
        if self._window.ui.rbVScan.isChecked() or self._window.ui.rbBothScan.isChecked():
            # try to parse user input for all vertical textboxes
            try:
                vmin = float(self._window.ui.txtVMinPos.text())
                vmax = float(self._window.ui.txtVMaxPos.text())
                vstep = float(self._window.ui.txtVStepPos.text())
                vminv = float(self._window.ui.txtVMinV.text())
                vmaxv = float(self._window.ui.txtVMaxV.text())
                vstepv = float(self._window.ui.txtVStepV.text())
            except ValueError:
                # TODO: Meaningful error message
                return

            # user input should be sequential, and having a step of 0 will cause a divide by zero error
            if (vmin > vmax) or (vminv > vmaxv) or vstep == 0 or vstepv == 0:
                # TODO: Meaningful error message
                return

            # otherwise we can calculate the vertical points
            self._dm.devices['vstepper']['scan'] = \
                [calibrate(np.arange(vmin, vmax, vstep), self._dm.devices['vstepper'], reverse=True)]
            self._dm.devices['vreg']['scan'][0] = \
                calibrate(np.arange(vminv, vmaxv, vstepv), self._dm.devices['vreg'], reverse=True)

            v_points = len(self._dm.devices['vstepper']['scan'][0]) * \
                        len(self._dm.devices['vreg']['scan'][0])

        # calculate number of horizontal points
        if self._window.ui.rbHScan.isChecked() or self._window.ui.rbBothScan.isChecked():
            try:
                hmin = float(self._window.ui.txtHMinPos.text())
                hmax = float(self._window.ui.txtHMaxPos.text())
                hstep = float(self._window.ui.txtHStepPos.text())
                hminv = float(self._window.ui.txtHMinV.text())
                hmaxv = float(self._window.ui.txtHMaxV.text())
                hstepv = float(self._window.ui.txtHStepV.text())
            except ValueError:
                # TODO: Meaningful error message
                return

            if (hmin > hmax) or (hminv > hmaxv) or hstep == 0 or hstepv == 0:
                # TODO: Meaningful error message
                return

            # otherwise we can calculate the number of vertical points
            self._dm.devices['hstepper']['scan'] = \
                [calibrate(np.arange(hmin, hmax, hstep), self._dm.devices['hstepper'], reverse=True)]
            self._dm.devices['vreg']['scan'][1] = \
                calibrate(np.arange(hminv, hmaxv, hstepv), self._dm.devices['vreg'], reverse=True)

            h_points = len(self._dm.devices['hstepper']['scan'][0]) * \
                        len(self._dm.devices['vreg']['scan'][1])

        # if we made it here, then we can update the text box
        self._window.ui.lblScanPoints.setText(
                'Total points: {}'.format(h_points + v_points))

        if self._scanfile != '':
            self._window.ui.btnStartStopScan.setEnabled(True)

    def scan(self):
        ''' Called when user presses the start scan button. Calculates the
            points to scan from the textboxes, and creates a Daq object
            which runs in its own thread
        '''

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

    def on_scan_status_rb_changed(self):
        ''' Update the plot when the user switches views '''
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

        with open(self._scanfile, 'w') as f:
            f.write(preamble)

    def on_scan_pt(self, pt):
        ''' update the file and plot label when a new point comes in '''
        if self._window.ui.rbVScanStatus.isChecked():
            self._window.draw_scan_hist(self._daq.vdata)
        else:
            self._window.draw_scan_hist(self._daq.hdata)

        # append data to text file
        with open(self._scanfile, 'a') as f:
            f.write(','.join([str(_) for _ in pt]))
            f.write('\n')

    def on_one_scan_finished(self, final):
        ''' Function called after first scan (vertical) is finished.
            Only relevant during "both" scans
        '''
        if not final:
            self._window.ui.rbHScanStatus.setChecked(True)

    def on_scan_finished(self):
        ''' Clean up after scan, and tell the thread to quit
            This function is called when the stepper has returned to
            its original position.
        '''
        self._daq.deleteLater()
        self._scan_thread.quit()
        self._scan_thread.wait()

    def shutdown_scan(self):
        ''' Safely re-enable the gui once the thread is done. This is called
            when the QThread holding the DAQ object emits its finished signal.
        '''
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
