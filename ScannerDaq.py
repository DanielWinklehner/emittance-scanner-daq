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
from collections import deque

import numpy as np

from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QFileDialog

from gui import MainWindow

class CouldNotConnectError(Exception):
    pass

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

            net_sleep = sleep_time - (timeit.default_timer() - loop_start)
            if net_sleep > 0:
                time.sleep(net_sleep)

    def terminate(self):
        self._terminate = True

class DaqView():

    #sig_connected = pyqtSignal()
    #sig_disconnected = pyqtSignal()

    def __init__(self):
        self._window = MainWindow.MainWindow()
        self._window.btnConnect.clicked.connect(self.connect_to_server)

        # stepper manual controls
        self._window.ui.btnRetract.clicked.connect(lambda: self.stepper_com('SL SP'))
        self._window.ui.btnExtend.clicked.connect(lambda: self.stepper_com('SL - SP'))
        self._window.ui.btnStop.clicked.connect(lambda: self.stepper_com('\x1b'))
        self._window.ui.btnCalibMoveSet.clicked.connect(self.calib_move_set)

        # calibration buttons
        self._window.ui.btnVCalibSet.clicked.connect(self.set_vstepper_calibration)
        self._window.ui.btnHCalibSet.clicked.connect(self.set_hstepper_calibration)
        self._window.ui.btnVolCalibSet.clicked.connect(self.set_vreg_calibration)

        self._window.ui.btnVCalibReset.clicked.connect(self.reset_vstepper_calibration)
        self._window.ui.btnHCalibReset.clicked.connect(self.reset_hstepper_calibration)

        # calibration radio buttons
        self._window.ui.rbVCalibPt1.toggled.connect(self.on_v_calib_check_changed)

        # scan page textboxes all call the same checking function, so we do this
        # with eval instead of writing out 12 separate lines
        txtlist = ['txtVMinPos', 'txtVMaxPos', 'txtVStepPos', 'txtVMinV', 'txtVMaxV', 'txtVStepV',
                   'txtHMinPos', 'txtHMaxPos', 'txtHStepPos', 'txtHMinV', 'txtHMaxV', 'txtHStepV']

        for txt in txtlist:
            eval('self._window.ui.{}.textChanged.connect(self.on_scan_textbox_change)'.format(txt))

        # set up main device dictionary
        device_name_list = ['pico', 'vstepper', 'hstepper', 'vreg']
        self._devices = dict()

        for name in device_name_list:
            self._devices[name] = {'value': 0.0, # real value returned from the server
                                   'deque': deque(maxlen=10),
                                   'hasErr': False,
                                   'label': None,
                                   'name': '',
                                   'unit': '',
                                   'status': '',
                                   'fmt': '{0:.2f}',
                                   'calibration': [(None, None), (None, None)] # Linear interpolate between these points if set
                                  }

        # manually assign labels & properties to each device
        self._devices['pico']['label'] = self._window.lblCur
        self._devices['vstepper']['label'] = self._window.lblVer
        self._devices['hstepper']['label'] = self._window.lblHor
        self._devices['vreg']['label'] = self._window.lblV

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

        # calibration variables
        self._vercalib = False
        self._horcalib = False

        self._com_thread = QThread()

        self.update_display_values()

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
        self._comm.sig_data.connect(self.on_data)
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
        self._comm.terminate()
        self._com_thread.quit()

        self._window.statusBar.showMessage(
                'Server connection at {}:{} closed.'.format(
                self._comm.server_ip, self._comm.port))

        # reset gui elements
        self._window.btnConnect.setText('Connect')
        self._window.tabCalib.setEnabled(False)
        self._window.tabScan.setEnabled(False)

        # reset devices
        self._ver_calib_pts = (None, None)
        self._hor_calib_pts = (None, None)
        self.check_calibration()

    def on_poll_rate(self, rate):
        self._window.lblPollRate.setText('Polling rate: {0:.2f} Hz'.format(rate))

    def calibrate(self, val, dev):
        ''' Converts a value to a device's units based on its calibration '''
        # python trickery to flatten a list of tuples
        if None in list(sum(dev['calibration'], ())):
            # if calibration not set, don't do anuthing
            return val

        pt1 = dev['calibration'][0]
        pt2 = dev['calibration'][1]

        # find slope
        m = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])

        # return linear extrapolation
        return m * (val - pt1[0]) + pt1[1]

    def on_data(self, data):
        ''' Updates devices when a message is received from the server '''
        data = data.decode("utf-8")
        cur, ver, hor, vol = [float(x) if x != 'ERR' \
                                else 'ERR' for x in data.split(' ')]

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

        self.update_display_values()

    def update_display_values(self):
        ''' Updates the GUI with the latest device values '''
        for device_name, info in self._devices.items():
            if info['hasErr']:
                info['label'].setText('{0}: Error!'.format(info['name']))
            else:
                # Device name: <value> <unit> <optional message>
                info['label'].setText('{0}: {1} {2} {3}'.format(
                        info['name'], info['fmt'].format(
                            self.calibrate(info['value'],
                                self._devices[device_name])),
                        info['unit'], '(%s)' % (info['status']) if \
                            info['status'] != '' else ''
                    )
                )

    # stepper buttons on calibration page call this function with fixed commands
    def stepper_com(self, cmd):
        if self._window.ui.rbVMove.isChecked():
            msg = 'vmove '
        else:
            msg = 'hmove '

        msg += cmd

        self._comm._command_queue.put(msg)

    def calib_move_set(self):
        ''' Called when the user presses Go button on calibration page '''

        stepdest = None
        vdest = None

        # Did the user provide a number of steps?
        steptxt = self._window.ui.txtMovePos.text()
        if steptxt.strip() != '':
            try:
                stepdest = int(steptxt)
            except ValueError:
                pass

        # Did the user provide a voltage target?
        vtxt = self._window.ui.txtMoveV.text()
        if vtxt.strip() != '':
            try:
                stepdest = float(steptxt)
            except ValueError:
                pass

        if stepdest is not None:
            # send the absolute move command to the selected stepper
            self.stepper_com('MA {}'.format(stepdest))

        if vdest is not None:
            # send the voltage set command to the voltage regulator
            self._comm._command_queue.put('vset {}'.format(vdest))

    def check_calibration(self):
        ''' Determines if certain devices are calibrated or not '''
        # are both vertical points set?
        # python trickery to flatten a list of tuples
        if None not in list(sum(self._devices['vstepper']['calibration'], ())):
            self._devices['vstepper']['status'] = ''
            self._devices['vstepper']['unit'] = 'mm'
            self._vercalib = True
        else:
            self._devices['vstepper']['status'] = 'Not calibrated'
            self._devices['vstepper']['unit'] = 'steps'
            self._vercalib = False

        # same for horizontal points
        if None not in list(sum(self._devices['hstepper']['calibration'], ())):
            self._devices['hstepper']['status'] = ''
            self._devices['hstepper']['unit'] = 'mm'
            self._horcalib = True
        else:
            self._devices['hstepper']['status'] = 'Not calibrated'
            self._devices['hstepper']['unit'] = 'steps'
            self._horcalib = False

        if self._vercalib or self._horcalib:
            self._window.tabScan.setEnabled(True)

            if not self._vercalib:
                self._window.ui.rbVScan.setEnabled(False)
                self._window.ui.rbHScan.setChecked(True)

            if not self._horcalib:
                self._window.ui.rbHScan.setEnabled(False)
                self._window.ui.rbVScan.setChecked(True)

    def set_vstepper_calibration(self):
        # see if the user entered an Ok value
        try:
            val = float(self._window.ui.txtVCalib.text().strip())
        except ValueError:
            return

        #is the user setting pt 1 or pt 2?
        idx = 0
        if self._window.ui.rbVCalibPt2.isChecked():
            idx = 1
            self._window.ui.rbVCalibPt1.setChecked(True)
        else:
            self._window.ui.rbVCalibPt2.setChecked(True)

        self._devices['vstepper']['calibration'][idx] = \
                (self._devices['vstepper']['value'], val)

        # replace the label text depending on which point is selected
        msg = self._window.ui.lblVCalib.text().split('\n')
        msg[idx] = '{} mm = {} steps'.format(int(val),
                int(self._devices['vstepper']['value']))
        self._window.ui.lblVCalib.setText('\n'.join(msg))

        self.check_calibration()

    def reset_vstepper_calibration(self):
        self._devices['vstepper']['calibration'] = [(None, None), (None, None)]
        self._window.ui.lblVCalib.setText('1. Not set\n2. Not set')
        self.check_calibration()

    def set_hstepper_calibration(self):
        pass

    def reset_hstepper_calibration(self):
        self._devices['hstepper']['calibration'] = [(None, None), (None, None)]
        self._window.ui.lblHCalib.setText('1. Not set\n2. Not set')
        self.check_calibration()

    def set_vreg_calibration(self):
        pass

    def on_v_calib_check_changed(self):
        self._window.ui.txtVCalib.setText('')
        if self._window.ui.rbVCalibPt1.isChecked():
            pass
        else:
            pass

    # Scan page functions
    def on_scan_textbox_change(self):
        ''' Function that calculates the number of scan points '''
        self._window.ui.lblScanPoints.setText('Total points: --')
        self._window.ui.btnStartStopScan.setEnabled(False)

        v_points = 0
        h_points = 0

        # calculate number of vectical points
        if self._window.ui.rbVScan.isChecked() or self._window.ui.rbBothScan.isChecked():
            # try to parse user input for all vertical textboxes
            try:
                vmin = float(txtVMinPos.text())
                vmax = float(txtVMaxPos.text())
                vstep = float(txtVStepPos.text())
                vminv = float(txtVMinV.text())
                vmaxv = float(txtVMaxV.text())
                vstepv = float(txtVStepV.text())
            except ValueError:
                return

            # user input should be sequential
            if (vmin > vmax) or (vminv > vmaxv):
                return

            # otherwise we can calculate the number of vertical points
            v_points = len(np.arange(vmin, vmax, vstep)) * \
                        len(np.arange(vminv, vmaxv, vstepv))

        # calculate number of horizontal points
        if self._window.ui.rbHScan.isChecked() or self._window.ui.rbBothScan.isChecked():
            try:
                hmin = float(txtHMinPos.text())
                hmax = float(txtHMaxPos.text())
                hstep = float(txtHStepPos.text())
                hminv = float(txtHMinV.text())
                hmaxv = float(txtHMaxV.text())
                hstepv = float(txtHStepV.text())
            except ValueError:
                return

            if (hmin > hmax) or (hminv > hmaxv):
                return

            # otherwise we can calculate the number of vertical points
            h_points = len(np.arange(hmin, hmax, hstep)) * \
                        len(np.arange(hminv, hmaxv, hstepv))

        # if we made it here, then we can update the text box
        self._window.ui.lblScanPoints.setText(
                'Total points: {}'.format(h_points + v_points))
        self._window.ui.btnStartStopScan.setEnabled(True)

    def scan(self):
        pass
        # calculate points to scan
        # queue commands to be sent
        # execute command
        # wait for anticipated response
        # record current for some time step & average
        # move to next command

    def run(self):
        self._window.show()

if __name__ == '__main__':
    app = QApplication([])

    dq = DaqView()
    dq.run()
    sys.exit(app.exec_())
