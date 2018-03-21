#!/usr/bin/env python2

# Dummy server for testing communication while not connected to all the real
# devices at PSFC

import socket
import time
import threading
import sys
import Queue
from inspect import isclass
# from collections import OrderedDict

import numpy as np

debug = False

TCP_IP = '0.0.0.0'
TCP_PORT = 5000
BUFFER_SIZE = 1024  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# allow for reconnections if the server crashes without unbinding
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind((TCP_IP, TCP_PORT))
s.listen(1)

# device dict just holds device name & current values for dummy server
devices = {'pico': 0.0, 'vstepper': 0.0, 'hstepper': 0.0, 'vreg': 0.0}

#hstepper_thread = threading.Thread(target=)
#vreg_thread = threading.Thread(target=)

vstepper_queue = Queue.Queue()
hstepper_queue = Queue.Queue()

# flag to kill threads
shutdown = False

class Mover():
    def __init__(self, device, speed):
        self._dt = 0.01
        self._device = device
        self._terminate = False
        self._target = devices[device]
        self._speed = speed

    def run(self):
        while not self._terminate:
            _dir = 1 if devices[self._device] < self._target else -1
            current_target = self._target
            while abs(devices[self._device] - self._target) > self._speed * self._dt:
                devices[self._device] += self._speed * self._dt * _dir
                time.sleep(self._dt)
                if self._target != current_target:
                    break
            else:
                # i.e. the loop terminated naturally
                devices[self._device] = self._target
                time.sleep(self._dt)

    def set_target(self, value):
        self._target = value

    def terminate(self):
        self._terminate = True

def run_vstepper():
    vmover = Mover('vstepper', 6400)
    move_thread = threading.Thread(target=vmover.run)
    move_thread.start()
    while not shutdown:
        if not vstepper_queue.empty():
            cmd = vstepper_queue.get_nowait()
            if cmd[:2] == 'MA':
                target = min(max(float(cmd.split(' ')[1]), -50000), 50000)
                vmover.set_target(target)
            elif cmd == 'SL - SP':
                target = -50000
                vmover.set_target(target)
            elif cmd == 'SL SP':
                target = 50000
                vmover.set_target(target)
            elif cmd == '\x1b':
                vmover.set_target(devices['vstepper'])
        else:
            pass

def run_hstepper():
    hmover = Mover('hstepper', 6400)
    move_thread = threading.Thread(target=hmover.run)
    move_thread.start()
    while not shutdown:
        if not hstepper_queue.empty():
            cmd = hstepper_queue.get_nowait()
            if cmd[:2] == 'MA':
                target = min(max(float(cmd.split(' ')[1]), -50000), 50000)
                hmover.set_target(target)
            elif cmd == 'SL - SP':
                target = -50000
                hmover.set_target(target)
            elif cmd == 'SL SP':
                target = 50000
                hmover.set_target(target)
            elif cmd == '\x1b':
                hmover.set_target(devices['hstepper'])
        else:
            pass

# need some fake processes to be able to simulate delayed responses from devices
vstepper_thread = threading.Thread(target=run_vstepper)
hstepper_thread = threading.Thread(target=run_hstepper)

vstepper_thread.start()
hstepper_thread.start()

def poll():
    '''
    Aggregate available info from all devices
    Return 'ERR' string if a device is uninitialized
    '''
    # order devices manually
    device_names = ['pico', 'vstepper', 'hstepper', 'vreg']

    # what I would like to do:
    # device_names = [device_name for device_name, _ in devices.iteritems()]
    # maybe this can be solved with an ordered dict
    devices['pico'] = np.random.normal(1, 1)
    values = [devices[device_name] for device_name in device_names]

    for i in [1,2]:
        if values[i] == 50000:
            values[i] = '50000,MAX'
        elif values[i] == -50000:
            values[i] = '-50000,MIN'

    return ' '.join([str(val) for val in values])

def vset(arg):
    # will be sent command 'vset ####'
    try:
        devices['vreg'] = min(max(float(arg.split(' ')[1]), -10), 10)
    except:
        pass

def hmove(arg):
    hstepper_queue.put(arg)

def vmove(arg):
    vstepper_queue.put(arg)

def set_devices(v=None, h=None, vol=None):
    ''' gui can send one command to move a stepper and set voltage '''
    if v is not None and h is not None:
        print('Attempted to move vertical and horizontal steppers at the same time!')
        return

    if vol is not None:
        vset('vset {}'.format(vol))

    if v is not None:
        vmove('MA {}'.format(v))

    if h is not None:
        hmove('MA {}'.format(h))

# mapping of received words to function calls
fmap = {
        b'poll': poll,
        b'vset': vset,
        b'hmove': hmove,
        b'vmove': vmove
       }

print("Scanner DAQ Server accepting connections")

try:
    # outer loop to continuously accept connections
    while True:
        conn, addr = s.accept()
        print("got connection at {}".format(addr))
        # inner loop to handle messages with current connection
        while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                break
            print("received data: {}".format(data))

            # call function corresponding to what was sent
            # if sent data has an argument, split it from the command word
            tp = data.split(' ', 1) # max split = 1
            if tp == ['poll']:
                # on poll request we immediately send the result
                conn.send(fmap[tp[0]]())
            elif tp[0] != 'setall':
                # single arg commands
                word, arg = tp
                try:
                    fmap[word](arg)
                except KeyError:
                    print('Did not understand message: {}'.format(word))
                    pass
            else:
                word, args = tp
                # should be exactly 3 words (2 splits) in the arg list
                # convert the string None to python None
                args = [None if arg == 'None' else arg \
                        for arg in args.split(' ', 2)]
                set_devices(*args)

        conn.close()
except KeyboardInterrupt:
    shutdown = True
