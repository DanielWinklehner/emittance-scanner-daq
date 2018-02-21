#!/usr/bin/env python2

import socket
import time
import threading
import sys
import subprocess
from inspect import isclass
# from collections import OrderedDict

from devices.pico import Pico
from devices.stepper import Stepper
from devices.vreg import Vreg

debug = False

TCP_IP = '0.0.0.0'
TCP_PORT = 5000
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# allow for reconnections if the server crashes without unbinding
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind((TCP_IP, TCP_PORT))
s.listen(1)

# device dict, default assignment to the respective class
devices = {
            'pico': {'device': Pico, 'serial': 'Controller', 'thread': None, 'port': 'COM6'},
            'vstepper': {'device': Stepper, 'serial': '8212017125346', 'thread': None, 'port': 'COM5'},
            'hstepper': {'device': Stepper, 'serial': 'aaa', 'thread': None, 'port': ''},
            'vreg': {'device': Vreg, 'serial': 'aaa', 'thread': None, 'port': ''}
}

'''
# poll usb ports to associate devices
proc = subprocess.Popen('/home/mist-1/Work/server/usb.sh', stdout=subprocess.PIPE, shell=True)
output = proc.stdout.read().strip()

# Loop through all found devices
for line in output.split("\n"):
    port, raw_info = line.split(" - ")
    serial_number = raw_info.split("_")[-1]

    # if a serial number matches, instantiate a new object
    for device_name, info in devices.items():
        if info['serial'] == serial_number:
            info['device'] = info['device'](port, debug=debug)
            break
'''
# TODO windows method hard-codes the ports... for now
for device_name, info in devices.items():
    if info['port'] != '':
        info['device'] = info['device'](info['port'], debug=debug)

    if device_name == 'vreg':
        info['device'] = info['device'](debug=debug)

# check that all devices were found & _initialized
for device_name, info in devices.iteritems():
    if isclass(info['device']): # i.e. uninitialized.
        print('Error: Could not initialize {}. '
              'Serial number {} not found.'.format(
              device_name, info['serial']))
        #sys.exit(0)


# set up devices
for device_name, info in devices.iteritems():
    if not isclass(info['device']):
        info['thread'] = threading.Thread(target=info['device'].run)
        info['thread'].start()

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

    values = ['ERR' if isclass(devices[device_name]['device']) \
                else devices[device_name]['device'].current_value \
                for device_name in device_names
            ]

    return ' '.join([str(val) for val in values])

def vset(arg):
    if isclass(devices['vreg']['device']):
        print('Attempted to send command to offline device! (vreg)')
        return
    devices['vreg']['device'].add_command_to_queue(arg)

def hmove(arg):
    if isclass(devices['hstepper']['device']):
        print('Attempted to send command to offline device! (hstepper)')
        return
    devices['hstepper']['device'].add_command_to_queue(arg)

def vmove(arg):
    if isclass(devices['vstepper']['device']):
        print('Attempted to send command to offline device! (vstepper)')
        return
    devices['vstepper']['device'].add_command_to_queue(arg)

def move(v=None, h=None, vol=None):
    ''' gui can send one command to move a stepper and set voltage '''
    if v is not None and h is not None:
        print('Attempted to move vertical and horizontal steppers at the same time!')
        return

    if vol is not None:
        vset(vol)

    if v is not None:
        vmove(v)

    if h is not None:
        hmove(h)

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
            else:
                word, arg = tp
                fmap[word](arg)

        conn.close()
except KeyboardInterrupt:
    for device_name, info in devices.iteritems():
        if not isclass(info['device']):
            info['device'].terminate()
