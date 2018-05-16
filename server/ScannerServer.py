#!/usr/bin/env python2

import socket
import time
import threading
import sys
from inspect import isclass
from serial.tools import list_ports

from devices.pico import Pico
from devices.stepper import Stepper
from devices.vreg import Vreg

debug = False

TCP_IP = '0.0.0.0'
TCP_PORT = 5000
BUFFER_SIZE = 1024  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# allow for reconnections if the server crashes without unbinding
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind((TCP_IP, TCP_PORT))
s.listen(1)

serial_mapping = {
    'a': 'pico',
    '8212017125346': 'vstepper',
    'b': 'hstepper',
    'c': 'vreg',
}

# device dict, default assignment to the respective class
devices = {
            'pico': {'device': Pico, 'serial': 'Controller', 'thread': None, 'port': ''},
            'vstepper': {'device': Stepper, 'serial': '8212017125346', 'thread': None, 'port': ''},
            'hstepper': {'device': Stepper, 'serial': 'aaa', 'thread': None, 'port': ''},
            'vreg': {'device': Vreg, 'serial': 'aaa', 'thread': None, 'port': ''}
}

# fill in port field in the main device dictionary by matching serial numbers detected in Windows
for device_info in list_ports.comports():
    try:
        devices[serial_mapping[device_info.serial_number]]['port'] = device_info.device
    except KeyError:
        print('Found serial number {} but it is not associated with any scanner-associated devices.'.format(
            device_info.serial_number))

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

    # for stepper motors, there is a special flag to send if at minimum or maximum positions
    # indicated by error codes from the stepper motor
    if not isclass(devices['vstepper']['device']):
        if devices['vstepper']['device'].error_code == 83:
            values[1] = '{},MAX'.format(values[1])
        elif devices['vstepper']['device'].error_code == 84:
            values[1] = '{},MIN'.format(values[1])

    if not isclass(devices['hstepper']['device']):
        if devices['hstepper']['device'].error_code == 83:
            values[2] = '{},MAX'.format(values[2])
        elif devices['hstepper']['device'].error_code == 84:
            values[2] = '{},MIN'.format(values[2])

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
            # try to catch communication error
            # too fast communication can result in a poll message being appended
            # to a set command. Check if this is the case
            if len(data) > 4 and data[-4:] == 'poll':
                print('Communication too fast! Data is: {}'.format(data))
                continue

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
    for device_name, info in devices.iteritems():
        if not isclass(info['device']):
            info['device'].terminate()
