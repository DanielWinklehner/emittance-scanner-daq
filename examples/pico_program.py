import serial
import time

import numpy as np

def fast_read(ser):
    # Our own 'readline()' function
    response = b''

    start_time = time.time()
    while (time.time() - start_time) < 4.0:
        resp = ser.read(1)
        if resp:
            response += bytes(resp)
            if resp in [b'\n', b'\r']:
                break
            elif resp == b';':
                # Handle MFC Readout (read in two more bytes for checksum and break)
                response += bytes(ser.read(2))
                break
    else: # thanks, python
        response += b''

    return response

initialize = [
        '*RST', 'TRIG:DEL 0', 'TRIG:COUN 10', 'NPLC .01', 'RANG .002',
        'SYST:ZCH OFF', 'SYST:AZER:STAT OFF', 'DISP:ENAB OFF', '*CLS',
        'TRAC:POIN 10', 'TRAC:CLE'
        ]
        
program = [
        'TRAC:FEED:CONT NEXT', 'STAT:MEAS:ENAB 512', '*SRE 1', #'*OPC?', 'r', 
        'INIT', 'DISP:ENAB ON', 'TRAC:DATA?', 'r'
        ]

s = serial.Serial('/dev/ttyUSB0', 9600)

s.reset_input_buffer()
s.reset_output_buffer()

mydata = []

for cmd in initialize:
    #print cmd
    s.write(cmd + '\r\n')

while True:
    for cmd in program:
        #print cmd
        if cmd != 'r':
            s.write(cmd + '\r\n')
        else:
            resp = fast_read(s).split(',')
            data = [float(msg[:-1]) for msg in resp if msg[-1] == 'A']
            if len(data) > 0:
                print np.mean(data)
                mydata.append(np.mean(data))

    with open('test.txt', 'a+') as f:
        f.write(str(np.mean(data)) + '\n')
