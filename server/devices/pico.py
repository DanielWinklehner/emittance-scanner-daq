import serial
import time

from devices.serial_com import fast_read

class Pico():
    def __init__(self, port, debug=False):

        self._terminate = False
        self._debug = debug

        # startup commands
        self._init_prgm = [
                '*RST',           # reset to default settings
                'FORM:ELEM READ', # output only the value (i.e. no unit/timestamp, etc)
                'NPLC .01',       # fastest polling rate
                'SYST:ZCH OFF'    # disable zero checking
        ]

        # loop commands
        self._run_prgm = [
        'READ?', 'r'
        ]

        self._s = serial.Serial(port, 57600)

        self._s.reset_input_buffer()
        self._s.reset_output_buffer()

        # dummy initial value
        self._current_value = -1.

    @property
    def current_value(self):
        return self._current_value

    def run(self):

        for cmd in self._init_prgm:
            if self._debug:
                print(cmd)
            self._s.write(cmd + '\r')

        while not self._terminate:
            for cmd in self._run_prgm:
                if self._debug:
                    print(cmd)
                if cmd != 'r':
                    self._s.write(cmd + '\r')
                else:
                    resp = fast_read(self._s)#.split(',')
                    if self._debug:
                        print(resp)
                    self._current_value = float(resp)

    def terminate(self):
        self._terminate = True
