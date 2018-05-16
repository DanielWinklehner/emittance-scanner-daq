import time
import Queue
import nidaqmx

from devices.serial_com import fast_read


class Vreg:
    def __init__(self, debug=False):

        self._terminate = False
        self._debug = debug

        self._current_value = -1.

        # server will put commands to be issued in this queue object
        self._command_queue = Queue.Queue()
        self._write_task = nidaqmx.Task()
        self._read_task = nidaqmx.Task()

        self._write_task.ao_channels.add_ao_voltage_chan("Dev1/ao0")
        self._read_task.ai_channels.add_ai_voltage_chan("Dev1/ai0")

    @property
    def current_value(self):
        return self._current_value

    def set_volt(self, msg):
        chars = msg.split()
        if chars[0] == 'vset':
            volt = float(chars[1])
            # make sure we don't set the voltage too high or low
            volt = max(min(10.0, volt), -10.0)
            if self._debug:
                print(volt)
            self._write_task.write(volt, auto_start = True)

    def run(self):
        while not self._terminate:
            # check if we have outstanding commands in the queue

            if not self._command_queue.empty():
                cmd = self._command_queue.get_nowait()
                if self._debug:
                    print(cmd)
                self.set_volt(cmd)
                # call this because stepper echoes each command
                continue

            self._current_value = self._read_task.read()

    def add_command_to_queue(self, cmd):
        self._command_queue.put(cmd)

    def terminate(self):
        self._terminate = True
        self._task.close()
