#!/usr/bin/env python

import socket
import time

TCP_IP = '10.77.0.3'
TCP_PORT = 5000
BUFFER_SIZE = 1024
MESSAGE = b"Hello, World!"
END = b"END"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(MESSAGE)

time.sleep(0.5)

s.send(MESSAGE)
data = s.recv(BUFFER_SIZE)
s.close()

print("received data:", data)
