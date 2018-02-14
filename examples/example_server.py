#!/usr/bin/env python

import socket
import time

TCP_IP = '0.0.0.0'
TCP_PORT = 5000
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

while True:
    conn, addr = s.accept()
    while True:
        data = conn.recv(BUFFER_SIZE)
        if not data:
            break
        print "received data:", data
        conn.send(data)  # echo
    conn.close()
