#!/usr/bin/python

import socket
from struct import *

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(14) # buffer size
    unpacked =  unpack('>fffB', data)
    print "received message: ", unpacked