#!/usr/bin/python
import time
import socket
from struct import *

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

numReceived = 0;
startTime = time.time()
while True:
    data, addr = sock.recvfrom(14) # buffer size
    unpacked =  unpack('>fffB', data)
    print "received message: ", unpacked
    numReceived += 1
    if numReceived >= 100:
      numReceived = 0
      deltaTime = time.time() - startTime
      rate = 100/deltaTime
      #print("Rate: " + str(rate))
      startTime = time.time()