#!/usr/bin/python
#import the simulator and other stuff
from simulator import lidarSimulator
import simulator
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import Laser
from collections import namedtuple
import time
import math
import random

#import the library to plot points
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

#import the libraries to send the data over udp
from struct import *
import socket

#where to send UDP data
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

#define the named tuples
Point = namedtuple('Point', 'x y')
Lineseg = namedtuple('Lineseg', 's e')

#define the size of the map to generate in pixels and meters
MAP_SIZE_PIXELS         = 2048
MAP_SIZE_METERS         = 20

#define the points of the room and maike a loop of lineseg namedtuples from them
room_pts = [Point(0, 0), Point(10, 0), Point(10, 2), Point(12, 2), Point(15,5),Point(15,10),Point(5,10), Point(5,12), Point(0,12)]
room = simulator.loop(room_pts)
#add an obstacle in the room
obstacle = simulator.loop([Point(2,3), Point(3,3), Point(2.5,4)])
#add the obstacle to the room (In python, + concatenates lists)
room = room + obstacle

#define the starting point of the lidar simulator
lidarPoint = Point(4,4)
lidarAngle = 3

#define data about the lidar simulator
num_scans =  50
lidarRange = 16
noise = 0.01

#create a udp socket to send data on
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

#function to send data over UDP
def sendUDP(x, y, theta):
    #generate binary data of the angle and position
    packed = pack('>fff', theta, x, y)
    
    #calculate a checksum, total off all data modulo 256
    sum = 0
    for c in packed:
        sum += ord(c)
    sum = sum % 256
    
    #generate binary data including checksum
    packed = pack('>fffB', theta, x, y, sum)
    
    #send data
    sock.sendto(packed, (UDP_IP, UDP_PORT))

#initialize lidar siulator
lidarSim = lidarSimulator(lidarPoint, lidarAngle, num_scans, lidarRange, noise, room)

#create laser model
laser = Laser(num_scans, 10, 360, 1000 * lidarRange, 0,0)

#initialize the slam
slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed = int(random.uniform(0,10000)))

#create empty lists to fill with points and plot when the program is done
positions = []
actual_pos = []

#number of loops, crude way of changing directions
i = 0

#repeat 500 times
for unused in range(500):
    #increment loop counter
    i += 1
    #move diagonally down/right half of the time, and the other way half of the time
    if i < 100:
        lidarSim.translate(Point(0.01, 0.01))
        lidarSim.rotate(0.01)
    else:
        lidarSim.translate(Point(-0.01,-0.01))
        lidarSim.rotate(-0.02)
    if i == 200:
        #reset timer if necesary
        i = 0
     
    #add the actual position to the list
    actual_pos.append((lidarSim.position.x, lidarSim.position.y, lidarSim.angle))
    
    #run the lidar simulator - this can be replaced by the code that reads from a real sensor
    lidarSim.simLidar()
    scan = []
    scan = lidarSim.getScan()
    
    #Run the actual slam algorithm to determine the position from the scan
    slam.update([1000 * s for s in scan])
    
    #get the position
    x,y,theta = slam.getpos()
    #add the position to the list to plot layer
    positions.append((x,y,theta))
    
    #uncomment to plot every 10th scan
    #if i %10 == 0:
        #lidarSim.plotScan('scan' + str(i))
    
    #print data for debugging and send it over UDP
    print("X: " + str(x/1000) + ", Y: " + str(y/1000) + ", Angle: " + str(theta))
    sendUDP(x / 1000 - 10,y / 1000 - 10,theta)
    
    #uncomment to scan at a realistic rate
    #time.sleep(0.1)

#plot the lidar scan and actual position
fig = plt.figure(1)
plt.plot([(p[0] - positions[0][0])/ 1000 for p in positions],[(p[1] - positions[0][1]) / 1000 for p in positions], 'b-')
plt.plot([p[0] - actual_pos[0][0] for p in actual_pos],[p[1] - actual_pos[0][1] for p in actual_pos], 'r-')

#find files with the same name and add a number on the end to avoid duplicate files
import os
i = 0
while os.path.exists('{}{:d}.png'.format('path', i)):
    i += 1
#save the figure with the two paths
fig.savefig('{}{:d}.png'.format('path', i))
