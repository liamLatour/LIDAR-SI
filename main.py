import matplotlib.animation as animation
from bresenham import bresenham
import matplotlib.pyplot as plt
import matplotlib.pylab as lab
import matplotlib as mpl
import numpy as np
from math import *
import random
import time


class Robot:
    def __init__(self, lineOfSight, speed, realMap):
        self.position = (150, 50)
        self.knownMap = np.zeros((len(realMap), len(realMap)), dtype=int)
        self.lineOfSight = lineOfSight
        self.realMap = realMap
        self.maxSpeed = speed
        self.speed = 2
        self.movement = pi # angle where it is going

    def polToCar(self, center, dist, angle):
        return (round(cos(angle)*dist+center[0]), round(sin(angle)*dist + center[1]))

    def point(self, position, size, color):
        offset = round(size/2 - 0.5)
        size = round(size)
        for x in range(size):
            for y in range(size):
                self.knownMap[ position[0]+x-offset ][ position[1]+y-offset ] = color

    def line(self, center, radius, angle, color):
        end = self.polToCar(center, radius, angle)
        points = list(bresenham(center[0], center[1], end[0], end[1]))
        
        for point in points:
            self.knownMap[ point[0] ][ point[1] ] = color
            if self.realMap[ point[0] ][ point[1] ][0] == 1:
                self.knownMap[ point[0] ][ point[1] ] = 1
            else:
                return sqrt(pow(abs(center[0]-point[0]), 2) + pow(abs(center[1]-point[1]), 2))/scaleSize # Return number of meters from hit
        return float('inf') # To be sure not to mistaken with real hits

    def LIDAR(self):
        alreay = False
        for angle in range(0, 360, 4):
            rad = (angle/360)*(2*pi)
            dist = self.line(self.position, self.lineOfSight, rad, 3)
            if dist < 0.5 and not alreay:
                changeAngle = pi/8
                if self.movement-rad > 0:
                    self.movement += changeAngle
                else:
                    self.movement -= changeAngle
                self.speed = 2
                alreay = True

    def move(self):
        self.point(self.position, scaleSize/5, 1)
        self.position = self.polToCar(self.position, self.speed, self.movement)
        self.LIDAR()
        self.point(self.position, scaleSize/5, 2)

        self.speed = min(self.speed + 0.02, self.maxSpeed)
        return self.knownMap
        

# Main loop of the program
def updatefig(*args):
    global myRobot
    im.set_array(myRobot.move())
    return im,


realMap = lab.imread('Map.png')

scaleSize = 40 # Number of pixels for 1 meter

period = 50 # Inverse of speed (time between 2 frame)
myRobot = Robot(4*scaleSize, 3, realMap) # Declare the Robot instance

# Generates the figures
cmap = mpl.colors.ListedColormap(['black','grey','red','blue'])
norm = mpl.colors.BoundaryNorm([0,1,2,3,4], cmap.N)
fig = plt.figure()
im = plt.imshow(np.zeros((len(realMap), len(realMap)), dtype=int),interpolation='nearest',
                        cmap=cmap,norm=norm,origin="lower", animated=True)

ani = animation.FuncAnimation(fig, updatefig, interval=period, blit=True)
plt.show()