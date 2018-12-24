import matplotlib.animation as animation
from bresenham import bresenham
import matplotlib.pyplot as plt
import matplotlib.pylab as lab
import matplotlib as mpl
import numpy as np
from math import *
import random
import time

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Robot:
    def __init__(self, lineOfSight, speed, realMap):
        self.position = (scaleSize+5, scaleSize)
        self.knownMap = np.zeros((len(realMap), len(realMap)), dtype=int) - 1
        self.lineOfSight = lineOfSight
        self.realMap = realMap
        self.maxSpeed = speed
        self.speed = 2
        self.movement = pi # angle where it is going
        self.mode = "DISCOVER" # can be "DISCOVER" or "STOP"
        self.path = None
        self.whereInPath = 0

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
            self.knownMap[ point[0] ][ point[1] ] = 1
            if self.realMap[ point[0] ][ point[1] ][0] != 1:
                self.knownMap[ point[0] ][ point[1] ] = color
                return sqrt(pow(abs(center[0]-point[0]), 2) + pow(abs(center[1]-point[1]), 2))/scaleSize # Return number of meters from hit

        return float('inf') # To be sure not to mistaken with real hits

    def FindPath(self, destination):
        grid = Grid(matrix=self.knownMap)
        start = grid.node(self.position[1], self.position[0])
        end = grid.node(destination[0], destination[1])

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        self.path, runs = finder.find_path(start, end, grid)

        if self.path != []:
            self.whereInPath = 0
            self.mode = "FOLLOW"

    def LIDAR(self):
        alreay = False
        for angle in range(0, 360, 4):
            rad = (angle/360)*(2*pi)
            dist = self.line(self.position, self.lineOfSight, rad, 0)
            if self.mode == "DISCOVER":
                if dist < 0.5 and not alreay:
                    changeAngle = pi/4
                    if self.movement-angle > 0:
                        self.movement += changeAngle
                    else:
                        self.movement -= changeAngle
                    self.speed = 0.7
                    alreay = True

        if self.mode == "DISCOVER":
            self.position = self.polToCar(self.position, self.speed, self.movement)
        elif self.mode == "FOLLOW":
            if ceil(self.whereInPath) < len(self.path):
                self.position = (self.path[ceil(self.whereInPath)][1], self.path[ceil(self.whereInPath)][0])
                self.whereInPath += self.maxSpeed
            else:
                self.mode = "DISCOVER"
        elif self.mode == "STOP":
            self.speed = 0        

    def move(self):
        self.LIDAR()
        self.point(self.position, scaleSize/10, 2)

        self.speed = min(self.speed + 0.02, self.maxSpeed)
        return self.knownMap
        

# Main loop of the program
def updatefig(*args):
    global myRobot
    im.set_array(myRobot.move())
    return im,

# Get clicks
def onclick(event):
    global myRobot
    if event.dblclick:
        print("Clicked at x=%d, y=%d" % (event.xdata, event.ydata))
        myRobot.FindPath((floor(event.xdata), floor(event.ydata)))




realMap = lab.imread('Map.png')
scaleSize = 40 # Number of pixels for 1 meter

period = 50 # Inverse of speed (time between 2 frame)
myRobot = Robot(4*scaleSize, 4, realMap) # Declare the Robot instance

# Generates the figures
cmap = mpl.colors.ListedColormap(['black','blue','grey','red'])
norm = mpl.colors.BoundaryNorm([    -1,     0,     1,     2,   3], cmap.N)
fig = plt.figure()
fig.canvas.mpl_connect('button_press_event', onclick)
im = plt.imshow(np.zeros((len(realMap), len(realMap)), dtype=int),interpolation='nearest',
                        cmap=cmap,norm=norm, animated=True)

ani = animation.FuncAnimation(fig, updatefig, interval=period, blit=True)
plt.show()