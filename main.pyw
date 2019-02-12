
import matplotlib.animation as animation
import matplotlib.colors as mplColors
import matplotlib.pyplot as plt
import matplotlib.pylab as lab
from bresenham import bresenham
import numpy as np
from math import *
import random
import time

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.grid import Grid

# TODO: get the movement angle when moving with A*


class Robot:
    def __init__(self, lineOfSight, speed, realMap):
        self.position = (scaleSize+5, scaleSize)
        self.knownMap = np.zeros((len(realMap), len(realMap)), dtype=int) - 1
        self.lineOfSight = lineOfSight
        self.realMap = realMap
        self.maxSpeed = speed
        self.speed = 2
        self.movement = pi/2 # angle where it is going
        self.mode = "DISCOVER" # can be "DISCOVER" or "STOP"
        self.path = None
        self.whereInPath = 0
        self.angleResolution = 4 
        self.safeAngle = int(8 / self.angleResolution)

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
        distances = np.zeros(int(360/self.angleResolution))
        for angle in range(0, 360, self.angleResolution):
            rad = (angle/360)*(2*pi)
            dist = self.line(self.position, self.lineOfSight, rad, 0)
            distances[int(angle/self.angleResolution)] = (self.lineOfSight+0.1)/scaleSize - dist

        shiftDeg = -int((self.movement/(2*pi) * 360 - 180)/self.angleResolution)
        distances = np.roll(distances, shiftDeg)

        return distances

    def obstacleAvoidance(self, distances):
        if self.mode == "DISCOVER":
            # Check it can continue forward
            canContinue = True

            for i in range(-int(self.safeAngle/2), int(self.safeAngle/2)):
                if distances[int(len(distances)/2 + i)] > 3.5:
                    canContinue = False
                    break

            if not canContinue:
                newAngleIndex = -1
                nbGood = 0

                for i in range(int((len(distances)+self.safeAngle)/2), len(distances)):
                    if distances[i] < 3.5:
                        nbGood = nbGood + 1
                        if nbGood == self.safeAngle:
                            newAngleIndex = i
                            break
                    else:
                        nbGood = 0

                if newAngleIndex == -1:
                    newAngleIndex = len(distances)-1

                nbGood = 0
                for i in range(int((len(distances)-self.safeAngle)/2), int(len(distances)-newAngleIndex-self.safeAngle/2), -1):
                    if distances[i] < 3.5:
                        nbGood = nbGood + 1
                        if nbGood == self.safeAngle:
                            newAngleIndex = i
                            break
                    else:
                        nbGood = 0

                self.movement = ((newAngleIndex*4) / 360) * 2*pi

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
        dists = self.LIDAR()
        self.obstacleAvoidance(dists)
        self.point(self.position, scaleSize/10, 2)

        self.speed = min(self.speed + 0.02, self.maxSpeed)
        return [self.knownMap, dists]
        

# Main loop of the program
def updatefig(i):
    global barGraph
    global myRobot

    curentFrame = myRobot.move()
    im.set_array(curentFrame[0])

    for i,bar in enumerate(barGraph):
        bar.set_height(curentFrame[1][i])

    return patches

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
cmap = mplColors.ListedColormap(['black','blue','grey','red'])
norm = mplColors.BoundaryNorm([    -1,     0,     1,     2,   3], cmap.N)
fig = plt.figure()
fig.canvas.mpl_connect('button_press_event', onclick)

plt.subplot(1, 2, 1)
im = plt.imshow(np.zeros((len(realMap), len(realMap)), dtype=int),interpolation='nearest',
                        cmap=cmap,norm=norm, animated=True)

axes2 = plt.subplot(1, 2, 2)
axes2.set_ylim([0, 4.2])
barGraph = plt.bar(range(90), range(90), width=1)

patches = [im] + list(barGraph)

ani = animation.FuncAnimation(fig, updatefig, interval=period, blit=True)
plt.show()