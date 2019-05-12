from math import floor
import numpy as np
import time

import matplotlib.animation as animation
import matplotlib.colors as mplColors
import matplotlib.pyplot as plt
import matplotlib.pylab as lab

import robot

trace = []

# Main loop of the program
def updatefig(i):
    global barGraph
    global myRobot
    global trace

    curentFrame = myRobot.move() # Asks for the next move of the robot

    for i,bar in enumerate(barGraph):
        bar.set_height(curentFrame[1][i]) # Displays the bar graph representing what it sees

    mine = np.copy(curentFrame[0])
    for hole in curentFrame[2]:
        pos = robot.polToCar(hole[0], 2.3*40+1, hole[1])
        mine[pos[0]][pos[1]] = 3

    
    trace.append(curentFrame[3])

    for pos in trace:
        mine[pos[0]][pos[1]] = 2

    im.set_array(mine) # Displays the map with the robot position

    return patches

# Get clicks on the map
def onclick(event):
    global myRobot
    if event.dblclick: # Only if we double clicked
        print("Clicked at x=%d, y=%d" % (event.xdata, event.ydata))
        myRobot.FindPath((floor(event.xdata), floor(event.ydata)), True) # Sends data to the robot for it to aim at the waypoint

# To animate
#os.chdir("C:\\Users\\Programming\\Desktop\\LIDAR-SI\\animated")

realMap = lab.imread('aurelien.png') # Loads the image of the map
scaleSize = 40 # Number of pixels for 1 meter

period = 20 # Inverse of speed (time between 2 frames)
myRobot = robot.Robot(2.3*scaleSize, 4, realMap, scaleSize) # Declares the Robot instance

# Generates the figures
cmap = mplColors.ListedColormap(['black','blue','grey','red', 'white']) # All the used colors
norm = mplColors.BoundaryNorm([    -1,     0,     1,     2,      3,    4], cmap.N) # Maps the colors to ints
fig = plt.figure()
fig.canvas.mpl_connect('button_press_event', onclick) # Binds the double click to the map

plt.subplot(1, 2, 1)
im = plt.imshow(np.zeros((len(realMap), len(realMap)), dtype=int),interpolation='nearest',
                        cmap=cmap,norm=norm, animated=True) # Map (image) instance

axes2 = plt.subplot(1, 2, 2)
axes2.set_ylim([0, 4.2])
barGraph = plt.bar(range(90), range(90), width=1) # Bar graph instance

patches = [im] + list(barGraph)

ani = animation.FuncAnimation(fig, updatefig, interval=period, blit=True) # Animates eveything
plt.show() # Finally displays everything