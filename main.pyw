from math import floor
from robot import Robot
import matplotlib.animation as animation
import matplotlib.colors as mplColors
import matplotlib.pyplot as plt
import matplotlib.pylab as lab
import numpy as np
import random
import time

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

# To animate
#os.chdir("C:\\Users\\Programming\\Desktop\\LIDAR-SI\\animated")

realMap = lab.imread('Map.png')
scaleSize = 40 # Number of pixels for 1 meter

period = 50 # Inverse of speed (time between 2 frame)
myRobot = Robot(4*scaleSize, 4, realMap, scaleSize) # Declare the Robot instance

# Generates the figures
cmap = mplColors.ListedColormap(['black','blue','grey','red', 'yellow'])
norm = mplColors.BoundaryNorm([    -1,     0,     1,     2,      3,    4], cmap.N)
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