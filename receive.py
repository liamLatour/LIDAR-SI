import os
import random
import re
import subprocess
import time
from math import cos, floor, sin, atan2, pi

import matplotlib.pyplot as plt
import numpy as np
import serial

import operator

def polToCar(center, dist, angle):
    return (cos(angle)*dist+center[0], sin(angle)*dist + center[1])

def moindreCarre(mesX, mesY):
    moyX = sum(mesX) / len(mesX)
    moyY = sum(mesY) / len(mesY)

    denom = sum( pow(np.subtract(mesX,moyX), 2) )

    if denom == 0:
        denom = 0.000000001

    a = sum( np.subtract(mesX,moyX)*np.subtract(mesY,moyY) ) / denom
    b = moyY-(a*moyX)
    return (a, b)

def angleBetweenLines(a1, a2):
    denom = 1+a1*a2
    if denom == 0:
        denom = 0.000000001
    return atan2(a1-a2, denom)

def projeter(a, b, u, v):
    #d = v+u/a
    #y = (b+a*d)/(1+a)
    #x = (y-b)/a

    return (u, u*a+b)

def filterLine(mesures):
    # always keep smallest angle
    mesX = mesures[0]
    mesY = mesures[1]

    newMesuresX = []
    newMesuresY = []
    '''
    droite = moindreCarre(mesX, mesY)

    for last in range(len(mesX)):
        newPoint = projeter(droite[0], droite[1], mesX[last], mesY[last])
        newMesuresX.append(newPoint[0])
        newMesuresY.append(newPoint[1])
    '''

    averageOn = 5
    
    droite = moindreCarre(mesX[0 : averageOn], mesY[0 : averageOn])

    currentA = droite[0]
    currentB = droite[1]
    clampedPoints = averageOn

    for i in range(averageOn+1, len(mesX)):
        droite = moindreCarre(mesX[i-averageOn : i], mesY[i-averageOn : i])

        if angleBetweenLines(droite[0], currentA) > pi/6: # point is on a new line

            # clamp all previous points
            for last in range(i-clampedPoints, i):
                newPoint = projeter(currentA, currentB, mesX[last], mesY[last])
                newMesuresX.append(newPoint[0])
                newMesuresY.append(newPoint[1])

            currentA = droite[0]
            currentB = droite[1]
            clampedPoints = averageOn
        else: # point is on current line
            clampedPoints += 1
            droite = moindreCarre( mesX[i-clampedPoints : i], mesY[i-clampedPoints : i] )
            currentA = droite[0]
            currentB = droite[1]
    try:
        for last in range(len(mesX)-clampedPoints, len(mesX)):
            newPoint = projeter(currentA, currentB, mesX[last], mesY[last])
            newMesuresX.append(newPoint[0])
            newMesuresY.append(newPoint[1])
    except:
        pass

    return [newMesuresX, newMesuresY]

def toArray(mesures):
    for mes in mesures:
        mesures[mes] = sum(mesures[mes]) / len(mesures[mes])

    sorted_mesures = sorted(mesures.items(), key=operator.itemgetter(0))

    newMesuresX = []
    newMesuresY = []

    for mes in sorted_mesures:
        cart = polToCar((0,0), mes[1], float(mes[0]))

        newMesuresX.append(cart[0])
        newMesuresY.append(cart[1])

    return [newMesuresX, newMesuresY]

try:
    os.system("python -m serial.tools.list_ports -v")
    port = str(input())
    ser = serial.Serial(port, 115200, timeout=0.5)

    mesure = {}

    plt.axis('equal')
    while True:
        ser.write(bytes([1]))
        point = ser.readline()
        try:
            point = point.decode().split(':')

            #cart = polToCar((0,0), float(point[0]), float(point[1]))
            #plt.scatter(cart[0], cart[1], alpha=0.5, s=5, c="black")

            
            if float(point[0]) <= 200:
                if float(point[1]) in mesure:
                    if len(mesure[float(point[1])]) > 20:
                        mesure[float(point[1])].pop()
                    mesure[float(point[1])].append(float(point[0]))
                else:
                    mesure[float(point[1])] = [float(point[0])]
            
        except Exception as e:
            print(point)
            print(e)

        if len(mesure) > 1:
            plt.clf()
            plt.axis('equal')
            result = toArray(mesure.copy())
            filtered = filterLine(result)
            #plt.scatter(filtered[0], filtered[1], alpha=1, s=10, c="black")
            plt.scatter(result[0], result[1], alpha=0.5, s=5, c="red")
        
        plt.pause(0.01)
except KeyboardInterrupt:
    print("Saving file")
    plt.savefig('lidar.png')
    exit()

plt.show()