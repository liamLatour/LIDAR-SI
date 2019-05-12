import os
import random
import re
import subprocess
import time
from math import cos, floor, sin

import matplotlib.pyplot as plt
import numpy as np
import serial

import operator


def doAverage(mesures, averageSize=2):
    for mes in mesures:
        mesures[mes] = sum(mesures[mes]) / len(mesures[mes])

    sorted_mesures = sorted(mesures.items(), key=operator.itemgetter(0))

    newMesuresX = []
    newMesuresY = []

    for counter, mes in enumerate(sorted_mesures):
        tempArray = []
        for i in range(-averageSize, averageSize+1):
            tempArray.append(sorted_mesures[min(max(i+counter, 0), len(sorted_mesures)-1)][1])  #min(max(i, 0), len(sorted_mesures)-1)

        cart = polToCar((0,0), sorted(tempArray)[averageSize], mes[0])

        newMesuresX.append(cart[0])
        newMesuresY.append(cart[1])

    return [newMesuresX, newMesuresY]


def polToCar(center, dist, angle):
    return (cos(angle)*dist+center[0], sin(angle)*dist + center[1])


try:
    os.system("python -m serial.tools.list_ports -v")
    port = str(input())
    ser = serial.Serial(port, 115200, timeout=0.5)

    mesure = {}

    plt.axis([-200, 200, -200, 200])
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
            print(e)
        
        plt.clf()
        result = doAverage(mesure.copy(), averageSize=4)
        plt.scatter(result[0], result[1])

        plt.pause(0.01)
except KeyboardInterrupt:
    print("Saving file")
    plt.savefig('lidar.png')
    exit()

plt.show()