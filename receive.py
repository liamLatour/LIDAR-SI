import os
import random
import re
import subprocess
import time
from math import cos, floor, sin

import matplotlib.pyplot as plt
import numpy as np
import serial

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
            #plt.scatter(cart[0], cart[1])

            if float(point[0]) <= 200:
                if float(point[1]) in mesure:
                    if len(mesure[float(point[1])]) > 100:
                        mesure[float(point[1])].pop()
                    mesure[float(point[1])].append(float(point[0]))
                else:
                    mesure[float(point[1])] = [float(point[0])]
        except Exception as e:
            print(e)

        plt.clf()
        for mes in mesure:
            cart = polToCar((0,0), sum(mesure[mes]) / len(mesure[mes]), mes)
            plt.scatter(cart[0], cart[1])
        plt.pause(0.01)
except KeyboardInterrupt:
    print("Saving file")
    plt.savefig('lidar.png')
    exit()

plt.show()
