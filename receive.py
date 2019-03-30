import re
import subprocess
import serial
import os

os.system("python -m serial.tools.list_ports -v")

port = str(raw_input())

os.system("sudo chmod 666 "+port)

ser = serial.Serial(port, 115200)

while 1:
    print(ser.readline())