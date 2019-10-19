import math
import threading
import time

import serial
from kivy.app import App
from kivy.clock import Clock
from kivy.core.text import Label as CoreLabel
from kivy.graphics import Color, Ellipse, Rectangle
from kivy.uix.actionbar import ActionDropDown
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button

from functions import getPorts


class MenuDropDown(ActionDropDown):
    pass


class UI(BoxLayout):
    def __init__(self, *args, **kwargs):
        super(UI, self).__init__(*args, **kwargs)
        self.settings = App.get_running_app().config
        self.portDropDown = MenuDropDown()
        self.diametre = 5

        self.points = {}
        self.zoomFactor = 1
        self.offset = [0, 0]
        self.curOffest = [0, 0]
        self.dragStart = -1

        self.board = -1
        self.avgLen = 20
        self.lastAngle = None

        self.motorWrite = False
        self.motorOn = 1
        self.speed = 3

        self.ids.motorSwitch.bind(active=self.motorActive)
        self.ids.motorSpeed.bind(value=self.motorSpeed)
        Clock.schedule_interval(self.update_rect, 0.04)

    def show_port(self):
        self.portDropDown.open(self.ids.port_button)
        self.portDropDown.clear_widgets()

        self.portDropDown.add_widget(Button(
            text="No port", height=48, size_hint_y=None, on_release=lambda a: self.change_port(-1)))
        arduinoPorts = getPorts()
        for port in arduinoPorts:
            self.portDropDown.add_widget(Button(
                text=port, height=48, size_hint_y=None, on_release=lambda a: self.change_port(port)))

    def startReading(self, port):
        self.board = serial.Serial(str(port), 2000000, timeout=0.1)
        time.sleep(2)
        while True:
            if self.motorWrite:
                #self.board.write(bytes([self.motorOn]))

                self.board.write(bytes([self.speed]))
                print(self.board.read())

                self.motorWrite = False
            time.sleep(0.1)
            """
            self.board.write(bytes([0]))
            p = self.board.readline().decode().split(':')

            if float(p[1]) in self.points:
                if len(self.points[float(p[1])]) > self.avgLen:
                    self.points[float(p[1])].pop()
                self.points[float(p[1])].append(int(p[0])/100)
            else:
                self.points[float(p[1])] = [int(p[0])/100]
            
            self.lastAngle = float(p[1])
            """

    def change_port(self, port):
        self.ids.port_button.text = "Port"
        if port != -1:
            self.ids.port_button.text = "Port" + " " + port
            threading.Thread(target=lambda: self.startReading(port), daemon=True).start()

        self.portDropDown.dismiss()

    def update_rect(self, *args):
        self.ids.libreDrawing.canvas.before.clear()

        middle = (self.ids.libreDrawing.center_x,
                  self.ids.libreDrawing.center_y)
        xOffset = middle[0]-self.diametre/2 + (self.offset[0]+self.curOffest[0]) * self.zoomFactor
        yOffset = middle[1]-self.diametre/2 + (self.offset[1]+self.curOffest[1]) * self.zoomFactor

        with self.ids.libreDrawing.canvas.before:
            Color(1, 0, 0, 1)
            Ellipse(pos=(xOffset, yOffset),
                    size=(self.diametre, self.diametre))
            Color(0, 1, 0, 0.7)
            points = self.points.copy()
            for p in points:

                dist = sum(self.points[p]) / len(self.points[p])

                point = (math.cos(p)*dist, math.sin(p)*dist)
                Ellipse(pos=(xOffset + point[0] * self.zoomFactor,
                             yOffset + point[1] * self.zoomFactor),
                             size=(self.diametre, self.diametre))

            if self.lastAngle != None:
                text = "Distance: " + str(round( (sum(self.points[self.lastAngle]) / len(self.points[self.lastAngle])) *10)/10)

                Color(0.5, 0.5, 0.5, .6)
                Rectangle(pos=(self.size[0]-200, self.size[1] - (70+50)), size=(200, 70))

                label = CoreLabel(text=text, font_size=30, halign='left', valign='top', padding=(5, 5))
                label.refresh()
                text = label.texture
                Color(1, 1, 1, 1)
                Rectangle(pos=(self.size[0]-200, self.size[1] - (70+50)), size=(200, 70), texture=text)

    def clickedDown(self, touch):
        x = touch.x
        y = touch.y

        if self.ids.libreDrawing.collide_point(x, y):
            if touch.is_mouse_scrolling:
                self.zoomFactor += 0.3 if touch.button == 'scrollup' else -0.3
                self.zoomFactor = max(self.zoomFactor, 0.001)
            self.dragStart = (touch.x, touch.y)

    def clickedUp(self, touch):
        self.dragStart = -1
        self.offset[0] += self.curOffest[0]
        self.offset[1] += self.curOffest[1]
        self.curOffest = [0, 0]

    def clickedMove(self, touch):
        if self.dragStart != -1:
            self.curOffest = ((touch.x-self.dragStart[0])/self.zoomFactor, (touch.y-self.dragStart[1])/self.zoomFactor)

    def motorActive(self, *args):
        self.motorOn = 2
        if args[1]:
            self.motorOn = 1
        self.motorWrite = True

    def motorSpeed(self, *args):
        print(args[1])
        self.speed = args[1]
        self.motorWrite = True
