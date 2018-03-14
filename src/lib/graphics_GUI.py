#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Author: Misael Antonio Alarcón Herrera
    Description: UI to display graphics of position, attitude and velocity of
                 the AR Drone in execution time.
"""

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

from ardrone_autonomy.msg import Navdata
from gazebo_msgs.msg import LinkStates

import PySide
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
from drone_utils import L


UPDATE_TIME = 2

class windowGraphics(QtGui.QWidget):
    def __init__(self,num_widgets,rows,cols,titulos):
        super(windowGraphics,self).__init__()

        self.rows = rows
        self.cols = cols
        self.titulos = titulos

        self.layout = QtGui.QGridLayout()
        self.setLayout(self.layout)
        self.setWindowTitle('Graficas')
        self.widgets = []
        self.plots = []
        self.curves = []
        for i in range(num_widgets):
            self.curves.append([])
        self.sync = 0

        for i in range(num_widgets):
            w = pg.GraphicsWindow()
            self.layout.addWidget(w, i, 0)
            self.widgets.append(w)

        for h,widget in enumerate(self.widgets):
            m = 0
            for i in range(self.rows):
                for j in range(self.cols):
                    p = widget.addPlot()
                    self.plots.append(p)
                    p.setLabel('left',self.titulos[m])
                    p.setDownsampling(mode='peak')
                    p.setClipToView(True)
                    pp = p.plot(pen=(155,255,50))
                    pp1 = p.plot(pen=(55,130,255))
                    self.curves[h].append((pp,pp1))
                    m+=1
                widget.nextRow()

        maxlen = 2000
        self.xg = L(maxlen)
        self.yg = L(maxlen)
        self.zg = L(maxlen)
        self.yawg = L(maxlen)
        self.vxg = L(maxlen)
        self.vyg = L(maxlen)
        self.vzg = L(maxlen)
        self.vyawg = L(maxlen)

        self.dataPos = []
        self.dataVel = []
        for i in range(self.rows*self.cols):
            self.dataPos.append([])
            self.dataVel.append([])

        timer = QtCore.QTimer(self)
        self.connect(timer, QtCore.SIGNAL("timeout()"), self.update_data)
        timer.start(UPDATE_TIME)

    def update_data(self):
        for h,widget in enumerate(self.widgets):
            m = 0
            for curve in self.curves[h]:
                if h == 0:
                    curve[0].setData(self.dataPos[m])
                    curve[1].setData(self.dataPosD[m])
                if h == 1:
                    curve[0].setData(self.dataVel[m])
                    curve[1].setData(self.dataVelD[m])
                m+=1
                if m == len(self.dataPos):
                    m = 0