#!/usr/bin/env python

import rospy

from ardrone_autonomy.msg import Navdata
from gazebo_msgs.msg import LinkStates
from drone_status import DroneStatus
from std_msgs.msg import Float32MultiArray

import PySide
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from drone_utils import L

UPDATE_TIME = 5

class View_3D(QtGui.QWidget):

    StatusMessages = {
        DroneStatus.Emergency: 'Emergency',
        DroneStatus.Inited: 'Initialized',
        DroneStatus.Landed: 'Landed',
        DroneStatus.Flying: 'Flying',
        DroneStatus.Hovering: 'Hovering',
        DroneStatus.Test: 'Test (?)',
        DroneStatus.TakingOff: 'Taking Off',
        DroneStatus.GotoHover: 'Going to Hover Mode',
        DroneStatus.Landing: 'Landing',
        DroneStatus.Looping: 'Looping (?)'
    }
    DisconnectedMessage = 'Disconnected'
    UnknownMessage = 'Unknown Status'

    def __init__(self):
        super(View_3D,self).__init__()

        self.setWindowTitle('3D Path')

        self.layout = QtGui.QGridLayout()
        self.setLayout(self.layout)
        self.j = 0

        self.w1 = gl.GLViewWidget()
        self.l1 = QtGui.QLabel()
        self.layout.addWidget(self.w1, 0, 0)
        self.layout.addWidget(self.l1, 1, 0)
        self.w1.setMinimumSize(384, 360)
        self.w1.opts['distance'] = 40
        self.l1.setText('Hola')

        gz = gl.GLGridItem()
        self.w1.addItem(gz)

        self.subGazebo = rospy.Subscriber('/gazebo/link_states', LinkStates, self.ReceivePosicion)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

        self.x = L(2)
        self.y = L(2)
        self.z = L(2)

        self.statusMessage = ''

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(UPDATE_TIME)

    def update_data(self):

        pts = np.vstack([self.x,self.y,self.z]).transpose()
        plt = gl.GLLinePlotItem(pos=pts,color=pg.glColor((0,25)),width=5,antialias=True)
        self.w1.addItem(plt)

        self.l1.setText(self.statusMessage)

    def ReceivePosicion(self,data):
        if self.j == 5:
            try:
                self.x.append(data.pose[-1].position.x)
                self.y.append(data.pose[-1].position.y)
                self.z.append(data.pose[-1].position.z)

                self.dataPos.insert(0, self.x)
                self.dataPos.insert(1, self.y)
                self.dataPos.insert(2, self.z)
            except AttributeError:
                pass
            self.j = 0
        else:
            self.j+=1

    def ReceiveNavdata(self,navdata):
        msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        self.statusMessage = '{} (Battery: {}  %)'.format(msg,int(navdata.batteryPercent))


if __name__ == "__main__":
    import sys
    rospy.init_node('Path_GUI')
    app = QtGui.QApplication(sys.argv)
    graph2 = View_3D()
    graph2.show()
    app.instance().exec_()