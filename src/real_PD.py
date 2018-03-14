#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Author: Misael Antonio Alarcón Herrera
    Description: Script to control the AR Drone 2.0 using OptiTrack system.
                 The control law used in this script is show in
                 Santiaguillo - Seguimiento de Trayectorias para un
                 Helicoptero de 4 Rotores AR.Drone 2.0 Utilizando ROS
"""

import sys
import os
import time
import logging
from math import pi, sqrt, sin, cos, copysign
import rospy
import rospkg
from lib.Controller import Controller
from lib.drone_utils import deriv, degtorad, filter_FIR
from std_msgs.msg import Float32MultiArray
from ardrone_autonomy.msg import Navdata
import PySide
from pyqtgraph import QtGui,QtCore

class PD(Controller):
    def __init__(self,model, num_widgets, rows, cols, titulos):
        super(PD,self).__init__(model, num_widgets, rows, cols, titulos)

        # Gains
        self.kdx = .3
        self.kdy = .3
        self.kdz = 2
        self.kpx = 1.5
        self.kpy = 3
        self.kpz = 2
        self.kpa = 3

        # Trajectory
        # # Point
        # self.A = 0.
        # self.B = 0.
        # self.C = 0.
        # self.D = 0.
        # self.E = 0.
        # self.T = 20.

        # # Lemniscate
        # self.A = 0.5
        # self.B = 2.
        # self.C = 0.5
        # self.D = 4.
        # self.E = 0.2
        # self.T = 120.

        # Oval
        self.A = 0.5
        self.B = 2.
        self.C = 0.5
        self.D = 2.
        self.E = 0.2
        self.T = 30.

        self.repeat = 1.
        self.psid = degtorad(0)

        # Center of the trajectory
        self.z0 = 0.8

        logger.info('Simulation: {}, Control Period: {} ms, '
                    'Maximum tilt: {} deg'.format(sim_num, self.control_period,
                                                  maxTilt))
        logger.info(
            'Gains: {}, Trajectory: {}'.format((self.kdx, self.kdy, self.kpx,
                                                self.kpy, self.kpz, self.kpa),
                                               (self.A, self.B, self.C, self.D,
                                                self.E, self.alt_0, self.T,
                                                self.repeat, self.psid)))
        logger.info('Center {}'.format((self.x0, self.y0, self.z0)))

        time.sleep(5)  # Wait for the communication to stablish

        # Suscribers and timers
        self.subPosition = rospy.Subscriber('/droneLeader', Float32MultiArray,
                                            self.ReceivePosition)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata,
                                           self.ReceiveNavdata)

    def leyControl(self,_):
        if self.start and self.t < self.repeat*self.T:
            prev = time.time()
            self.t += self.h
            g = 9.8086

            # Control
            a = self.rx(self.xPos, self.vx, self.t)
            b = self.ry(self.yPos, self.vy, self.t)

            # Desired Angles
            self.phid = (1 / g) * (a * sin(self.psid) - b * cos(self.psid))
            self.thetad = (1 / g) * (a * cos(self.psid) + b * sin(self.psid))

            # Derivative of desired angles
            self.phidp = deriv(self.phid, self.p_phid, self.h)
            self.thetadp = deriv(self.thetad, self.p_thetad, self.h)

            # Derivative of drone angles
            self.phip = deriv(self.rotationX, self.p_rotationX, self.h)
            self.thetap = deriv(self.rotationY, self.p_rotationY, self.h)

            # Low Pass Filter
            self.vz = deriv(self.zPos, self.p_zPos, self.h)
            self.vz = filter_FIR(0.0005, self.hist_vz, self.vz)
            self.vyaw = deriv(self.rotationZ, self.p_rotationZ, self.h)
            self.vyaw = filter_FIR(0.0005, self.hist_vyaw, self.vyaw)

            # Roll & Pitch Control Signals
            self.roll = self.phid / degtorad(maxTilt)
            if self.roll > 1:
                self.roll = 1
            elif self.roll < -1:
                self.roll = -1
            self.pitch = self.thetad / degtorad(maxTilt)
            if self.pitch > 1:
                self.pitch = 1
            elif self.pitch < -1:
                self.pitch = -1

            # Z velocity control signal
            self.z_velocity = self.mzp(self.t)-self.kpz*(self.zPos-self.mz(self.t))

            # Yaw velocity control signal
            self.yaw_velocity = -self.kpa * (self.rotationZ - self.psid)

            self.SetCommand(-self.roll, self.pitch, -self.yaw_velocity,
                                  self.z_velocity)

            k = (time.time()-prev)*1000
            if k > self.control_period:
                msg = 'Control law function taking more time than ' \
                      'control period: {:3} ms'.format(k)
                rospy.logwarn(msg)
                logger.warning(msg)
        else:
            if not self.start and self.t == 0:
                time.sleep(5)
                rospy.logwarn('Taking Off')
                logger.info('Taking Off')
                self.SendTakeoff()
                self.start = True
            else:
                logger.info('Landing')
                self.paroEmergencia()

    def ReceivePosition(self, pos_data):
        """ Overwrites the method of class Controller.
        Function that receives position and attitude from the droneLeader
        pulisher of the socket_optitrack package"""
        self.xPos = pos_data.data[0]
        self.yPos = pos_data.data[1]
        self.zPos = pos_data.data[2]
        self.rotationZ = pos_data.data[3]
        self.rotationX = pos_data.data[4]
        self.rotationY = pos_data.data[5]

    def ReceiveNavdata(self, navdata):
        """ Overwrites the method of class Controller.
        Function that receives velocity in x and y from the Navdata
        publisher"""
        self.status = navdata.state
        self.vx = navdata.vx / 1000.0  # Meters per second
        self.vy = navdata.vy / 1000.0  # Meters per second

# Setup the application
if __name__=='__main__':
    maxTilt = 5.73  # degrees
    sim_num = 1
    ejes = ['Roll', 'Pitch', 'X',
            'Y', 'Z', 'Yaw']

    # Create Folder
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('drone_cinvestav')
    res_path = '/src/Results/Experiments/'
    exp_path = 'PD/'
    folder_path = str(sim_num)
    path = pkg_path + res_path + exp_path + folder_path

    while os.path.exists(path):
        sim_num += 1
        folder_path = str(sim_num)
        path = pkg_path + res_path + exp_path + folder_path

    os.mkdir(path)

    # Create and  configure logger
    name = __file__
    logger = logging.getLogger(name.split('/')[-1][:-3].upper())
    logger.setLevel(logging.INFO)
    LOG_FORMAT = "[%(levelname)s] [%(name)s] [%(asctime)s] --> %(message)s"
    formatter = logging.Formatter(LOG_FORMAT)
    file_handler = logging.FileHandler(path + '/info.log')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    logger.info(4 * '#########################')
    # Initialize ROS node
    rospy.init_node('GUI')
    logger.info('Rospy node created')

    # Construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    controller = PD('ardrone', 2, 2, 3, ejes)
    logger.info('Qt GUI created')
    controller.show()

    # executes the QT application
    status = app.instance().exec_()
    logger.info('Qt GUI executed')

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    logger.info('Rospy node closed')
    controller.graficar(show=False)
    sys.exit(status)
