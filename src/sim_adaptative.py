#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Author: Misael Antonio Alarcón Herrera
    Description: Script to control the AR Drone 2.0 using Gazebo Simulator.
                 The control law used in this script is show in
                 Xun Gong - Adaptive Backstepping Sliding Mode Trajectory
                 Tracking Control for a Quad-rotor
"""

import sys
import os
import time
import logging
from math import pi, sqrt, asin, atan, copysign
import rospy
import rospkg
from lib.Controller import Controller
from lib.drone_utils import deriv, degtorad, filter_FIR
from gazebo_msgs.msg import LinkStates
from ardrone_autonomy.msg import Navdata
import PySide
from pyqtgraph import QtGui, QtCore

class Adaptative(Controller):
    def __init__(self, model, num_widgets, rows, cols, titulos):
        super(Adaptative, self).__init__(model, num_widgets, rows, cols,
                                         titulos)

        self.control_period = 5.
        self.maxlen = 1000.
        self.visualize = True
        self.path = path

        # Gains
        self.kdx = 4
        self.kdy = 2
        self.kdz = 1
        self.kpx = 8
        self.kpy = 6
        self.kpz = 4
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
        self.T = 120.

        self.repeat = 1.
        self.psid = degtorad(0)

        # Center of the trajectory
        self.z0 = 0.8

        logger.info(
            'Simulation: {}, Control Period: {} ms, '
            'Maximum tilt: {} deg'.format(sim_num, self.control_period,
                                          maxTilt))
        logger.info(
            'Gains: {}, Trajectory: {}'.format((self.kdx, self.kdy, self.kdz,
                                                self.kpx, self.kpy, self.kpz,
                                                self.kpa),
                                               (self.A, self.B, self.C, self.D,
                                                self.E, self.T, self.repeat,
                                                self.psid)))
        logger.info('Center {}'.format((self.x0, self.y0, self.z0)))

        # Suscribers and timers
        rospy.Subscriber('/gazebo/link_states', LinkStates,
                         self.ReceivePosition)
        rospy.Subscriber('/' + model + '/navdata', Navdata,
                         self.ReceiveNavdata)

    def leyControl(self,_):
        if self.start and self.t < self.repeat*self.T:
            prev = time.time()
            self.t += self.h

            # Low Pass Filter
            self.vz = deriv(self.zPos, self.p_zPos, self.h)
            self.vz = filter_FIR(0.0005, self.hist_vz, self.vz)
            self.vyaw = deriv(self.rotationZ, self.p_rotationZ, self.h)
            self.vyaw = filter_FIR(0.0005, self.hist_vyaw, self.vyaw)

            g = 9.8086
            m = .42

            # Control
            ux = self.mxpp(self.t)+self.kdx*(self.mxp(self.t)-self.vx)\
                 +self.kpx*(self.mx(self.t)-self.xPos)
            uy = self.mypp(self.t)+self.kdy*(self.myp(self.t)-self.vy)\
                 +self.kpy*(self.my(self.t)-self.yPos)
            uz = self.kpz*(self.mz(self.t)-self.zPos)+self.mzp(self.t)

            F = m*sqrt(((uz+g)**2)+(ux**2)+(uy**2))

            # Desired Angles
            if abs(uy/F) < 1 and abs(uy/F) > -1:
                self.phid = -asin(uy/F)  # roll deseado (x)
            else:
                self.phid = -asin(copysign(1.0,uy/F))
                logger.warning('uy/F off asin limits: {}'.format(uy/F))
            self.thetad = atan(ux/(uz+g))  # pitch deseado (y)

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
            self.z_velocity = uz

            # Yaw Velocity control signal
            self.yaw_velocity = -self.kpa*(self.rotationZ-self.psid)

            self.SetCommand(-self.roll, self.pitch, self.yaw_velocity,
                                  self.z_velocity)

            # Derivative of desired angles
            self.phidp = deriv(self.phid,self.p_phid,self.h)
            self.thetadp = deriv(self.thetad,self.p_thetad, self.h)

            # Derivative of drone angles
            self.phip = deriv(self.rotationX, self.p_rotationX, self.h)
            self.thetap = deriv(self.rotationY, self.p_rotationY, self.h)

            super(Adaptative, self).appendData(self)

            k = (time.time()-prev)*1000
            if k > self.control_period:
                logger.warning(
                    'Control law function taking more time '
                    'than control period: {:3} ms'.format(k))
        else:
            if not self.start and self.t == 0:
                logger.info('Taking Off')
                time.sleep(1)
                controller.SendTakeoff()
                self.start = True
            else:
                logger.info('Landing')
                self.paroEmergencia()


# Setup the application
if __name__=='__main__':
    maxTilt = 5.73  # degrees
    sim_num = 1
    ejes = ['Roll', 'Pitch', 'X',
            'Y', 'Z', 'Yaw']

    # Create Folder
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('ardrone_cinvestav')
    res_path = '/src/Results/Simulation/'
    exp_path = 'Adaptative/'
    folder_path = str(sim_num)
    path = pkg_path+res_path+exp_path+folder_path

    while os.path.exists(path):
        sim_num+=1
        folder_path = str(sim_num)
        path = pkg_path + res_path + exp_path + folder_path

    os.mkdir(path)

    # Create and  configure logger
    name = __file__
    logger = logging.getLogger(name.split('/')[-1][:-3].upper())
    logger.setLevel(logging.INFO)
    LOG_FORMAT = "[%(levelname)s] [%(name)s] [%(asctime)s] --> %(message)s"
    formatter = logging.Formatter(LOG_FORMAT)
    file_handler = logging.FileHandler(path+'/info.log')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    logger.info(4 * '#########################')
    # Initialize ROS node
    rospy.init_node('GUI')
    logger.info('Rospy node created')

    # Construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    controller = Adaptative('ardrone_1',2,2,3,ejes)
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
