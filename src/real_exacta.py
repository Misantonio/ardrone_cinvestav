#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Author: Misael Antonio Alarcón Herrera
    Description: Script to control the AR Drone 2.0 using Gazebo Simulator.
                 The control law used in this script is show in
                 D. MERCADO - Control de Formacion de Quadrirotores en un
                 esquema lider-seguidor
"""

import sys
import os
import time
import logging
from math import pi, sqrt, sin, cos, asin, atan, copysign
import rospy
import rospkg
from lib.Controller import Controller
from lib.drone_utils import deriv, degtorad, filter_FIR
from std_msgs.msg import Float32MultiArray
from ardrone_autonomy.msg import Navdata
import PySide
from pyqtgraph import QtGui,QtCore

class Exacta(Controller):
    def __init__(self, model, num_widgets, rows, cols, titulos):
        super(Exacta,self).__init__(model, num_widgets, rows, cols,
                                    titulos)

        self.control_period = 5.
        self.path = path
        self.maxlen = 500

        # Gains
        self.kx1 = .5
        self.kx2 = .5
        self.ky1 = 3
        self.ky2 = 8
        self.kz1 = 5
        self.kz2 = 5
        self.kpa = 2

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
        self.T = 10.

        self.repeat = 1.
        self.psid = degtorad(0)

        # Center of the trajectory
        self.z0 = 0.8

        logger.info('Simulation: {}, Control Period: {} ms, '
                    'Maximum tilt: {} deg'.format(sim_num, self.control_period,
                                                  maxTilt))
        logger.info(
            'Gains: {}, Trajectory: {}'.format((self.kx1, self.kx2,
                                                self.ky1, self.ky2,
                                                self.kz1, self.kz2,
                                                self.kpa),
                                               (self.A, self.B, self.C, self.D,
                                                self.E, self.T, self.repeat,
                                                self.psid)))
        logger.info('Center {}'.format((self.x0, self.y0, self.z0)))

        time.sleep(5)  # Wait for the communication to stablish

        ################ SUSCRIPTORES #####################
        self.subPosition = rospy.Subscriber('/droneLeader', Float32MultiArray,
                                            self.ReceivePosition)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata,
                                           self.ReceiveNavdata)

    def leyControl(self,_):
        if self.start and self.t < self.repeat*self.T:
            prev = time.time()
            g = 9.8086
            m = .42
            ex = self.xPos-self.mx(self.t)
            ey = self.yPos-self.my(self.t)
            ez = self.zPos-self.mz(self.t)
            exp = self.vx-self.mxp(self.t)
            eyp = self.vz-self.myp(self.t)
            ezp = deriv(self.zPos,self.p_zPos,self.h)-self.mzp(self.t)

            rdx = m*(self.mxpp(self.t)-self.kx1*exp-self.kx2*ex)
            rdy = m*(self.mypp(self.t)-self.ky1*eyp-self.ky2*ey)
            rdz = m*(g+self.mzpp(self.t)-self.kz1*ezp-self.kz2*ez)
            Td = sqrt((rdx**2)+(rdy**2)+(rdz**2))

            rdx = rdx/Td
            rdy = rdy/Td
            rdz = rdz/Td

            self.phid = asin(-rdy)
            try:
                self.thetad = asin(rdx/cos(self.phid))
            except ValueError as e:
                logger.error('Value Error: {}'.format(e))

                # Roll & Pitch Control Signals
            self.roll = self.phid/degtorad(maxTilt)
            self.pitch = self.thetad/degtorad(maxTilt)

            # Z Velocity control signal
            self.z_velocity = -self.kz2*(self.zPos-self.mz(self.t))+self.mzp(
                self.t)
            # Yaw velocity control signal
            self.yaw_velocity = -self.kpa * (self.rotationZ - self.psid)

            self.SetCommand(self.roll, self.pitch, self.yaw_velocity,
                                  self.z_velocity)

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

            k = (time.time()-prev)*1000
            if k > self.control_period:
                msg = 'Control law function taking more time than ' \
                      'control period: {:3} ms'.format(k)
                rospy.logwarn(msg)
                logger.info(msg)
        else:
            if not self.start and self.t < self.repeat*self.T:
                rospy.logwarn('taking Off')
                logger.info('Taking Off')
                time.sleep(10)
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
    exp_path = 'Exacta/'
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
    controller = Exacta('ardrone', 2, 2, 3, ejes)
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
