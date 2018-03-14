#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Author: Misael Antonio Alarcón Herrera
    Description: Class Controller that provides structure for controlling
                 the AR Drone using the BasicDroneController class from the
                 ardrone_tutorials package and a self programmed UI to
                 visualize the position, angles and velocity of these at
                 execution time.
"""

import __main__
import sys
import os
import time
import logging
from math import pi, sqrt, asin, atan, copysign, cos, sin
import numpy as np
import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from drone_controller import BasicDroneController
from graphics_GUI import windowGraphics
from drone_status import DroneStatus
from drone_utils import deriv, degtorad, filter_FIR, avg_error
from gazebo_msgs.msg import LinkStates
from ardrone_autonomy.msg import Navdata
import PySide
from pyqtgraph import QtGui,QtCore
import cPickle as pickle

# Get logger name of the main file
name = __main__.__file__
logger = logging.getLogger(name.split('/')[-1][:-3].upper())

# Define the keyboard map for the controller
class KeyMapping(object):
    Emergency = QtCore.Qt.Key.Key_Space

# Controller class has multiple inheritance from windowsGraphics and
# BasicDroneController
class Controller(windowGraphics, BasicDroneController):
    def __init__(self,model, num_widgets, rows, cols, titulos):
        windowGraphics.__init__(self, num_widgets, rows, cols, titulos)
        BasicDroneController.__init__(self, model)

        self.t = 0.0  # initial time
        self.control_period = 20.  # time in ms the control law is called
        self.h = self.control_period / 1000.0  # delta of time in ms
        self.visualize = False  # If set to True, the UI will show the graphs
                                # in exectuion time. Can cause the control
                                # law to slow down. Recommended just for
                                # troubleshooting.
        self.maxlen = 300.  # Number of points shown in the graph of the UI.
                            # If self.visualize is set to False,
                            # this variable has no effect in the program.
        self.start = False  # variable to point that control has started

        # Center of the trayectory
        self.x0 = 0.
        self.y0 = 0.
        self.z0 = 0.

        # Control Variables of the AR Drone
        # Check for more INFO
        # https://github.com/AutonomyLab/ardrone_autonomy/issues/116
        self.pitch = 0.  # Fraction of maxTilt parameter. From -1 to 1
        self.roll = 0.  # Fraction of maxTilt parameter. From -1 to 1
        self.yaw_velocity = 0.  # velocity of yaw angle (~95 deg/s)
        self.z_velocity = 0.  # linear velocity in z (m/s)

        # AR Drone Navdata
        self.status = -1  # indicates if the drone is flying, hovering, etc
        # Attitude
        self.rotationX = 0.  # current roll (phi) angle (rad)
        self.rotationY = 0.  # current pitch (theta) angle (rad)
        self.rotationZ = 0.  # current yaw (psi) angle (rad)
        self.phid = 0.  # phi angle desired
        self.phip = 0.  # time derivative of phi angle
        self.phidp = 0.  # time derivative of phi angle desired
        self.thetad = 0.  # theta angle desired
        self.thetap = 0.  # time derivative of theta angle
        self.thetadp = 0.  # time derivative of theta angle desired
        # Auxiliar variables to compute the derivative
        self.p_rotationX = 0.  # roll angle in t-1 (rad)
        self.p_rotationY = 0.  # pitch angle in t-1 (rad)
        self.p_rotationZ = 0.  # yaw angle in t-1 (rad)
        self.p_phid = degtorad(0)  # phi angle desired in t-1 (rad)
        self.p_thetad = degtorad(0)  # roll angle desired in t-1 (rad)

        # Linear Position
        self.xPos = 0.  # linear position in X (m)
        self.yPos = 0.  # linear position in Y (m)
        self.zPos = 0.  # linear position in Z (m)
        # Auxiliar variable to compute time derivative of zPos
        self.p_zPos = 0.  # linear position in Z in t-1 (m)

        # Position errors
        self.sum_ex = 0  # sum of the position error in X
        self.sum_ey = 0  # sum of the position error in Y
        self.sum_ez = 0  # sum of the position error in Z

        # Linear velocities
        self.vx, self.vy, self.vz, self.vyaw = 0, 0, 0, 0
        # Since velocities in Z and yaw angle are computed by a discrete
        # time derivative, it is used a FIR filtered of order 20 to clean noise
        # in the resulting signal and for that are necessary a history of the
        # values.
        self.hist_vz = np.zeros((20,))  # history of values of velocity in Z
        self.hist_vyaw = np.zeros((20,))  # history of values of yaw velocity

        # Arrays to store the data to be plotted by Matplotlib
        self.t_array = []
        # Position
        self.roll_pos_array = []
        self.pitch_pos_array = []
        self.x_pos_array = []
        self.y_pos_array = []
        self.z_pos_array = []
        self.yaw_pos_array = []
        # Velocity
        self.roll_vel_array = []
        self.pitch_vel_array = []
        self.x_vel_array = []
        self.y_vel_array = []
        self.z_vel_array = []
        self.yaw_vel_array = []
        # Desired position
        self.rolld_pos_array = []
        self.pitchd_pos_array = []
        self.xd_pos_array = []
        self.yd_pos_array = []
        self.zd_pos_array = []
        self.yawd_pos_array = []
        # Desired velocity
        self.rolld_vel_array = []
        self.pitchd_vel_array = []
        self.xd_vel_array = []
        self.yd_vel_array = []
        self.zd_vel_array = []
        self.yawd_vel_array = []
        # Control Signals
        self.signal_roll = []
        self.signal_pitch = []
        self.signal_yaw = []
        self.signal_z = []

        # Arrays to store the data to be plotted by the UI
        self.dataPosD = [[], [], [], [], [], []]
        self.dataVelD = [[], [], [], [], [], []]
        self.index = 0

        # Timer that calls control law each control period time
        rospy.Timer(rospy.Duration(self.control_period / 1000.0),
                    self.leyControl)

    def mx(self,tp):
        """ Function that return x desired postion at time tp """
        return self.x0 + self.A * np.cos(self.B * pi * tp / self.T)

    def mxp(self,tp):
        """ Function that return x desired velocity at time tp """
        return -(self.A * self.B * pi / self.T) * np.sin(
            self.B * pi * tp / self.T)

    def mxpp(self,tp):
        """ Function that return x desired acceleration at time tp """
        return -(self.A * self.B * self.B * pi * pi / self.T ** 2) * np.cos(
            self.B * pi * tp / self.T)

    def my(self,tp):
        """ Function that return y desired postion at time tp """
        return self.y0 + self.C * np.sin(self.D * pi * tp / self.T)

    def myp(self,tp):
        """ Function that return y desired velocity at time tp """
        return (self.C * self.D * pi / self.T) * np.cos(
            self.D * pi * tp / self.T)

    def mypp(self,tp):
        """ Function that return y desired acceleration at time tp """
        return (self.C * self.D * pi / self.T) * np.cos(
            self.D * pi * tp / self.T)

    def mz(self,tp):
        """ Function that return z desired postion at time tp """
        return self.z0 + self.E * np.cos((self.B * pi * tp / self.T) + pi)

    def mzp(self,tp):
        """ Function that return z desired velocity at time tp """
        return -(self.E * self.B * pi / self.T) * np.sin(
            (self.B * pi * tp / self.T) + pi)

    def mzpp(self,tp):
        """ Function that return z desired acceleration at time tp """
        return -(self.B * self.E * self.E * pi * pi / self.T ** 2) * np.cos(
            (self.B * pi * tp / self.T) + pi)

    def appendData(self, _):
        """ Function that is called at the end of control law to append al
        data to be plotted """
        # Matplotlib data
        # Time
        self.t_array.append(self.t)
        # Position
        self.roll_pos_array.append(self.rotationX)
        self.pitch_pos_array.append(self.rotationY)
        self.x_pos_array.append(self.xPos)
        self.y_pos_array.append(self.yPos)
        self.z_pos_array.append(self.zPos)
        self.yaw_pos_array.append(self.rotationZ)
        # Velocity
        self.roll_vel_array.append(self.phip)
        self.pitch_vel_array.append(self.thetap)
        self.x_vel_array.append(self.vx)
        self.y_vel_array.append(self.vy)
        self.z_vel_array.append(self.vz)
        self.yaw_vel_array.append(self.vyaw)
        # Desired position
        self.rolld_pos_array.append(self.phid)
        self.pitchd_pos_array.append(self.thetad)
        self.xd_pos_array.append(self.mx(self.t))
        self.yd_pos_array.append(self.my(self.t))
        self.zd_pos_array.append(self.mz(self.t))
        self.yawd_pos_array.append(self.psid)
        # Desired velocity
        self.rolld_vel_array.append(self.phidp)
        self.pitchd_vel_array.append(self.thetadp)
        self.xd_vel_array.append(self.mxp(self.t))
        self.yd_vel_array.append(self.myp(self.t))
        self.zd_vel_array.append(self.mzp(self.t))
        self.yawd_vel_array.append(0)
        # Control signals
        self.signal_roll.append(-self.roll)
        self.signal_pitch.append(self.pitch)
        self.signal_yaw.append(self.yaw_velocity)
        self.signal_z.append(self.z_velocity)

        # Update t-1 positions
        self.p_rotationX = self.rotationX
        self.p_rotationY = self.rotationY
        self.p_rotationZ = self.rotationZ
        self.p_zPos = self.zPos

        # UI data
        if self.visualize:
            # this keeps the maxlength of the arrays fixed
            if len(self.x_pos_array) > self.maxlen:
                self.index += 1

            # Position
            self.dataPos.insert(0, self.roll_pos_array[self.index:])
            self.dataPos.insert(1, self.pitch_pos_array[self.index:])
            self.dataPos.insert(2, self.x_pos_array[self.index:])
            self.dataPos.insert(3, self.y_pos_array[self.index:])
            self.dataPos.insert(4, self.z_pos_array[self.index:])
            self.dataPos.insert(5, self.yaw_pos_array[self.index:])
            # Desired position
            self.dataPosD.insert(0, self.rolld_pos_array[self.index:])
            self.dataPosD.insert(1, self.pitchd_pos_array[self.index:])
            self.dataPosD.insert(2, self.xd_pos_array[self.index:])
            self.dataPosD.insert(3, self.yd_pos_array[self.index:])
            self.dataPosD.insert(4, self.zd_pos_array[self.index:])
            self.dataPosD.insert(5, self.yawd_pos_array[self.index:])
            # Velocity
            self.dataVel.insert(0, self.roll_vel_array[self.index:])
            self.dataVel.insert(1, self.pitch_vel_array[self.index:])
            self.dataVel.insert(2, self.x_vel_array[self.index:])
            self.dataVel.insert(3, self.y_vel_array[self.index:])
            self.dataVel.insert(4, self.z_vel_array[self.index:])
            self.dataVel.insert(5, self.yaw_vel_array[self.index:])
            # Desired velocity
            self.dataVelD.insert(0, self.rolld_vel_array[self.index:])
            self.dataVelD.insert(1, self.pitchd_vel_array[self.index:])
            self.dataVelD.insert(2, self.xd_vel_array[self.index:])
            self.dataVelD.insert(3, self.yd_vel_array[self.index:])
            self.dataVelD.insert(4, self.zd_vel_array[self.index:])
            self.dataVelD.insert(5, self.yawd_vel_array[self.index:])

    def ReceivePosition(self, pos_data):
        """ Function receives position info from the Gazebo publisher """
        self.xPos = pos_data.pose[-1].position.x
        self.yPos = pos_data.pose[-1].position.y
        self.zPos = pos_data.pose[-1].position.z

    def ReceiveNavdata(self, navdata):
        """ Function that receives data from the Navdata's AR Drone
        Publisher """
        self.status = navdata.state
        self.vx = navdata.vx / 1000.0 #Meters per second
        self.vy = navdata.vy / 1000.0 #Meters per second
        self.rotationX = degtorad(navdata.rotX)
        self.rotationY = degtorad(navdata.rotY)
        self.rotationZ = degtorad(navdata.rotZ) #+/- 180 degrees

        self.communicationSinceTimer = True

    def keyPressEvent(self, event):
        """ Functions that handles key press event to trigger the Emergency
        Stop function """
        key = event.key()
        if not event.isAutoRepeat():
            if key == KeyMapping.Emergency:
                logger.info('Emergency Stop')
                self.paroEmergencia()

    def paroEmergencia(self):
        """ Emergency stop function that first sets all control variables to 0
        and then lands the AR Drone. When already landed, it computes and logs
        the average error in X, Y and Z"""
        time.sleep(0.1)
        self.start = False
        self.SetCommand(0, 0, 0, 0)
        self.SendLand()

        # Computes the average error
        arrx = np.array([(self.x_pos_array, self.xd_pos_array)])
        arry = np.array([(self.y_pos_array, self.yd_pos_array)])
        arrz = np.array([(self.z_pos_array, self.zd_pos_array)])
        arrt = np.array([(arrx), (arry), (arrz)])
        errx = avg_error(arrx)
        erry = avg_error(arry)
        errz = avg_error(arrz)
        logger.info('Error in X: {}'.format(errx))
        logger.info('Error in Y: {}'.format(erry))
        logger.info('Error in Z: {}'.format(errz))
        self.close()
        time.sleep(1)

    def graficar(self, show=True):
        """ Matplotlib based function that plots and saves:
        - Trajectory in 3D Space
        - Position error over time
        - Attitude error over time
        - Control signals over time

        If show parameter is set to True, at the end of the control law
        execution the plots will be displayed.
        """
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        try:
            # Trajectory in 3D Space
            ax.plot(self.x_pos_array, self.y_pos_array, self.z_pos_array,
                    label="Drone")
            ax.plot(self.xd_pos_array,self.yd_pos_array,
                    self.zd_pos_array, label="Trayectoria Deseada")
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=4, ncol=2,
                       borderaxespad=0.)

            # Position error over time
            fig1, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(12, 10))
            ax0.plot(self.t_array, np.array(self.xd_pos_array)
                     - np.array(self.x_pos_array))
            ax0.set_xlim(0, self.T * self.repeat)
            ax0.set_title('Error de Posicion en X')
            ax0.set_ylabel('Error (m)')
            ax0.grid()

            ax1.plot(self.t_array, np.array(self.yd_pos_array)
                     - np.array(self.y_pos_array))
            ax1.set_xlim(0, self.T * self.repeat)
            ax1.set_title('Error de Posicion en Y')
            ax1.set_ylabel('Error (m)')
            ax1.grid()

            ax2.plot(self.t_array, np.array(self.zd_pos_array)
                     - np.array(self.z_pos_array))
            ax2.set_xlim(0, self.T * self.repeat)
            ax2.set_title('Error de Posicion en Z')
            ax2.set_xlabel('Tiempo (s)')
            ax2.set_ylabel('Error (m)')
            ax2.grid()

            # Attitude error over time
            fig2, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(12, 10))
            ax0.plot(self.t_array, np.array(self.rolld_pos_array)
                     - np.array(self.roll_pos_array))
            ax0.set_xlim(0, self.T * self.repeat)
            ax0.set_title('Error de Orientacion en Roll')
            ax0.set_ylabel('Error (rad)')
            ax0.grid()

            ax1.plot(self.t_array, np.array(self.pitchd_pos_array)
                     - np.array(self.pitch_pos_array))
            ax1.set_xlim(0, self.T * self.repeat)
            ax1.set_title('Error de Orientacion en Pitch')
            ax1.set_ylabel('Error (rad)')
            ax1.grid()

            ax2.plot(self.t_array, np.array(self.yawd_pos_array)
                     - np.array(self.yaw_pos_array))
            ax2.set_xlim(0, self.T * self.repeat)
            ax2.set_title('Error de Orientacion en Yaw')
            ax2.set_xlabel('Tiempo (s)')
            ax2.set_ylabel('Error (rad)')
            ax2.grid()

            # Control signals over time
            fig3, (ax0, ax1, ax2, ax3) = plt.subplots(nrows=4,
                                                      figsize=(15,14))
            ax0.plot(self.t_array, self.signal_roll)
            ax0.set_xlim(0, self.T * self.repeat)
            ax0.set_title('Control en Roll')
            ax0.grid()

            ax1.plot(self.t_array, self.signal_pitch)
            ax1.set_xlim(0, self.T * self.repeat)
            ax1.set_title('Control en Pitch')
            ax1.grid()

            ax2.plot(self.t_array, self.signal_yaw)
            ax2.set_xlim(0, self.T * self.repeat)
            ax2.set_title('Control en Yaw')
            ax2.grid()

            ax3.plot(self.t_array, self.signal_z)
            ax3.set_xlim(0, self.T * self.repeat)
            ax3.set_title('Control en Z')
            ax3.set_xlabel('Tiempo (s)')
            ax3.grid()

            # Save plots
            fig.savefig(self.path+'/trayectoria.png')
            fig1.savefig(self.path+'/posicion.png')
            fig2.savefig(self.path+'/orientacion.png')
            fig3.savefig(self.path+'/control.png')

            # Save data in pickle format
            pack = {'pos': [self.x_pos_array, self.y_pos_array,
                            self.z_pos_array, self.yaw_pos_array],
                    'vel': [self.x_vel_array, self.y_vel_array,
                            self.yaw_vel_array, self.yaw_vel_array],
                    'tiempo': [self.t_array],
                    'control':[self.signal_roll, self.signal_pitch,
                               self.signal_yaw, self.signal_z]}

            with open(self.path+'/data.pickle', 'wb') as f:
                pickle.dump(pack, f)

            logger.info('Data saved')

            if show:
                logger.info('Data showed')
                plt.show()

        except Exception as e:
            logger.error('Could not create graphics: {}'.format(e))


class Follower(Controller):
    def __init__(self, model, num_widgets, rows, cols, titulos):
        super(Follower, self).__init__(model, num_widgets, rows, cols, titulos)
        self.xd = 0
        self.yd = 0

    def G(self, x_g):
        eyaw = x_g[2]
        temp = [[-cos(eyaw), sin(eyaw), 0],
                [-sin(eyaw), -cos(eyaw), 0],
                [0, 0, 1]]
        return np.array(temp)

    def F(self, x_g):
        ex = x_g[0]
        ey = x_g[1]
        eyaw = x_g[2]

        temp = [(ey * self.l_vyaw) + self.l_vx - (self.l_vyaw * self.lam_yd),
                -(ex * self.l_vyaw) + self.l_vy + (self.l_vyaw * self.lam_xd),
                eyaw]
        return np.array(temp)

    def ReceivePosicion(self, pos_data):
        self.xPos = pos_data.pose[-2].position.x
        self.yPos = pos_data.pose[-2].position.y
        self.zPos = pos_data.pose[-2].position.z

    def appendData(self, _):
            """ MATPLOTLIB """
            self.t_array.append(self.t)
            ##### posicion #####
            self.roll_pos_array.append(self.rotationX)
            self.pitch_pos_array.append(self.rotationY)
            self.x_pos_array.append(self.xPos)
            self.y_pos_array.append(self.yPos)
            self.z_pos_array.append(self.zPos)
            self.yaw_pos_array.append(self.rotationZ)
            ##### velocidad ####
            self.roll_vel_array.append(self.phip)
            self.pitch_vel_array.append(self.thetap)
            self.x_vel_array.append(self.vx)
            self.y_vel_array.append(self.vy)
            self.z_vel_array.append(self.vz)
            self.yaw_vel_array.append(self.vyaw)
            ##### posicion deseada #####
            self.rolld_pos_array.append(self.phid)
            self.pitchd_pos_array.append(self.thetad)
            self.xd_pos_array.append(self.xd)
            self.yd_pos_array.append(self.yd)
            self.zd_pos_array.append(self.z0)
            self.yawd_pos_array.append(self.fi)
            ##### velocidad deseada ####
            self.rolld_vel_array.append(self.phidp)
            self.pitchd_vel_array.append(self.thetadp)
            self.xd_vel_array.append(self.vxd)
            self.yd_vel_array.append(self.vyd)
            self.zd_vel_array.append(0)
            self.yawd_vel_array.append(self.vyawd)
            # Control signals
            self.signal_roll.append(-self.roll)
            self.signal_pitch.append(self.pitch)
            self.signal_yaw.append(self.yaw_velocity)
            self.signal_z.append(self.z_velocity)

            self.p_rotationX = self.rotationX
            self.p_rotationY = self.rotationY
            self.p_rotationZ = self.rotationZ
            self.p_zPos = self.zPos

            """ GUI """
            if len(self.x_pos_array) > self.maxlen:
                self.index += 1

            self.dataPos.insert(0, self.roll_pos_array[self.index:])
            self.dataPos.insert(1, self.pitch_pos_array[self.index:])
            self.dataPos.insert(2, self.x_pos_array[self.index:])
            self.dataPos.insert(3, self.y_pos_array[self.index:])
            self.dataPos.insert(4, self.z_pos_array[self.index:])
            self.dataPos.insert(5, self.yaw_pos_array[self.index:])

            self.dataPosD.insert(0, self.rolld_pos_array[self.index:])
            self.dataPosD.insert(1, self.pitchd_pos_array[self.index:])
            self.dataPosD.insert(2, self.xd_pos_array[self.index:])
            self.dataPosD.insert(3, self.yd_pos_array[self.index:])
            self.dataPosD.insert(4, self.zd_pos_array[self.index:])
            self.dataPosD.insert(5, self.yawd_pos_array[self.index:])

            self.dataVel.insert(0, self.roll_vel_array[self.index:])
            self.dataVel.insert(1, self.pitch_vel_array[self.index:])
            self.dataVel.insert(2, self.x_vel_array[self.index:])
            self.dataVel.insert(3, self.y_vel_array[self.index:])
            self.dataVel.insert(4, self.z_vel_array[self.index:])
            self.dataVel.insert(5, self.yaw_vel_array[self.index:])

            self.dataVelD.insert(0, self.rolld_vel_array[self.index:])
            self.dataVelD.insert(1, self.pitchd_vel_array[self.index:])
            self.dataVelD.insert(2, self.xd_vel_array[self.index:])
            self.dataVelD.insert(3, self.yd_vel_array[self.index:])
            self.dataVelD.insert(4, self.zd_vel_array[self.index:])
            self.dataVelD.insert(5, self.yawd_vel_array[self.index:])


    def graficar(self, show=True):
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        try:
            ##### TRAYECTORIA ######
            ax.plot(self.x_pos_array, self.y_pos_array,
                    self.z_pos_array, label="Drone")
            ax.plot(self.xd_pos_array, self.yd_pos_array,
                    self.zd_pos_array, label="Trayectoria Deseada")
            ax.plot(self.l_x_pos_array, self.l_y_pos_array,
                    self.l_z_pos_array,
                    label="Lider")
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.legend(bbox_to_anchor=(0., .952, 1., .152), loc=4,
                       ncol=2, borderaxespad=0.)

            fig1, (ax0, ax1, ax2) = plt.subplots(nrows=3,
                                                 figsize=(12, 10))

            ax0.plot(self.t_array,
                     np.array(self.xd_pos_array) - np.array(
                         self.x_pos_array))
            ax0.set_xlim(0, self.T * self.repeat)
            ax0.set_title('Error de Posicion en X')
            ax0.set_ylabel('Error (m)')
            ax0.grid()

            ax1.plot(self.t_array,
                     np.array(self.yd_pos_array) - np.array(
                         self.y_pos_array))
            # ax1.set_ylim(-.3, .3)
            ax1.set_xlim(0, self.T * self.repeat)
            ax1.set_title('Error de Posicion en Y')
            ax1.set_ylabel('Error (m)')
            ax1.grid()

            ax2.plot(self.t_array,
                     np.array(self.zd_pos_array) - np.array(
                         self.z_pos_array))
            # ax2.set_ylim(-.5, .5)
            ax2.set_xlim(0, self.T * self.repeat)
            ax2.set_title('Error de Posicion en Z')
            ax2.set_xlabel('Tiempo (s)')
            ax2.set_ylabel('Error (m)')
            ax2.grid()

            fig2, (ax0, ax1, ax2) = plt.subplots(nrows=3,
                                                 figsize=(12, 10))
            ax0.plot(self.t_array,
                     np.array(self.pitchd_pos_array) - np.array(
                         self.pitch_pos_array))
            # ax0.set_ylim(-.1, .1)
            ax0.set_xlim(0, self.T * self.repeat)
            ax0.set_title('Error de Orientacion en Pitch')
            ax0.set_ylabel('Error (rad)')
            ax0.grid()

            ax1.plot(self.t_array,
                     np.array(self.rolld_pos_array) - np.array(
                         self.roll_pos_array))
            # ax1.set_ylim(-.2, .2)
            ax1.set_xlim(0, self.T * self.repeat)
            ax1.set_title('Error de Orientacion en Roll')
            ax1.set_ylabel('Error (rad)')
            ax1.grid()

            ax2.plot(self.t_array,
                     np.array(self.yawd_pos_array) - np.array(
                         self.yaw_pos_array))
            # ax2.set_ylim(-.05, .05)
            ax2.set_xlim(0, self.T * self.repeat)
            ax2.set_title('Error de Orientacion en Yaw')
            ax2.set_xlabel('Tiempo (s)')
            ax2.set_ylabel('Error (rad)')
            ax2.grid()

            # Control signals over time
            fig3, (ax0, ax1, ax2, ax3) = plt.subplots(nrows=4,
                                                      figsize=(15, 14))
            ax0.plot(self.t_array, self.signal_roll)
            ax0.set_xlim(0, self.T * self.repeat)
            ax0.set_title('Control en Roll')
            ax0.grid()

            ax1.plot(self.t_array, self.signal_pitch)
            ax1.set_xlim(0, self.T * self.repeat)
            ax1.set_title('Control en Pitch')
            ax1.grid()

            ax2.plot(self.t_array, self.signal_yaw)
            ax2.set_xlim(0, self.T * self.repeat)
            ax2.set_title('Control en Yaw')
            ax2.grid()

            ax3.plot(self.t_array, self.signal_z)
            ax3.set_xlim(0, self.T * self.repeat)
            ax3.set_title('Control en Z')
            ax3.set_xlabel('Tiempo (s)')
            ax3.grid()

            # Save plots
            fig.savefig(self.path + '/trayectoria_follower.png')
            fig1.savefig(self.path + '/posicion_follower.png')
            fig2.savefig(self.path + '/orientacion_follower.png')
            fig3.savefig(self.path + '/control_follower.png')

            # Save data in pickle format
            pack = {'pos': [self.x_pos_array, self.y_pos_array,
                            self.z_pos_array, self.yaw_pos_array],
                    'vel': [self.x_vel_array, self.y_vel_array,
                            self.yaw_vel_array, self.yaw_vel_array],
                    'tiempo': [self.t_array],
                    'control': [self.signal_roll, self.signal_pitch,
                                self.signal_yaw, self.signal_z]}

            with open(self.path + '/data.pickle', 'wb') as f:
                pickle.dump(pack, f)

            logger.info('Data saved')

            if show:
                logger.info('Data showed')
                plt.show()

        except Exception as e:
            logger.error('Could not create graphics: {}'.format(e))
