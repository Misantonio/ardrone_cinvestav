#!/usr/bin/env python

import sys
import os
import time
import logging
from math import pi, sqrt, asin, atan, copysign, sin, cos
import numpy as np
import rospy
import rospkg
from sim_PD import PD
from lib.Controller import Controller, Follower
from lib.drone_utils import deriv, degtorad, filter_FIR
from lib.drone_status import DroneStatus
from gazebo_msgs.msg import LinkStates
from ardrone_autonomy.msg import Navdata
import PySide
from pyqtgraph import QtGui, QtCore

class LeaderPD(Controller):
    def __init__(self,model,num_widgets,rows,cols,titulos):
        super(LeaderPD,self).__init__(model,num_widgets,rows,cols,titulos)
        self.control_period = 5.
        self.maxlen = 500.
        self.visualize = True
        self.path = path

        # Gains
        self.kdx = 6
        self.kdy = 4
        self.kpx = 8
        self.kpy = 10
        self.kpz = 7
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
                                                self.E, self.T, self.repeat,
                                                self.psid)))
        logger.info('Center {}'.format((self.x0, self.y0, self.z0)))

        time.sleep(1)

        # Suscribers and timers
        rospy.Subscriber('/gazebo/link_states', LinkStates,
                         self.ReceivePosition)
        rospy.Subscriber('/' + model + '/navdata', Navdata,
                         self.ReceiveNavdata)

    def rx(self,x,xp,tp):
        return self.mxpp(tp)-self.kdx*(xp-self.mxp(tp))-self.kpx*(x-self.mx(tp))

    def ry(self,y,yp,tp):
        return self.mypp(tp)-self.kdy*(yp-self.myp(tp))-self.kpy*(y-self.my(tp))

    def leyControl(self,_):
        if self.start and self.t < self.repeat*self.T:
            prev = time.time()
            self.t += self.h
            g = 9.8086

            # Control
            a = self.rx(self.xPos, self.vx, self.t)
            b = self.ry(self.yPos, self.vy, self.t)

            # Desired Angles
            self.phid = (1/g)*(a*sin(self.psid)-b*cos(self.psid))
            self.thetad = (1/g)*(a*cos(self.psid)+b*sin(self.psid))

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
            self.z_velocity = -self.kpz * (self.zPos - self.mz(self.t)) \
                              + self.mzp(self.t)

            # Yaw velocity control signal
            self.yaw_velocity = -self.kpa * (self.rotationZ - self.psid)

            self.SetCommand(-self.roll, self.pitch, self.yaw_velocity,
                         self.z_velocity)

            # Derivative of desired angles
            self.phidp = deriv(self.phid,self.p_phid,self.h)
            self.thetadp = deriv(self.thetad,self.p_thetad, self.h)

            # Derivative of drone angles
            self.phip = deriv(self.rotationX, self.p_rotationX, self.h)
            self.thetap = deriv(self.rotationY, self.p_rotationY, self.h)

            # Low Pass Filter
            self.vz = deriv(self.zPos, self.p_zPos, self.h)
            self.vz = filter_FIR(0.0005, self.hist_vz, self.vz)
            self.vyaw = deriv(self.rotationZ, self.p_rotationZ, self.h)
            self.vyaw = filter_FIR(0.0005, self.hist_vyaw, self.vyaw)

            super(LeaderPD, self).appendData(self)

            k = (time.time()-prev)*1000
            if k > self.control_period:
                logger.warning('Control law function taking more time than '
                               'control period: {:3} ms'.format(k))

        else:
            if not self.start and self.t < self.repeat*self.T:
                logger.info('Taking Off')
                time.sleep(1)
                self.SendTakeoff()
                self.start = True
            else:
                logger.info('Landing')
                self.paroEmergencia()

    def ReceivePosition(self, pos_data):
        """ Function receives position info from the Gazebo publisher """
        self.xPos = pos_data.pose[-1].position.x
        self.yPos = pos_data.pose[-1].position.y
        self.zPos = pos_data.pose[-1].position.z



class FollowerAdaptative(Follower):
    def __init__(self, model, num_widgets, rows, cols, titulos):
        super(FollowerAdaptative, self).__init__(model, num_widgets, rows,
                                                 cols, titulos)

        self.control_period = 5.
        self.path = path
        self.visualize = True

        self.kf = 0.2
        self.kd1 = 3
        self.kd2 = 5
        self.kp1 = 3
        self.kp2 = 4
        self.kpa = 3
        self.kpz = 2.
        self.z0 = 1
        self.fi = 0
        self.lam = 1.5
        self.lam_xd = self.lam * cos(self.fi)
        self.lam_yd = self.lam * sin(self.fi)
        self.yawF = 0.
        self.x0 = 2.

        # Leader data
        self.l_vx = leader.vx
        self.l_vy = leader.vy
        self.l_vyaw = leader.vyaw

        self.l_x_pos_array = leader.x_pos_array
        self.l_y_pos_array = leader.y_pos_array
        self.l_z_pos_array = leader.z_pos_array

        self.T = leader.T
        self.repeat = leader.repeat

        time.sleep(1)
        # Suscribers and timers
        rospy.Subscriber('/gazebo/link_states', LinkStates,
                         self.ReceivePosition)
        rospy.Subscriber('/' + model + '/navdata', Navdata,
                         self.ReceiveNavdata)


    def leyControl(self,_):
        if self.status == DroneStatus.Flying and leader.status == \
                DroneStatus.Flying:
            g = 9.8086
            prev = time.time()
            self.t += self.h
            xL = leader.xPos
            xLp = leader.vx
            yL = leader.yPos
            yawL = leader.rotationZ
            yawLp = leader.vyaw

            # 1. Calcular lam_x, lam_y
            lam_x = -(xL - self.xPos) * cos(yawL) - (yL - self.yPos) * sin(
                yawL)
            lam_y = (xL - self.xPos) * sin(yawL) - (yL - self.yPos) * cos(yawL)

            # 2. Calcular ex,ey,eyaw
            ex = self.lam_xd - lam_x
            ey = self.lam_yd - lam_y
            eyaw = self.yawF - yawL

            # 3. Contruir XG
            XG = np.array([ex, ey, eyaw])

            # 4. Calcular vF_d
            vF_d = np.dot(np.linalg.inv(self.G(XG)),
                          (-self.F(XG) - self.kf * XG))
            self.vxd = vF_d[0]
            self.vyd = vF_d[1]
            self.vyawd = vF_d[2]

            self.xd = xL + self.lam_xd * cos(yawL) - self.lam_yd * sin(yawL)
            self.yd = yL + self.lam_xd * sin(yawL) + self.lam_yd * cos(yawL)

            u1 = leader.mxpp(self.t) + self.kd1 * (
                        self.vxd - self.vx) + self.kp2 * (
                         self.xd - self.xPos)
            u2 = leader.mypp(self.t) + self.kd1 * (
                        self.vyd - self.vy) + self.kp2 * (
                         self.yd - self.yPos)
            u3 = leader.mzpp(self.t) + self.kd1 * (0 - self.vz) + self.kpz * (
                        self.z0 - self.zPos)

            ###### ANGULOS DESEADOS #######
            arg = (u1 * sin(self.fi) - u2 * cos(self.fi)) / (
                sqrt(u1 ** 2 + u2 ** 2 + (u3 + g) ** 2))
            try:
                self.phid = -asin(arg)  # roll deseado (x)
            except ValueError as e:
                self.phid = -asin(copysign(1.0, arg))
                logger.warning('{}: {}'.format(e, arg))
            self.thetad = atan(
                u1 * cos(self.fi) + u2 * sin(self.fi) / (u3 + g))  # pitch

            ########## ROLL & PITCH ########
            self.roll = self.phid / degtorad(maxTilt)
            self.pitch = self.thetad / degtorad(maxTilt)
            ########## POS Z ###############
            self.z_velocity = u3
            ########## POS YAW #############
            self.yaw_velocity = -self.kpa * (self.rotationZ - self.fi)

            self.SetCommand(self.roll, self.pitch,
                                       self.yaw_velocity, self.z_velocity)

            # Derivative of desired angles
            self.phidp = deriv(self.phid, self.p_phid, self.h)
            self.thetadp = deriv(self.thetad, self.p_thetad, self.h)

            # Derivative of drone angles
            self.phip = deriv(self.rotationX, self.p_rotationX, self.h)
            self.thetap = deriv(self.rotationY, self.p_rotationY, self.h)

            self.vz = deriv(self.zPos, self.p_zPos, self.h)
            self.vz = filter_FIR(0.0005, self.hist_vz, self.vz)
            self.vyaw = deriv(self.rotationZ, self.p_rotationZ, self.h)
            self.vyaw = filter_FIR(0.0005, self.hist_vyaw, self.vyaw)

            super(FollowerAdaptative, self).appendData(self)
            # print self.status

            k = (time.time() - prev) * 1000
            if k > self.control_period:
                logger.warning(
                    'Follower control law function taking more time than '
                    'control period: {:3} ms'.format(k))

        elif leader.status == DroneStatus.Flying \
                and self.status == DroneStatus.Landed:
            self.SendTakeoff()
        elif leader.status == DroneStatus.Landed:
            self.paroEmergencia()

    def ReceivePosition(self, pos_data):
        """ Function receives position info from the Gazebo publisher """
        self.xPos = pos_data.pose[-2].position.x
        self.yPos = pos_data.pose[-2].position.y
        self.zPos = pos_data.pose[-2].position.z

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
    exp_path = 'Follower_PD/'
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
    leader = LeaderPD('ardrone_1', 2, 2, 3, ejes)
    follower = FollowerAdaptative('ardrone_2', 2, 2, 3, ejes)
    logger.info('Qt GUI created')
    # leader.show()
    # follower.show()

    # executes the QT application
    status = app.instance().exec_()
    logger.info('Qt GUI executed')

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    logger.info('Rospy node closed')
    leader.graficar(show=False)
    follower.graficar(show=True)
    sys.exit(status)
