#!/usr/bin/python

# -*- coding: utf-8 -*-

## Created by Cheolhui Min @ Korea Univ.
## KRRI logistics demo using Sawyer robot

from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import *

import time
import argparse
import rospy
import os
import shutil
import socket
import threading
import os
import operator
import cv2
from bisect import bisect
from copy import copy
from os import path
import actionlib
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
from object_detection_YOLOv2.msg import *
from std_msgs.msg import *
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
import numpy as np
import pdb
import transforms3d
from transforms3d import euler
from transforms3d import quaternions
import math
from math import degrees as deg
from math import radians as rad
import PyKDL
import intera_interface
from intera_interface import CHECK_VERSION
from tf_conversions import posemath
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from intera_core_msgs.msg import (
    DigitalIOState,
    DigitalOutputCommand,
    IODeviceStatus
)
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import (  # Forward Kinematics & Inverse Kinematics
    SolvePositionFK,
    SolvePositionFKRequest,
    SolvePositionIK,
    SolvePositionIKRequest,

)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.msg import (

    DigitalIOState,
    DigitalOutputCommand,
    IODeviceStatus
)


from intera_motion_msgs.msg import (
    Trajectory,
    TrajectoryOptions,
    Waypoint
)
from intera_motion_interface import (

    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from operator import itemgetter
from intera_interface import Limb
from intera_interface import Cuff

from robotiq_85_msgs.msg import GripperCmd, GripperStat

from new_sawyer_interface.msg import Cmd


# UR5
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

import math
from math import pi


try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

import sys
import urx


reload(sys)
sys.setdefaultencoding('utf8')


base_dir = os.path.dirname(os.path.realpath(__file__))



class trajectorySender(QThread):

    def __init__(self, parent = None):

        QThread.__init__(self)
        self.limb = Limb()
        self.trajectory_option = TrajectoryOptions(interpolation_type='CARTESIAN')
        self.trajectory = MotionTrajectory(trajectory_options=self.trajectory_option, limb=self.limb)
        self.temp_trajectory = MotionTrajectory(trajectory_options=self.trajectory_option, limb=self.limb)

        self.wpt_opts = MotionWaypointOptions(max_linear_speed=0.3,
                                              max_linear_accel=0.3,
                                              max_rotational_speed=3.0,
                                              max_rotational_accel=3.0,
                                              max_joint_speed_ratio=0.7,
                                              corner_distance=0.3)

        self.waypoint = MotionWaypoint(options=self.wpt_opts.to_msg(), limb=self.limb)
        self.waypoint_initial = MotionWaypoint(options=self.wpt_opts.to_msg(), limb=self.limb)
        self.temp_waypoint = MotionWaypoint(options=self.wpt_opts.to_msg(), limb=self.limb)


    def sendTrajectory(self):
        log = self.trajectory.send_trajectory(timeout=10)
        print (log)
        if self.limb.has_collided():
            rospy.logerr('collision detected!!!')
            rospy.sleep(.5)
        self.clearTrajectory()
        self.waypoint_count = 0

    def clearTrajectory(self):
        self.trajectory.clear_waypoints()
        self.waypont_count = 0
        # print('all waypoints have been cleared')

class urMotion(QThread):

    def __init__(self, parent = None):
        # self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        # print ("Waiting for server...")
        # self.client.wait_for_server()
        # print ("Connected to server")
        #
        # self.initpointDeg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.initpointRad = [ rad*(pi/180) for rad in self.initpointDeg]
        QThread.__init__(self)
        self.init_wpr = (0.369, -0.309, 0.385, 1.62, -0.309, 0.537)
        self.wp1 = (0.552, -0.104, 0.179, 2.127, -2.572, 0.339)
        self.wp2 = (0.380, 0.255, 0.234, 3.809, -0.420, -0.0591)
        self.wp3 = (0.320, 0.412, 0.171, 4.032, -0.143, -0.077)
        self.wp4 = (0.320, 0.414, 0.0180, 4.24, -0.14, 0.286)

        self.wp5 = (0.320, 0.1709, 0.2089, 4.24, -0.14, 0.286)
        self.wp6 = (0.320, 0.414, 0.0180, 4.24, -0.14, 0.286)
        self.ur5 = urx.Robot("192.168.1.12")
        self.ur5.set_tcp((0.0, 0.0, 0.242, 0.0, 0.0, 0.0))
        rospy.sleep(0.2)
        self.lin_accel = 0.3
        self.lin_vel = 0.3
        self.ur5.movel(self.wp1,self.lin_accel,self.lin_vel)







class Ui_MainWindow(QMainWindow):

    def __init__(self, parent = None):
        super(Ui_MainWindow,self).__init__()
        print("Initializing node...")
        rospy.init_node("sawyerNode")
        self.cuff = Cuff()
        self.bridge = CvBridge()
        self.th = trajectorySender(parent=self)
        # th = trajectorySender(parent=self)
        self.th.start()
        # th.start()
        self.th2 = urMotion(parent=self)
        self.th2.start()
        self.head_display = intera_interface.HeadDisplay()
        self.head_display.display_image(base_dir+"/head.png")


        # inputs
        # self.gripper = intera_interface.Gripper("right")
        # self.gripper.calibrate()
        # self.gripper.set_velocity(self.gripper.MAX_VELOCITY)
        # self.gripper.set_holding_force(self.gripper.MAX_FORCE)
        # self.gripper.open()

        self.initial_position = {'right_j0': rad(-2.553), 'right_j1': rad(-68.290), 'right_j2': rad(4.096),
                                 'right_j3': rad(127.056), 'right_j4': rad(-5.501), 'right_j5': rad(25.005),
                                 'right_j6': rad(81.015)}
        self.boxAobjectDictList = {'label': [], 'gloX': [], 'gloY': [], 'gloZ': [], 'item': []}
        self.boxBobjectDictList = {'label': [], 'gloX': [], 'gloY': [], 'gloZ': [], 'item': []}
        self.tempTargetApos = [0.0, 0.0, 0.0]
        self.tempTargetAposRobCoord = [0.0, 0.0, 200.0]
        self.tempTargetAlabel = ''
        self.tempTargetBpos = [0.0, 0.0, 0.0]
        self.tempTargetBposRobCoord = [0.0, 0.0, 200.0]
        self.tempTargetBlabel = ''

        self.tempTargetTemppos = [0.0, 0.0, 0.0]
        self.tempTargetTempRobCoord = [0.0, 0.0, 300.0]

        self.tempTargetTemppos2 = [0.0, 0.0, 0.0]
        self.tempTargetTempRobCoord2 = [0.0, 0.0, 300.0]

        self.palletizingStartBoxPoint = [-176.25, 106.66, 0.0]
        self.palletizingStartRobCoordA = [0.0, 0.0, 300.0]
        self.palletizingStartRobCoordB = [0.0, 0.0, 300.0]

        # self.offsetXYZboxA = [23.3,-585.0,163.0]
        self.offsetXYZboxA = [7.418,-622.5, 10.0]
        # self.offsetXYZboxB = [614.4,13.3,0.0]
        self.offsetXYZboxB = [621.60 ,-1.143 ,-5.0]
        self.approach_height = 0.25
        self.palletizing_height = 0.0
        self.grippingCheck_height = 0.25
        self.retrieve_height = 0.25
        self.empty_placing_height = 0.12
        # self.gripperlength = 0.178
        self.gripperlength = 0.00
        self.waypoint_count = 0
        self.initPos = {'x': 341.305,'y': 172.857,'z': 206.613,'Roll': -177.0,'Pitch': 4.0,'Yaw': -165.0}
        self.initCartesian = [0.66, -0.012, 0.488, -179.0, 0.0, 100.0]


        self.endpoint_state = self.th.limb.tip_state('right_hand')
        self.targetPose = self.endpoint_state.pose
        self.poseStamped = PoseStamped()

        self.robposeforCalibrationA = {'first': {'x': 0.0, 'y':0.0}, 'second': {'x': 0.0, 'y':0.0}, 'third':{'x': 0.0, 'y':0.0},'fourth':{'x': 0.0, 'y':0.0}} # 4 (X, Y) coordinate sets
        self.robposeforCalibrationB = {'first': {'x': 0.0, 'y':0.0}, 'second': {'x': 0.0, 'y':0.0}, 'third':{'x': 0.0, 'y':0.0},'fourth':{'x': 0.0, 'y':0.0}} # 4 (X, Y) coordsets & one Z

        rospy.Subscriber("/robot/digital_io/right_valve_1a/state", DigitalIOState, self.valve1a_Callback)
        rospy.Subscriber("/robot/digital_io/right_valve_1b/state", DigitalIOState, self.valve1b_Callback)
        rospy.Subscriber("/robot/digital_io/right_valve_2a/state", DigitalIOState, self.valve2a_Callback)
        rospy.Subscriber("/robot/digital_io/right_valve_2b/state", DigitalIOState, self.valve2b_Callback)
        rospy.Subscriber("/io/end_effector/right_vacuum_gripper/state", IODeviceStatus, self.vacuumStatus_Callback)
        rospy.Subscriber("/io/end_effector/right_vacuum_gripper/state", IODeviceStatus, self.vacuumStatus_Callback)
        rospy.Subscriber("/io/end_effector/right_vacuum_gripper/state", IODeviceStatus, self.vacuumStatus_Callback)
        rospy.Subscriber("/detection1", Detection, self.objdct1Callback)
        rospy.Subscriber("/detection2", Detection, self.objdct2Callback)
        rospy.Subscriber("/detection_array1", DetectionArray, self.objdctArr1Callback)
        rospy.Subscriber("/detection_array2", DetectionArray, self.objdctArr2Callback)
        rospy.Subscriber("/detection_full1", DetectionFull, self.objdctFull1Callback)
        rospy.Subscriber("/detection_full2", DetectionFull, self.objdctFull2Callback)
        rospy.Subscriber("/gripper/stat", GripperStat, self._update_gripper_stat, queue_size=10)

        # self.publishVacuumCommand = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size =30 )
        self.publishSchunk = rospy.Publisher('/wsg_32_driver/goal_position', Cmd, queue_size =30 )
        self.robotiqPub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

        self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        self.timer1 = QtCore.QTimer()
        self.timer2 = QtCore.QTimer()
        self.timer1.timeout.connect(self.calibrateboxAPos)
        self.timer2.timeout.connect(self.calibrateboxBPos)
        self.timer1Flag = False
        self.timer2Flag = False
        self.init_endpoint_dict = self.th.limb.endpoint_effort()
        self.pallet_firstrow_offset = 0.006
        self.pallet_thirdrow_offset = 0.006
        self.endpoint_ft_dict = self.th.limb.endpoint_effort()
        self.curFzValue = self.endpoint_ft_dict['force'][2]
        self.flagAtoBorBtoA = False # If False, AtoB, otherwise, BtoA

        self.curDetectedItemsbyClassAReal = {'dole_mango':[], 'cantata_latte':[], 'chokoemong':[], 'starbucks_skunnylatte':[], 'danone_greek': []}
        self.curDetectedItemsbyClassA = {'dole_mango':[], 'cantata_latte':[], 'chokoemong':[], 'starbucks_skunnylatte':[], 'danone_greek': []}
        self.curDetectedItemsbyClassBReal = {'dole_mango':[], 'cantata_latte':[], 'chokoemong':[], 'starbucks_skunnylatte':[], 'danone_greek': []}
        self.curDetectedItemsbyClassB = {'dole_mango':[], 'cantata_latte':[], 'chokoemong':[], 'starbucks_skunnylatte':[], 'danone_greek': []}
        self.totalObjectListInBoxAReal = []
        self.totalObjectListInBoxA = []
        self.totalObjectListInBoxBReal = []
        self.totalObjectListInBoxB = []

        self.userSpecifiedItemClassA = ''
        self.userSpecifiedItemClassB = ''
        self.palletizingSuccessCount = 0
        self.palletizingStep = 0
        self.palletizeAtBstartPos = []
        self.poseACount = 0
        self.poseBCount = 0
        self.okToPalletize = False
        self.objectWeightThreshold = 1.2  # in N
        self.isROSPyrunning = True
        self.endpoint_ft_dict = self.th.limb.endpoint_effort()
        self.nopayload_Fz = 0.0
        self.waitoverObjectTime1 = 0.1
        self.waitoverObjectTime2 = 0.4

        self.mode_string = ""
        self.closed_pos = 0.0 # mm (closed)
        self.open_pos = 50.0 # mm (open)
        self.speed = 50.0 # mm/s
        # self.vacuumCommand1 =

        self._gripper_cmd = GripperCmd()
        self._gripper_stat = GripperStat()

        self.sawyer_wp1 = [673, -361 , 268, -171, 3.09, 137.0]
        self.sawyer_wp2 = [664.58, -367.647, 318.202, -176.941, 0.881, 141.461]
        self.sawyer_wp3 = [648, -390, 142, -179, 4.02, 166.8]
        self.sawyer_wp4 = [474, -333.4, 393, -174, 17.3, 167.7]
        self.sawyer_wp5 = [254.5, -402.6, 292, -177, -22.3, 152.9]
        self.sawyer_wp6 = [97.85, -406.67, 144.051, -179.591, -0.762, 97.875]
        self.sawyer_wp7 = [97.85, -406.67, 170.051, -179.591, -0.762, 97.875]
        self.sawyer_wp8 = [140.85, -406.67, 240.051, -179.591, -0.762, 97.875]
        self.sawyer_wp9 = [300.85, -395.67, 302.051, -179.591, -0.762, 97.875]

        self.sawyerTrajSignal = SIGNAL("Sawyer")
        self.UR5TrajSignal = SIGNAL("UR5")

        # th.conn/ect(self, self.sawyerTrajSignal, th.sendTrajectory)
        # self.connect(self.th2, self.UR5TrajSignal, self.th2.sendTrajectory)
        # self.connect(self.th2, self.UR5TrajSignal, self.th2.sendTrajectory)

    def sendSawyerTrajSignal(self):
        self.emit(self.sawyerTrajSignal)

    def sendUR5TrajSignal(self):
        self.emit(self.UR5TrajSignal)

    def _update_gripper_stat(self, stat):
        self._gripper_stat = stat


    def robotiq_close(self):

        print ('robotiq closes')
        self._gripper_cmd.position = 0.0
        self._gripper_cmd.speed = 0.02
        self._gripper_cmd.force = 100.0
        self.robotiqPub.publish(self._gripper_cmd)

    def robotiq_open(self, ratio = 4.0):


        print ('robotiq opens %0.2f' % (0.85 * ratio /4.0))
        self._gripper_cmd.position = 0.085 * ratio / 4.0
        self._gripper_cmd.speed = 0.02
        self._gripper_cmd.force = 100.0

        self.robotiqPub.publish(self._gripper_cmd)


    def gripper_on(self):
        self.gripperCommand = Cmd()
        self.gripperCommand.pos = self.closed_pos
        self.gripperCommand.speed = self.speed
        self.publishSchunk.publish(self.gripperCommand)


    def gripper_off(self):
        self.gripperCommand = Cmd()
        self.gripperCommand.pos = self.open_pos
        self.gripperCommand.speed = self.speed
        self.publishSchunk.publish(self.gripperCommand)




    def valve1a_Callback(self, data):
        self.valave1a_state = data.state

    def valve1b_Callback(self, data):

        self.valave1b_state = data.state

    def valve2a_Callback(self, data):

        self.valave2a_state = data.state

    def valve2b_Callback(self, data):

        self.valave2b_state = data.state

    def vacuumStatus_Callback(self, data):

        self.vacuumStatus = data.signals

    def objdct1Callback(self, data):
        self.detection1 = data

    def objdct2Callback(self, data):
        self.detection1 = data

    def objdctArr1Callback(self, data):
        self.detectionArr1 = data

    def objdctArr2Callback(self, data):
        self.detectionArr2 = data

    def objdctFull1Callback(self, data):
        self.detectionFull1 = data
        self.detectionmsg1 = self.detectionFull1.detections.data # Detection array corresponding to the image view


    def objdctFull2Callback(self, data):
        self.detectionFull2 = data
        self.detectionmsg2 = self.detectionFull2.detections.data # Detection array corresponding to the image view


    def lowerX(self, value):

        return value.glo_x

    def higherY(self, value):

        return value.glo_y

    def higherPred(self, value):

        return value.confidence

    def monitorCurrentObjects(self): # sort by left most
        self.totalObjectListInBoxA = []
        self.totalObjectListInBoxB = []
        self.curDetectedItemsbyClassA['dole_mango'] = []
        self.curDetectedItemsbyClassA['cantata_latte'] = []
        self.curDetectedItemsbyClassA['chokoemong'] = []
        self.curDetectedItemsbyClassA['starbucks_skunnylatte'] = []
        self.curDetectedItemsbyClassA['danone_greek'] = []
        self.curDetectedItemsbyClassB['dole_mango'] = []
        self.curDetectedItemsbyClassB['cantata_latte'] = []
        self.curDetectedItemsbyClassB['chokoemong'] = []
        self.curDetectedItemsbyClassB['starbucks_skunnylatte'] = []
        self.curDetectedItemsbyClassB['danone_greek'] = []
        for item in self.boxAobjectDictList['item']:
            self.totalObjectListInBoxA.append(item)
            self.totalObjectListInBoxA = sorted(self.totalObjectListInBoxA, key=self.higherPred, reverse = True)
            if item.label == 'dole_mango':
                self.curDetectedItemsbyClassA['dole_mango'].append(item)
                self.curDetectedItemsbyClassA['dole_mango'] = sorted(self.curDetectedItemsbyClassA['dole_mango'], key=self.higherPred, reverse = True)
            elif item.label == 'cantata_latte':
                self.curDetectedItemsbyClassA['cantata_latte'].append(item)
                self.curDetectedItemsbyClassA['cantata_latte'] = sorted(self.curDetectedItemsbyClassA['cantata_latte'], key=self.higherPred, reverse=True)
            elif item.label == 'chokoemong':
                self.curDetectedItemsbyClassA['chokoemong'].append(item)
                self.curDetectedItemsbyClassA['chokoemong'] = sorted(self.curDetectedItemsbyClassA['chokoemong'], key=self.higherPred , reverse=True)
            elif item.label == 'starbucks_skunnylatte':
                self.curDetectedItemsbyClassA['starbucks_skunnylatte'].append(item)
                self.curDetectedItemsbyClassA['starbucks_skunnylatte'] = sorted(self.curDetectedItemsbyClassA['starbucks_skunnylatte'],key=self.higherPred, reverse=True)
            elif item.label == 'danone_greek':
                self.curDetectedItemsbyClassA['danone_greek'].append(item)
                self.curDetectedItemsbyClassA['danone_greek'] = sorted(self.curDetectedItemsbyClassA['danone_greek'], key=self.higherPred , reverse=True)
        self.totalObjectListInBoxAReal = self.totalObjectListInBoxA
        self.curDetectedItemsbyClassAReal = self.curDetectedItemsbyClassA
        self.cantataBoxA.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassA['cantata_latte'])))
        self.chocoBoxA.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassA['chokoemong'])))
        self.doleBoxA.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassA['dole_mango'])))
        self.starBoxA.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassA['starbucks_skunnylatte'])))
        self.danonBoxA.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassA['danone_greek'])))



        for item in self.boxBobjectDictList['item']:
            self.totalObjectListInBoxB.append(item)
            self.totalObjectListInBoxB = sorted(self.totalObjectListInBoxB, key=self.higherPred, reverse=True)
            if item.label == 'dole_mango':
                self.curDetectedItemsbyClassB['dole_mango'].append(item)
                self.curDetectedItemsbyClassB['dole_mango'] = sorted(self.curDetectedItemsbyClassB['dole_mango'], key=self.higherPred, reverse=True)
            elif item.label == 'cantata_latte':
                self.curDetectedItemsbyClassB['cantata_latte'].append(item)
                self.curDetectedItemsbyClassB['cantata_latte'] = sorted(self.curDetectedItemsbyClassB['cantata_latte'], key=self.higherPred, reverse=True)
            elif item.label == 'chokoemong':
                self.curDetectedItemsbyClassB['chokoemong'].append(item)
                self.curDetectedItemsbyClassB['chokoemong'] = sorted(self.curDetectedItemsbyClassB['chokoemong'], key=self.higherPred, reverse=True)
            elif item.label == 'starbucks_skunnylatte':
                self.curDetectedItemsbyClassB['starbucks_skunnylatte'].append(item)
                self.curDetectedItemsbyClassB['starbucks_skunnylatte'] = sorted(self.curDetectedItemsbyClassB['starbucks_skunnylatte'],key=self.higherPred, reverse=True)
            elif item.label == 'danone_greek':
                self.curDetectedItemsbyClassB['danone_greek'].append(item)
                self.curDetectedItemsbyClassB['danone_greek'] = sorted(self.curDetectedItemsbyClassB['danone_greek'], key=self.higherPred, reverse=True)
        self.totalObjectListInBoxBReal = self.totalObjectListInBoxB
        self.curDetectedItemsbyClassBReal = self.curDetectedItemsbyClassB
        self.cantataBoxB.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['cantata_latte'])))
        self.chocoBoxB.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['chokoemong'])))
        self.doleBoxB.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['dole_mango'])))
        self.starBoxB.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['starbucks_skunnylatte'])))
        self.danonBoxB.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['danone_greek'])))

        self.cantataTotal.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['cantata_latte'])+
                                                        len(self.curDetectedItemsbyClassA['cantata_latte'])))
        self.chocoTotal.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['chokoemong'])+
                                                        len(self.curDetectedItemsbyClassA['chokoemong'])))
        self.doleTotal.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['dole_mango'])+
                                                        len(self.curDetectedItemsbyClassA['dole_mango'])))
        self.starTotal.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['starbucks_skunnylatte'])+
                                                        len(self.curDetectedItemsbyClassA['starbucks_skunnylatte'])))
        self.danonTotal.setText(QtCore.QString.number(len(self.curDetectedItemsbyClassB['danone_greek'])+
                                                        len(self.curDetectedItemsbyClassA['danone_greek'])))




    def get_nopayload_Fz(self):
        QApplication.processEvents()
        total_Fz = 0.0
        for i in range(0,10):
            QApplication.processEvents()
            total_Fz += self.endpoint_ft_dict['force'][2]
            rospy.sleep(0.02)
        nonPL_Fz_avg = total_Fz / 10.0

        return nonPL_Fz_avg

    def grippingRetrial(self, numTrial=0, whichobject=''):
        base_retrial_x_offset = 10 / 235000.0  # w.r.t. robot's baseframe
        base_retrial_y_offset = 10 / 160000.0
        self.gripper_off()
        rospy.sleep(0.5)
        # self.detectObjectsinBoxes()
        if self.flagAtoBorBtoA == False:  # AtoB
            if whichobject == 'total':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.totalObjectListInBoxAReal):
                        retrial_x_offset = base_retrial_x_offset * self.totalObjectListInBoxAReal[0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.totalObjectListInBoxAReal[0].glo_y
                        self.tempTargetApos[0] = self.totalObjectListInBoxAReal[0].glo_x
                        self.tempTargetApos[1] = self.totalObjectListInBoxAReal[0].glo_y
                        self.tempTargetApos[2] = self.totalObjectListInBoxAReal[0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        rospy.sleep(0.5)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0] + retrial_x_offset,
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.totalObjectListInBoxAReal):
                        retrial_x_offset = base_retrial_x_offset * self.totalObjectListInBoxAReal[0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.totalObjectListInBoxAReal[0].glo_y
                        self.tempTargetApos[0] = self.totalObjectListInBoxAReal[0].glo_x
                        self.tempTargetApos[1] = self.totalObjectListInBoxAReal[0].glo_y
                        self.tempTargetApos[2] = self.totalObjectListInBoxAReal[0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'cantata_latte':
                if numTrial == 1:  # 1st retril
                    self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['cantata_latte']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['cantata_latte'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['cantata_latte'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['cantata_latte'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0] + retrial_x_offset,
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['cantata_latte']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['cantata_latte'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['cantata_latte'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['cantata_latte'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'starbucks_skunnylatte':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['starbucks_skunnylatte']):
                        retrial_x_offset = base_retrial_x_offset * \
                                           self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_x
                        retrial_y_offset = base_retrial_y_offset * \
                                           self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0] + retrial_x_offset,
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['starbucks_skunnylatte']):
                        retrial_x_offset = base_retrial_x_offset * \
                                           self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_x
                        retrial_y_offset = base_retrial_y_offset * \
                                           self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'chokoemong':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['chokoemong']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['chokoemong'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['chokoemong'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['chokoemong'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0] + retrial_x_offset,
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['chokoemong']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['chokoemong'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['chokoemong'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['chokoemong'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'dole_mango':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['dole_mango']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['dole_mango'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['dole_mango'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['dole_mango'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0] + retrial_x_offset,
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['dole_mango']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['dole_mango'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['dole_mango'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['dole_mango'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'danone_greek':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['danone_greek']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['danone_greek'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['danone_greek'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['danone_greek'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0] + retrial_x_offset,
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassA['danone_greek']):
                        retrial_x_offset = base_retrial_x_offset * self.curDetectedItemsbyClassA['danone_greek'][
                            0].glo_x
                        retrial_y_offset = base_retrial_y_offset * self.curDetectedItemsbyClassA['danone_greek'][
                            0].glo_y
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['danone_greek'][
                            0].glo_z  # re-object detection
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                x_offset=1.2 * retrial_x_offset,
                                                y_offset=1.2 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],
                                                self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
        ### B to A ### B to A### B to A### B to A### B to A### B to A### B to A### B to A### B to A### B to A### B to A

        else:  # B to A
            retrial_x_offset = 7 / 1000.0
            base_retrial_x_offset = 7 / 1000.0
            retrial_y_offset = 7 / 1000.0
            base_retrial_y_offset = 7 / 1000.0
            if whichobject == 'total':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.totalObjectListInBoxBReal):
                        self.tempTargetBpos[0] = self.totalObjectListInBoxBReal[0].glo_x
                        self.tempTargetBpos[1] = self.totalObjectListInBoxBReal[0].glo_y
                        self.tempTargetBpos[2] = self.totalObjectListInBoxBReal[0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.totalObjectListInBoxBReal):
                        self.tempTargetBpos[0] = self.totalObjectListInBoxBReal[0].glo_x
                        self.tempTargetBpos[1] = self.totalObjectListInBoxBReal[0].glo_y
                        self.tempTargetBpos[2] = self.totalObjectListInBoxBReal[0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'cantata_latte':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['cantata_latte']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['cantata_latte'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['cantata_latte']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['cantata_latte'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'starbucks_skunnylatte':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['starbucks_skunnylatte']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['starbucks_skunnylatte']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'chokoemong':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['chokoemong']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['chokoemong'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['chokoemong']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['chokoemong'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'dole_mango':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['dole_mango']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['dole_mango'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['dole_mango']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['dole_mango'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
            elif whichobject == 'danone_greek':
                if numTrial == 1:  # 1st retril
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['danone_greek']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['danone_greek'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-retrial_x_offset,
                                                y_offset=retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                elif numTrial == 2:
                    # self.detectObjectsinBoxes()
                    if len(self.curDetectedItemsbyClassB['danone_greek']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['danone_greek'][
                            0].glo_z  # re-object detection
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset,
                                                z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(0.5)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                x_offset=-1.6 * retrial_x_offset,
                                                y_offset=1.6 * retrial_y_offset)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        time.sleep(0.1)
                        self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                QApplication.processEvents()

    def pickplace_AtoB(self):
        self.flagAtoBorBtoA = False
        self.detectObjectsinBoxes()
        initial_amount = len(self.totalObjectListInBoxAReal)
        if self.flagAtoBorBtoA == False:  # AtoB
            # print('palletizing starts')
            self.LogText.append(_fromUtf8("palletizing starts"))
            if len(self.totalObjectListInBoxAReal):
                self.detectObjectsinBoxes()
                QApplication.processEvents()
                for i in range(0, 2* initial_amount):
                    # QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.totalObjectListInBoxAReal):
                        self.tempTargetApos[0] = self.totalObjectListInBoxAReal[0].glo_x
                        self.tempTargetApos[1] = self.totalObjectListInBoxAReal[0].glo_y
                        self.tempTargetApos[2] = self.totalObjectListInBoxAReal[0].glo_z
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping

                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime2)
                        if self.totalObjectListInBoxAReal[0].label != 'danone_greek' :
                            if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                                self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                        self.tempTargetAposRobCoord[2], z_offset=self.retrieve_height)
                                # self.addInitPoseWayPoint()
                                self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                                QApplication.processEvents()
                                time.sleep(0.1)
                                self.okToPalletize = True
                            else:
                                for retry_iter in range(1, 3):
                                    self.grippingRetrial(retry_iter, whichobject='total')
                                    if self.checkIfGrippingSuccessful(
                                            self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                        self.okToPalletize = True
                                        break
                                    else:
                                        self.okToPalletize = False
                        else:
                            self.okToPalletize = True

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetAposRobCoord[2])
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if not len(self.totalObjectListInBoxAReal):
                            break


    def pickplace_BtoA(self):
        self.flagAtoBorBtoA = True
        self.detectObjectsinBoxes()
        initial_amount = len(self.totalObjectListInBoxBReal)
        if self.flagAtoBorBtoA == True:  # BtoA
            self.LogText.append(_fromUtf8("palletizing starts"))

            if len(self.totalObjectListInBoxBReal):
                self.detectObjectsinBoxes()
                for i in range(0, 2* initial_amount):
                    QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.totalObjectListInBoxBReal):
                        self.tempTargetBpos[0] = self.totalObjectListInBoxBReal[0].glo_x
                        self.tempTargetBpos[1] = self.totalObjectListInBoxBReal[0].glo_y
                        self.tempTargetBpos[2] = self.totalObjectListInBoxBReal[0].glo_z
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.totalObjectListInBoxBReal[0].label != 'danone_greek':
                            if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                                self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                        self.tempTargetBposRobCoord[2], z_offset=self.retrieve_height)
                                # self.addInitPoseWayPoint()
                                self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                                QApplication.processEvents()
                                self.okToPalletize = True
                            else:
                                for retry_iter in range(1, 3):
                                    self.grippingRetrial(retry_iter, whichobject='total')
                                    if self.checkIfGrippingSuccessful(
                                            self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                        self.okToPalletize = True
                                        break
                                    else:
                                        self.okToPalletize = False
                        else:
                            self.okToPalletize = True

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetBposRobCoord[2])
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if not len(self.totalObjectListInBoxBReal):
                            break


    def pickCantataBtn(self):
        self.detectObjectsinBoxes()
        if self.flagAtoBorBtoA == False: # AtoB
            self.LogText.append(_fromUtf8('moving cantata from A to B'))
            object_amount = len(self.curDetectedItemsbyClassA['cantata_latte'])
            if len(self.curDetectedItemsbyClassA['cantata_latte']):
                self.detectObjectsinBoxes()
                for i in range(0,2*object_amount):
                    QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassA['cantata_latte']):
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['cantata_latte'][0].glo_z
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],self.tempTargetAposRobCoord[1],self.tempTargetAposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0],self.tempTargetAposRobCoord[1],self.tempTargetAposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                    self.tempTargetAposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            QApplication.processEvents()
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='cantata_latte')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetAposRobCoord[2])
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassA['cantata_latte']):
                            break

        else :  # BtoA
            self.LogText.append(_fromUtf8('moving cantata from B to A'))

            object_amount = len(self.curDetectedItemsbyClassB['cantata_latte'])
            if len(self.curDetectedItemsbyClassB['cantata_latte']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassB['cantata_latte']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['cantata_latte'][0].glo_z
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                    self.tempTargetBposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            QApplication.processEvents()
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='cantata_latte')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetBposRobCoord[2])
                        else:
                            self.gripper_off()
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassB['cantata_latte']):
                            break

    def pickStarbucksBtn(self):
        self.detectObjectsinBoxes()
        if self.flagAtoBorBtoA == False:  # AtoB
            self.LogText.append(_fromUtf8('moving starbucks from A to B'))
            object_amount = len(self.curDetectedItemsbyClassA['starbucks_skunnylatte'])
            if len(self.curDetectedItemsbyClassA['starbucks_skunnylatte']):
                for i in range(0, 2*object_amount):
                    QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassA['starbucks_skunnylatte']):
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['starbucks_skunnylatte'][0].glo_z
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                    self.tempTargetAposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            QApplication.processEvents()
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='starbucks_skunnylatte')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetAposRobCoord[2])
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassA['starbucks_skunnylatte']):
                            break
        else :  # BtoA
            self.LogText.append(_fromUtf8('moveing starbucks from B to A'))
            object_amount = len(self.curDetectedItemsbyClassB['starbucks_skunnylatte'])
            if len(self.curDetectedItemsbyClassB['starbucks_skunnylatte']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassB['starbucks_skunnylatte']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['starbucks_skunnylatte'][0].glo_z
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                    self.tempTargetBposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            QApplication.processEvents()
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='starbucks_skunnylatte')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetBposRobCoord[2])
                        else:
                            self.gripper_off()
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassB['starbucks_skunnylatte']):
                            break
    def pickChocoemonBtn(self):
        self.detectObjectsinBoxes()
        if self.flagAtoBorBtoA == False:  # AtoB
            self.LogText.append(_fromUtf8('moving chocoemong from A to B'))
            object_amount = len(self.curDetectedItemsbyClassA['chokoemong'])
            if len(self.curDetectedItemsbyClassA['chokoemong']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    QApplication.processEvents()
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassA['chokoemong']):
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['chokoemong'][0].glo_z
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                    self.tempTargetAposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='chokoemong')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False


                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetAposRobCoord[2])
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassA['chokoemong']):
                            break
        else :  # BtoA
            self.LogText.append(_fromUtf8('moving chocoemong from B to A'))
            object_amount = len(self.curDetectedItemsbyClassB['chokoemong'])
            if len(self.curDetectedItemsbyClassB['chokoemong']):
                self.detectObjectsinBoxes()
                for i in range(0, 2* object_amount):
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassB['chokoemong']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['chokoemong'][0].glo_z
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping

                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2])
                        self.th.sendTrajectory()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                    self.tempTargetBposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='chokoemong')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False


                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetBposRobCoord[2])
                        else:
                            self.gripper_off()
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassB['chokoemong']):
                            break
    def pickDoleMangoBtn(self):
        self.detectObjectsinBoxes()
        if self.flagAtoBorBtoA == False:  # AtoB
            self.LogText.append(_fromUtf8('moving dole_mango from A to B'))
            object_amount = len(self.curDetectedItemsbyClassA['dole_mango'])
            if len(self.curDetectedItemsbyClassA['dole_mango']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassA['dole_mango']):
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['dole_mango'][0].glo_z
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                    self.tempTargetAposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            QApplication.processEvents()
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='dole_mango')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False


                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetAposRobCoord[2])
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassA['dole_mango']):
                            break
        else :  # BtoA
            self.LogText.append(_fromUtf8('moving dole_mango from B to A'))
            object_amount = len(self.curDetectedItemsbyClassB['dole_mango'])
            if len(self.curDetectedItemsbyClassB['dole_mango']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassB['dole_mango']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['dole_mango'][0].glo_z
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        if self.checkIfGrippingSuccessful(self.nopayload_Fz):  # Successful gripping
                            self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                    self.tempTargetBposRobCoord[2], z_offset=self.retrieve_height)
                            # self.addInitPoseWayPoint()
                            self.th.sendTrajectory()  # Up to now, it's one fraction of palletizing
                            QApplication.processEvents()
                            self.okToPalletize = True
                        else:
                            for retry_iter in range(1, 3):
                                self.grippingRetrial(retry_iter, whichobject='dole_mango')
                                if self.checkIfGrippingSuccessful(
                                        self.nopayload_Fz):  # if retry sucessful, go to palletizing work
                                    self.okToPalletize = True
                                    break
                                else:
                                    self.okToPalletize = False


                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetBposRobCoord[2])
                        else:
                            self.gripper_off()
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassB['dole_mango']):
                            break

    def pickDanonGreekBtn(self):
        self.detectObjectsinBoxes()
        if self.flagAtoBorBtoA == False:  # AtoB
            self.LogText.append(_fromUtf8('moving danon_greek from A to B'))
            object_amount = len(self.curDetectedItemsbyClassA['danone_greek'])
            if len(self.curDetectedItemsbyClassA['danone_greek']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassA['danone_greek']):
                        self.tempTargetApos[0] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_x
                        self.tempTargetApos[1] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_y
                        self.tempTargetApos[2] = self.curDetectedItemsbyClassA['danone_greek'][0].glo_z
                        self.tempTargetAposRobCoord = self.coordTransformA(self.tempTargetApos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetAposRobCoord[0], self.tempTargetAposRobCoord[1],
                                                self.tempTargetAposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        self.okToPalletize = True

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetAposRobCoord[2])
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        else:
                            self.gripper_off()
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassA['danone_greek']):
                            break
        else :  # BtoA
            self.LogText.append(_fromUtf8('moving danon_greek from B to A'))
            object_amount = len(self.curDetectedItemsbyClassB['danone_greek'])
            if len(self.curDetectedItemsbyClassB['danone_greek']):
                self.detectObjectsinBoxes()
                for i in range(0, 2*object_amount):
                    self.detectObjectsinBoxes()
                    self.th.clearTrajectory()
                    if len(self.curDetectedItemsbyClassB['danone_greek']):
                        self.tempTargetBpos[0] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_x
                        self.tempTargetBpos[1] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_y
                        self.tempTargetBpos[2] = self.curDetectedItemsbyClassB['danone_greek'][0].glo_z
                        self.tempTargetBposRobCoord = self.coordTransformB(self.tempTargetBpos)
                        # self.addInitPoseWayPoint()
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.approach_height)
                        self.th.sendTrajectory()
                        rospy.sleep(self.waitoverObjectTime1)
                        self.nopayload_Fz = self.get_nopayload_Fz()  # Used for checking successful gripping
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2])
                        self.th.sendTrajectory()
                        # QApplication.processEvents()
                        self.gripper_on()
                        rospy.sleep(self.waitoverObjectTime2)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0], self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2], z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(self.waitoverObjectTime1)
                        QApplication.processEvents()
                        self.okToPalletize = True

                        if self.okToPalletize:
                            self.palletizingService(self.palletizingStartBoxPoint, pickedObjectheight=self.tempTargetBposRobCoord[2])
                        else:
                            self.gripper_off()
                            # Palletizing should follow
                            # refer to self.palletizingSuccessCount
                        if self.palletizingStep == 12:
                            self.palletizingStep = 0
                            break
                        if not len(self.curDetectedItemsbyClassB['danone_greek']):
                            break

    def addInitPoseWayPoint(self):
        rot_command = PyKDL.Rotation.RPY(rad(self.initPos['Roll']),
                                        rad(self.initPos['Pitch']),
                                         rad(self.initPos['Yaw']))
        self.quat_angle = rot_command.GetQuaternion()
        self.targetPose.position.x = self.initPos['x'] / 1000.0
        self.targetPose.position.y = self.initPos['y'] / 1000.0
        self.targetPose.position.z = self.initPos['z'] / 1000.0
        self.targetPose.orientation.x = self.quat_angle[0]
        self.targetPose.orientation.y = self.quat_angle[1]
        self.targetPose.orientation.z = self.quat_angle[2]
        self.targetPose.orientation.w = self.quat_angle[3]
        self.poseStamped.pose = self.targetPose
        self.th.waypoint.set_cartesian_pose(self.poseStamped, 'right_hand')
        self.th.waypoint.set_cartesian_pose(self.poseStamped, 'right_hand')
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.LogText.append(_fromUtf8('Initial_waypoint_appended'))

    def addMotionWayPoints(self, x, y, z, roll=-179.0, pitch=0.0, yaw=100.0, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        rot_command = PyKDL.Rotation.RPY(rad(roll), rad(pitch), rad(yaw))
        self.quat_angle = rot_command.GetQuaternion()
        self.targetPose.position.x = x / 1000.0 + x_offset
        self.targetPose.position.y = y / 1000.0 + y_offset
        self.targetPose.position.z = z / 1000.0 + z_offset + self.gripperlength
        self.targetPose.orientation.x = self.quat_angle[0]
        self.targetPose.orientation.y = self.quat_angle[1]
        self.targetPose.orientation.z = self.quat_angle[2]
        self.targetPose.orientation.w = self.quat_angle[3]
        self.poseStamped.pose = self.targetPose
        # print (self.poseStamped.pose)
        self.th.waypoint.set_cartesian_pose(self.poseStamped, 'right_hand', None)
        self.th.waypoint.set_cartesian_pose(self.poseStamped, 'right_hand', None)
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.waypoint_count += 1
        self.LogText.append(_fromUtf8('Waypoint %d appended' % self.waypoint_count))

    def onClickedCalibBoxA(self):
        if not self.timer1Flag:
            # print('Calibration for Box A has started')
            self.LogText.append('Setup for sawyer way points')
            self.poseACount == 0
            self.timer1Flag = True
            self.timer1.start(10)
        else:
            # print('Calibration has finished')
            self.LogText.append('Calibration has finished')

            self.timer1Flag = False
            self.timer1.stop()
            self.offsetXYZboxA[0] = (self.robposeforCalibrationA['first']['x'] + self.robposeforCalibrationA['second']['x']
                                     + self.robposeforCalibrationA['third']['x'] + self.robposeforCalibrationA['fourth']['x'])/4
            self.offsetXYZboxA[1] = (self.robposeforCalibrationA['first']['y'] + self.robposeforCalibrationA['second']['y']
                                     + self.robposeforCalibrationA['third']['y'] + self.robposeforCalibrationA['fourth']['y'])/4
            self.LogText.append('Calibrated pos. for box A x : %f  y : %f' %(self.offsetXYZboxA[0],self.offsetXYZboxA[1]))
            self.boxA_Xoffset.setText(QtCore.QString.number(self.offsetXYZboxA[0], 'f', 3))
            self.boxA_Yoffset.setText(QtCore.QString.number(self.offsetXYZboxA[1], 'f', 3))



    def onClickedCalibBoxB(self):
        if not self.timer2Flag:
            self.LogText.append('Calibration for Box B has started')
            self.poseBCount == 0
            self.timer2Flag = True
            self.timer2.start(10)
        else:
            self.LogText.append('Calibration has finished')
            self.timer2Flag = False
            self.timer2.stop()
            self.offsetXYZboxB[0] = (self.robposeforCalibrationB['first']['x'] + self.robposeforCalibrationB['second']['x']
                                     + self.robposeforCalibrationB['third']['x'] + self.robposeforCalibrationB['fourth']['x'])/4
            self.offsetXYZboxB[1] = (self.robposeforCalibrationB['first']['y'] + self.robposeforCalibrationB['second']['y']
                                     + self.robposeforCalibrationB['third']['y'] + self.robposeforCalibrationB['fourth']['y'])/4
            self.LogText.append('Calibrated pos. for box B x : %f  y : %f' %(self.offsetXYZboxB[0],self.offsetXYZboxB[1]))
            self.boxB_Xoffset.setText(QtCore.QString.number(self.offsetXYZboxB[0], 'f', 3))
            self.boxB_Yoffset.setText(QtCore.QString.number(self.offsetXYZboxB[1], 'f', 3))

    def calibrateboxAPos(self):
        if self.cuff.lower_button():
            self.calibposeAcquired = self.endpoint_pose = self.th.limb.endpoint_pose() #pose = {'position': (x,y,z), ...}
            if self.poseACount == 0:
                self.robposeforCalibrationA['first']['x'] = self.calibposeAcquired['position'][0] *1000.0# in mm
                self.robposeforCalibrationA['first']['y'] = self.calibposeAcquired['position'][1] *1000.0# in mm
                self.robposeforCalibrationA['first']['z'] = self.calibposeAcquired['position'][2] *1000.0# in mm
                self.poseACount += 1
                self.LogText.append('1st position has been saved')
                rospy.sleep(1.5)
            elif self.poseACount == 1:
                self.robposeforCalibrationA['second']['x'] = self.calibposeAcquired['position'][0] *1000.0# in mm
                self.robposeforCalibrationA['second']['y'] = self.calibposeAcquired['position'][1] *1000.0# in mm
                self.robposeforCalibrationA['second']['z'] = self.calibposeAcquired['position'][2] *1000.0# in mm
                self.poseACount += 1
                self.LogText.append('2nd position has been saved')
                rospy.sleep(1.5)
            elif self.poseACount == 2:
                self.robposeforCalibrationA['third']['x'] = self.calibposeAcquired['position'][0] *1000.0# in mm
                self.robposeforCalibrationA['third']['y'] = self.calibposeAcquired['position'][1] *1000.0# in mm
                self.robposeforCalibrationA['third']['z'] = self.calibposeAcquired['position'][2] *1000.0# in mm
                self.poseACount += 1
                self.LogText.append('3rd position has been saved')
                rospy.sleep(1.5)


            elif self.poseACount == 3:
                self.robposeforCalibrationA['fourth']['x'] = self.calibposeAcquired['position'][0] *1000.0# in mm
                self.robposeforCalibrationA['fourth']['y'] = self.calibposeAcquired['position'][1] *1000.0# in mm
                self.robposeforCalibrationA['fourth']['z'] = self.calibposeAcquired['position'][2] *1000.0# in mm
                self.poseACount += 1
                self.LogText.append('4th position has been saved')
                rospy.sleep(1.5)

    def calibrateboxBPos(self):
        if self.cuff.lower_button():
            self.calibposeAcquired = self.endpoint_pose = self.th.limb.endpoint_pose() #pose = {'position': (x,y,z), ...}
            if self.poseBCount == 0:
                self.robposeforCalibrationB['first']['x'] = self.calibposeAcquired['position'][0] *1000.0# in meter
                self.robposeforCalibrationB['first']['y'] = self.calibposeAcquired['position'][1] *1000.0# in meter
                self.robposeforCalibrationB['first']['z'] = self.calibposeAcquired['position'][2] *1000.0# in meter
                self.poseBCount += 1
                self.LogText.append('1st position has been saved')
                rospy.sleep(1.5)

            elif self.poseBCount == 1:
                self.robposeforCalibrationB['second']['x'] = self.calibposeAcquired['position'][0] *1000.0# in meter
                self.robposeforCalibrationB['second']['y'] = self.calibposeAcquired['position'][1] *1000.0# in meter
                self.robposeforCalibrationB['second']['z'] = self.calibposeAcquired['position'][2] *1000.0 # in meter
                self.poseBCount += 1
                self.LogText.append('2nd position has been saved')
                rospy.sleep(1.5)

            elif self.poseBCount == 2:
                self.robposeforCalibrationB['third']['x'] = self.calibposeAcquired['position'][0] *1000.0 # in meter
                self.robposeforCalibrationB['third']['y'] = self.calibposeAcquired['position'][1] *1000.0 # in meter
                self.robposeforCalibrationB['third']['z'] = self.calibposeAcquired['position'][2] *1000.0 # in meter
                self.poseBCount += 1
                self.LogText.append('3rd position has been saved')
                rospy.sleep(1.5)

            elif self.poseBCount == 3:
                self.robposeforCalibrationB['fourth']['x'] = self.calibposeAcquired['position'][0] *1000.0 # in meter
                self.robposeforCalibrationB['fourth']['y'] = self.calibposeAcquired['position'][1] *1000.0 # in meter
                self.robposeforCalibrationB['fourth']['z'] = self.calibposeAcquired['position'][2] *1000.0 # in meter
                self.poseBCount += 1
                self.LogText.append('4th position has been saved')
                rospy.sleep(1.5)

    def checkIfGrippingSuccessful(self, nonPayLoad=0.0):
        result = False
        total_cur_Fz = 0.0
        QApplication.processEvents()
        for j in range(0,10):
            QApplication.processEvents()
            total_cur_Fz += self.curFzValue
            rospy.sleep(0.1)
        cur_Fz_avg = total_cur_Fz / 10.0
        nonPayLoad =0.00
        self.LogText.append('weight of object : %f' %cur_Fz_avg)

        if abs(abs(nonPayLoad)-abs(cur_Fz_avg))<=self.objectWeightThreshold:
            self.LogText.append('Gripping has failed, retry...')
            result = False
        else:
            result = True
            self.LogText.append('Gripping has succeeded, keep going on')

        return result

    def palletizingService(self, startPosition = None, pickedObjectheight = 0.0):
        palletizing_waittime = 0.5
        self.curPallet_order.setText(QtCore.QString.number(self.palletizingStep))
        self.cur_objInfo.setText()
        self.LogText.append('urrent object height: %f' %pickedObjectheight)
        ## Box geometry 470mm * 320mm -> divided into 12 areas
        ## Palletizing startWaypoint is located top left (-176.25,106.66) w.r.t box frame right (posX), up(posY)
        palletizing_movement_BoxA = [0.05875*2,-0.05333*2 ]
        palletizing_movement_BoxB = [0.05333*2,0.05875*2 ]
        if self.flagAtoBorBtoA == False:
            self.palletizingStartRobCoordB = self.coordTransformB(startPosition)
            self.palletizingStartRobCoordB[2] = pickedObjectheight + 0.01
        else:
            self.palletizingStartRobCoordA = self.coordTransformA(startPosition)
            self.palletizingStartRobCoordA[2] = pickedObjectheight + 0.01
        pose = self.endpoint_state.pose # extract from palletizing start
        # coordinate transform is necesary
        if self.flagAtoBorBtoA == False: # AtoB
            if self.palletizingStep == 0: # place
                self.LogText.append('palletizing has started : placing @ 1st position')
                self.th.clearTrajectory()
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 1:
                self.LogText.append('placing @ 2nd position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, y_offset =palletizing_movement_BoxB[1],z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, y_offset=palletizing_movement_BoxB[1], z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, y_offset =palletizing_movement_BoxB[1],z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 2:
                self.LogText.append('placing @ 3rd position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2],x_offset = self.pallet_firstrow_offset,  y_offset=2*palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, y_offset=2*palletizing_movement_BoxB[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2],x_offset = self.pallet_firstrow_offset,  y_offset=2*palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 3:
                self.LogText.append('placing @ 4th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2],x_offset = self.pallet_firstrow_offset,  y_offset=3*palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset = self.pallet_firstrow_offset, y_offset=3*palletizing_movement_BoxB[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2],x_offset = self.pallet_firstrow_offset,  y_offset=3*palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 4:
                self.LogText.append('placing @ 5th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 5:
                self.LogText.append('placing @ 6th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=palletizing_movement_BoxB[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 6:
                self.LogText.append('placing @ 7th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=2*palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=2*palletizing_movement_BoxB[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=2*palletizing_movement_BoxB[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 7:
                self.LogText.append('placing @ 8th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0]  , y_offset=3*palletizing_movement_BoxB[1], z_offset= self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0] , y_offset=3*palletizing_movement_BoxB[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=palletizing_movement_BoxB[0]  , y_offset=3*palletizing_movement_BoxB[1], z_offset= self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 8:
                self.LogText.append('placing @ 9th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset= -self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0], z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0], z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset= -self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0], z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 9:
                self.LogText.append('placing @ 10th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0], y_offset=palletizing_movement_BoxB[1], z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=palletizing_movement_BoxB[1], z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0], y_offset=palletizing_movement_BoxB[1], z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 10:
                self.LogText.append('placing @ 11th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=2*palletizing_movement_BoxB[1], z_offset=self.palletizing_height+ self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=2*palletizing_movement_BoxB[1], z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=2*palletizing_movement_BoxB[1], z_offset=self.palletizing_height+ self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 11:
                self.LogText.append('placing @ 12th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=3*palletizing_movement_BoxB[1], z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=3*palletizing_movement_BoxB[1], z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordB[0], self.palletizingStartRobCoordB[1], self.palletizingStartRobCoordB[2], x_offset=-self.pallet_thirdrow_offset + 2 * palletizing_movement_BoxB[0] , y_offset=3*palletizing_movement_BoxB[1], z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 12:
                self.LogText.append('Palletizing 12 objects has finished. retrieving robot to its initial position')
                #### B to A #### B to A #### B to A #### B to A #### B to A #### B to A #### B to A #### B to A #### B to A #### B to A #### B to A ####
        else:
            if self.palletizingStep == 0: # place
                self.LogText.append('palletizing has started : placing @ 1st position')
                self.th.clearTrajectory()
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset = -self.pallet_firstrow_offset, z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset = -self.pallet_firstrow_offset, z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset = -self.pallet_firstrow_offset, z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 1:
                self.LogText.append('placing @ 2nd position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset = palletizing_movement_BoxA[0],  y_offset = -self.pallet_firstrow_offset ,z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset = palletizing_movement_BoxA[0], y_offset = -self.pallet_firstrow_offset,  z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset = palletizing_movement_BoxA[0],  y_offset = -self.pallet_firstrow_offset ,z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 2:
                self.LogText.append('placing @ 3rd position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2],x_offset = 2*palletizing_movement_BoxA[0],  y_offset = -self.pallet_firstrow_offset,  z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2],x_offset = 2*palletizing_movement_BoxA[0],  y_offset = -self.pallet_firstrow_offset,  z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2],x_offset = 2*palletizing_movement_BoxA[0],  y_offset = -self.pallet_firstrow_offset,  z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 3:
                self.LogText.append('placing @ 4th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2],x_offset = 3*palletizing_movement_BoxA[0], y_offset = -self.pallet_firstrow_offset,z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset = 3*palletizing_movement_BoxA[0], y_offset = -self.pallet_firstrow_offset , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2],x_offset = 3*palletizing_movement_BoxA[0], y_offset = -self.pallet_firstrow_offset,z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 4:
                self.LogText.append('placing @ 5th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset=palletizing_movement_BoxA[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset=palletizing_movement_BoxA[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset=palletizing_movement_BoxA[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 5:
                self.LogText.append('placing @ 6th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 6:
                self.LogText.append('placing @ 7th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=2*palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1],z_offset=self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=2*palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1] , z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=2*palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1],z_offset=self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 7:
                self.LogText.append('placing @ 8th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=3*palletizing_movement_BoxA[0]  , y_offset=palletizing_movement_BoxA[1], z_offset= self.approach_height + self.palletizing_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=3*palletizing_movement_BoxA[0] , y_offset=palletizing_movement_BoxA[1] , z_offset= self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=3*palletizing_movement_BoxA[0]  , y_offset=palletizing_movement_BoxA[1], z_offset= self.approach_height + self.palletizing_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 8:
                self.LogText.append('step %d has finished', self.palletizingStep)
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 9:
                self.LogText.append('placing @ 10th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=palletizing_movement_BoxA[0], y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=palletizing_movement_BoxA[0], y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=palletizing_movement_BoxA[0], y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 10:
                self.LogText.append('placing @ 11th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=2*palletizing_movement_BoxA[0] , y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height+ self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=2*palletizing_movement_BoxA[0] , y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=2*palletizing_movement_BoxA[0] , y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height+ self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 11:
                self.LogText.append('placing @ 12th position')
                # self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=3*palletizing_movement_BoxA[0] , y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height + self.approach_height)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=3*palletizing_movement_BoxA[0] , y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height)
                self.th.sendTrajectory()
                # QApplication.processEvents()
                self.gripper_off()
                rospy.sleep(palletizing_waittime)
                self.addMotionWayPoints(self.palletizingStartRobCoordA[0], self.palletizingStartRobCoordA[1], self.palletizingStartRobCoordA[2], x_offset=3*palletizing_movement_BoxA[0] , y_offset=2*palletizing_movement_BoxA[1]+self.pallet_thirdrow_offset, z_offset=self.palletizing_height + self.approach_height)
                # self.addInitPoseWayPoint()
                self.th.sendTrajectory()
                self.palletizingStep += 1
                # QApplication.processEvents()
                self.LogText.append('step %d has finished', self.palletizingStep)
            elif self.palletizingStep == 12:
                self.LogText.append('Palletizing 12 objects has finished. retrieving robot to its initial position')
        self.okToPalletize = False
        QApplication.processEvents()

    def coordTransformA(self, position):
        # Right-side to the Sawer's control box pheriperal
        # position array of 3 DoF  (X,Y,Z)
        # position[0] = X, position[1] = Y, position[2] = Z in camera coordinate
        r_z_right = 0.0 # Yaw rotation
        t_x = self.offsetXYZboxA[0]
        t_y = self.offsetXYZboxA[1]
        t_z = self.offsetXYZboxA[2]
        objectPose_boxFrame = PyKDL.Vector(position[0], position[1], position[2])

        rotation_matrix = PyKDL.Rotation.RotZ(rad(r_z_right)) # 3*3
        translation_matrix = PyKDL.Vector(t_x,t_y,t_z)  # 3*1./
        homegen_tf_matrix = PyKDL.Frame(rotation_matrix, translation_matrix)
        objectPose_robotFrame1 = homegen_tf_matrix * objectPose_boxFrame # 3*1
        return objectPose_robotFrame1


    def coordTransformB(self, position):

        # Right-side to the Sawer's control box pheriperal

        # position array of 3 DoF  (X,Y,Z)
        # position[0] = X, position[1] = Y, position[2] = Z in camera coordinate
        r_x_right = 90.0 # Yaw rotation
        r_y_right = 90.0 # Yaw rotation
        r_z_right = 90.0 # Yaw rotation
        t_x = self.offsetXYZboxB[0]
        t_y = self.offsetXYZboxB[1]
        t_z = self.offsetXYZboxB[2]

        objectPose_boxFrame = PyKDL.Vector(position[0], position[1], position[2])

        rotation_matrix = PyKDL.Rotation.RotZ(rad(r_z_right)) # 3*3
        translation_matrix = PyKDL.Vector(t_x,t_y,t_z)  # 3*1
        homegen_tf_matrix = PyKDL.Frame(rotation_matrix, translation_matrix)
        objectPose_robotFrame2  =   homegen_tf_matrix * objectPose_boxFrame # 3*1
        return objectPose_robotFrame2

    def robot_Button_Clicked(self):  # Sawyer Enable/Disable

        if self.Button_Init_Node.isChecked():
            self.LogText.append(_fromUtf8("Sawyer is being enabled"))
            self.enable_sawyer()
            global isEnabled
            isEnabled = True

            self.th.trajectory.clear_waypoints()
            self.endpoint_state = self.th.limb.tip_state('right_hand')
            self.curPose = self.endpoint_state.pose
            self.poseStamped1A = PoseStamped()
            self.poseStamped1A.pose = self.curPose
            self.th.waypoint_initial.set_cartesian_pose(self.poseStamped1A, 'right_hand', None)


        elif not self.Button_Init_Node.isChecked():
            self.LogText.append(_fromUtf8("Sawyer is being disabled"))
            self.disable_sawyer()
            global isEnabled
            isEnabled = False

    def command_UR5(self):  # Reset Sawyer

        if self.isROSPyrunning:
            self.robotiq_open()
            self.gripper_off()
            self.th2.ur5.movel(self.th2.wp1,self.th2.lin_accel,self.th2.lin_vel)
            self.th2.ur5.movel(self.th2.wp2,self.th2.lin_accel,self.th2.lin_vel)
            self.th2.ur5.movel(self.th2.wp3,self.th2.lin_accel,self.th2.lin_vel)
            self.th2.ur5.movel(self.th2.wp4,self.th2.lin_accel,self.th2.lin_vel)
            self.th2.ur5.stopl(self.th2.lin_accel)
            self.robotiq_close()
            self.addInitPoseWayPoint()
            self.addMotionWayPoints(self.sawyer_wp1[0], self.sawyer_wp1[1], self.sawyer_wp1[2],
                                    roll=self.sawyer_wp1[3], pitch = self.sawyer_wp1[4], yaw = self.sawyer_wp1[5])
            self.addMotionWayPoints(self.sawyer_wp2[0], self.sawyer_wp2[1], self.sawyer_wp2[2],
                                    roll=self.sawyer_wp2[3], pitch = self.sawyer_wp2[4], yaw = self.sawyer_wp2[5])
            self.addMotionWayPoints(self.sawyer_wp3[0], self.sawyer_wp3[1], self.sawyer_wp3[2],
                                    roll=self.sawyer_wp3[3], pitch = self.sawyer_wp3[4], yaw = self.sawyer_wp3[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()
            self.gripper_on()
            QApplication.processEvents()
            rospy.sleep(1.0)
            self.addMotionWayPoints(self.sawyer_wp4[0], self.sawyer_wp4[1], self.sawyer_wp4[2],
                                    roll=self.sawyer_wp4[3], pitch = self.sawyer_wp4[4], yaw = self.sawyer_wp4[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()
            QApplication.processEvents()
            self.addMotionWayPoints(self.sawyer_wp5[0], self.sawyer_wp5[1], self.sawyer_wp5[2],
                                    roll=self.sawyer_wp5[3], pitch=self.sawyer_wp5[4], yaw=self.sawyer_wp5[5])

            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()
            QApplication.processEvents()
            #
            self.addMotionWayPoints(self.sawyer_wp6[0], self.sawyer_wp6[1], self.sawyer_wp6[2],
                                    roll=self.sawyer_wp6[3], pitch=self.sawyer_wp6[4], yaw=self.sawyer_wp6[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()
            QApplication.processEvents()
            # self.gripper_off()
            rospy.sleep(1.0)
            self.addMotionWayPoints(self.sawyer_wp7[0], self.sawyer_wp7[1], self.sawyer_wp7[2],
                                    roll=self.sawyer_wp7[3], pitch=self.sawyer_wp7[4], yaw=self.sawyer_wp7[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()
            QApplication.processEvents()
            self.addMotionWayPoints(self.sawyer_wp6[0], self.sawyer_wp6[1], self.sawyer_wp6[2],
                                    roll=self.sawyer_wp6[3], pitch=self.sawyer_wp6[4], yaw=self.sawyer_wp6[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()
            # self.gripper_on()
            rospy.sleep(1.0)
            self.addMotionWayPoints(self.sawyer_wp8[0], self.sawyer_wp8[1], self.sawyer_wp8[2],
                                    roll=self.sawyer_wp8[3], pitch=self.sawyer_wp8[4], yaw=self.sawyer_wp8[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()

            self.addMotionWayPoints(self.sawyer_wp9[0], self.sawyer_wp9[1], self.sawyer_wp9[2],
                                    roll=self.sawyer_wp9[3], pitch=self.sawyer_wp9[4], yaw=self.sawyer_wp9[5])
            self.th.sendTrajectory()
            # self.sendSawyerTrajSignal()

            self.th2.ur5.movel(self.th2.wp5,self.th2.lin_accel,self.th2.lin_vel)


        else:
            rospy.sleep(1)
            print ('Resuming robot motion')

    def switchMode(self):

        if self.flagAtoBorBtoA == False: # AtoB
            self.flagAtoBorBtoA = True
            self.currentDir_text.setText('BtoA')
        else:
            self.flagAtoBorBtoA = False
            self.currentDir_text.setText('AtoB')

    def update_command_clicked(self):

        self.th.trajectory.clear_waypoints()

        self.endpoint_state = self.th.limb.tip_state('right_hand')
        self.curPose = self.endpoint_state.pose
        self.poseStamped1 = PoseStamped()

        self.poseStamped1.pose = self.curPose
        self.LogText.append('Pause the robot motion')

        self.th.waypoint.set_cartesian_pose(self.poseStamped1, 'right_hand', None)
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.LogText.append('Waypoint1 appended')

        self.targetPose = self.endpoint_state.pose

        self.targetPose.position.x = float(self.X_command.toPlainText()) / 1000.0
        self.targetPose.position.y = float(self.Y_command.toPlainText()) / 1000.0
        self.targetPose.position.z = float(self.Z_command.toPlainText()) / 1000.0

        if self.Rx_command and self.Ry_command and self.Rz_command:
            rot_command = PyKDL.Rotation.EulerZYX(rad(float(self.Rz_command.toPlainText())),
                                                  rad(float(self.Ry_command.toPlainText())),
                                                  rad(float(self.Rx_command.toPlainText()))
                                                  )
            self.quat_angle = rot_command.GetQuaternion()

        self.targetPose.orientation.x = self.quat_angle[0]
        self.targetPose.orientation.y = self.quat_angle[1]
        self.targetPose.orientation.z = self.quat_angle[2]
        self.targetPose.orientation.w = self.quat_angle[3]

        self.poseStamped2 = PoseStamped()
        self.poseStamped2.pose = self.targetPose
        self.th.waypoint.set_cartesian_pose(self.poseStamped2, 'right_hand')
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.LogText.append('Waypoint2 appended')

        self.th.temp_trajectory.clear_waypoints()
        self.th.temp_waypoint.set_cartesian_pose(self.poseStamped2, 'right_hand', None)
        self.th.temp_trajectory.append_waypoint(self.th.temp_waypoint.to_msg())
        self.th.temp_waypoint.set_cartesian_pose(self.poseStamped1, 'right_hand', None)
        self.th.temp_trajectory.append_waypoint(self.th.temp_waypoint.to_msg())

        log = self.th.trajectory.send_trajectory()

        if self.th.limb.has_collided():
            rospy.logerr('collision detected!!!')
            rospy.sleep(.5)

        self.connect()

    def retrieve_command_clicked(self):
        try:
            self.LogText.append('Retrieving motion')
            log = self.th.temp_trajectory.send_trajectory()
            print (log)
        except OSError:
            rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
            rospy.sleep(.5)

    def read_cur_pose_clicked(self):
        self.X_command.setText(QtCore.QString(self.X_read.toPlainText()))
        self.Y_command.setText(QtCore.QString(self.My_read_2.toPlainText()))
        self.Z_command.setText(QtCore.QString(self.Mz_read_2.toPlainText()))
        self.Rx_command.setText(QtCore.QString(self.Rx_read.toPlainText()))
        self.Ry_command.setText(QtCore.QString(self.Ry_read.toPlainText()))
        self.Rz_command.setText(QtCore.QString(self.Rz_read.toPlainText()))





    def enable_sawyer(self):
        self.gripper_off()
        self.curPallet_order.setText(QtCore.QString.number(self.palletizingStep))
        # self.qt_dspimg2 = QImage(self., w, h, byte, QImage.Format_RGB888)
        # painter2 = QPainter()
        # painter2.begin(self)
        # painter2.drawImage(650, 700, self.qt_dspimg2)
        # painter2.end()



        self.RobotState = intera_interface.RobotEnable(CHECK_VERSION)
        self.LogText.append('Enabling robot...')
        self.gripperLength.setText(QtCore.QString.number(self.gripperlength*1000.0))

        self.RobotState.enable()
        self.th.limb = intera_interface.Limb("right")

        self.joint_names = self.th.limb.joint_names()  # List type joint names
        self.joint_angle = self.th.limb.joint_angles()  # Dict { string : float }
        self.joint_efforts = self.th.limb.joint_efforts()  # Dict { string : float }
        self.endpoint_pose = self.th.limb.endpoint_pose()  # Dict {string: Point string Quaternion}
        self.endpoint_effort = self.th.limb.endpoint_effort()  # Dict {string : Point string : Point}

        ####### Command desired value to the robot
        self.command_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command_torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        global isAtNeutral
        if not isAtNeutral:
            # self.th.limb.move_to_neutral()
            # self.th.trajectory.stop_trajectory()
            # self.th.clearTrajectory()
            self.th.limb.set_joint_position_speed(0.1)
            self.th.limb.move_to_joint_positions(self.initial_position)
            # self.th.limb.move_to_joint_positions(self.initial_position)
        # self.gripper.open()
        # global isGripperOpen
        # isGripperOpen = True
        self.boxA_Xoffset.setText(QtCore.QString.number(self.offsetXYZboxA[0], 'f', 3))
        self.boxA_Yoffset.setText(QtCore.QString.number(self.offsetXYZboxA[1], 'f', 3))
        self.boxB_Xoffset.setText(QtCore.QString.number(self.offsetXYZboxB[0], 'f', 3))
        self.boxB_Yoffset.setText(QtCore.QString.number(self.offsetXYZboxB[1], 'f', 3))


    def disable_sawyer(self):
        self.LogText.append('Disabling robot...')
        self.gripper_off()
        self.th2.ur5.close()
        self.robotiq_open()
        # self.th.limb.move_to_neutral()
        self.th.limb.set_joint_position_speed(0.1)
        self.th.limb.move_to_joint_positions(self.initial_position)
        self.RobotState.disable()
        global isAtNeutral
        isAtNeutral = True
        # self.gripper.close()
        # global isGripperOpen
        # isGripperOpen = False
        # self.timer3.stop()




    def init_palletizing_num(self):

        if self.palletizingStep:
            self.palletizingStep = 0
            self.LogText.append('alletizing step has been initialized.')

    def read_status_from_Intera(self):
        '''
        self.joint_names = self.th.limb.joint_names()  # List type joint names
        self.joint_angle = self.th.limb.joint_angles()  # Dict { string : float }
        self.joint_efforts = self.th.limb.joint_efforts()  # Dict { string : float }
        self.endpoint_pose = self.th.limb.endpoint_pose()  # Dict {string: Point string Quaternion}
        self.endpoint_effort = self.th.limb.endpoint_effort()  # Dict {string : Poi    nt string : Point}
        '''
        global isEnabled
        if isEnabled:
            ###  read joint positions
            self.J1_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j0'))), 'f', 3))
            self.J2_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j1'))), 'f', 3))
            self.J3_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j2'))), 'f', 3))
            self.J4_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j3'))), 'f', 3))
            self.J5_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j4'))), 'f', 3))
            self.J6_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j5'))), 'f', 3))
            self.J7_read.setText(QtCore.QString.number(deg((self.th.limb.joint_angle('right_j6'))), 'f', 3))
            ### read_force_torque_info.     ### wrench = {'force':(x,y,z), 'torque':(x,y,z)}

            self.endpoint_ft_dict = self.th.limb.endpoint_effort()
            self.Fx_read.setText(QtCore.QString.number(self.endpoint_ft_dict['force'][0], 'f', 3))
            self.Fy_read.setText(QtCore.QString.number(self.endpoint_ft_dict['force'][1], 'f', 3))
            self.Fz_read.setText(QtCore.QString.number(self.endpoint_ft_dict['force'][2], 'f', 3))
            self.Mx_read6.setText(QtCore.QString.number(self.endpoint_ft_dict['torque'][0], 'f', 3))
            self.My_read.setText(QtCore.QString.number(self.endpoint_ft_dict['torque'][1], 'f', 3))
            self.Mz_read.setText(QtCore.QString.number(self.endpoint_ft_dict['torque'][2], 'f', 3))
            self.curFzValue = self.endpoint_ft_dict['force'][2]

            ### read_endpoint_pose_info.     ### wrench = {'position':(x,y,z), 'orientation':(x,y,z,w)}

            self.endpoint_pose = self.th.limb.endpoint_pose()
            self.endpoint_position = self.endpoint_pose['position']
            self.endpoint_orientation = self.endpoint_pose['orientation']

            self.rotationMat = PyKDL.Rotation.Quaternion(self.endpoint_orientation[0],
                                                         self.endpoint_orientation[1],
                                                         self.endpoint_orientation[2],
                                                         self.endpoint_orientation[3])

            self.endpoint_rotation = self.rotationMat.GetEulerZYX()
            # self.endpoint_rotation = euler.quat2euler([,
            #                                            self.endpoint_orientation[0],
            #                                            self.endpoint_orientation[1],
            #                                            self.endpoint_orientation[2]],
            #                                           'rzyx')


            self.X_read.setText(QtCore.QString.number(self.endpoint_position[0] * 1000.0, 'f', 3))
            self.My_read_2.setText(QtCore.QString.number(self.endpoint_position[1] * 1000.0, 'f', 3))
            self.Mz_read_2.setText(QtCore.QString.number(self.endpoint_position[2] * 1000.0, 'f', 3))
            ### Use tf3d to convert from quaternion to RPY
            # list (Rx,Ry,Rz)
            ### give the values to GUI
            self.Rx_read.setText(QtCore.QString.number(deg(self.endpoint_rotation[2]), 'f', 3))
            self.Ry_read.setText(QtCore.QString.number(deg(self.endpoint_rotation[1]), 'f', 3))
            self.Rz_read.setText(QtCore.QString.number(deg(self.endpoint_rotation[0]), 'f', 3))


    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1030, 951)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.RobotStatus_Group = QtGui.QGroupBox(self.centralwidget)
        self.RobotStatus_Group.setGeometry(QtCore.QRect(10, 20, 431, 421))
        self.RobotStatus_Group.setObjectName(_fromUtf8("RobotStatus_Group"))
        self.FT_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.FT_Label_read.setGeometry(QtCore.QRect(70, 50, 66, 17))
        self.FT_Label_read.setObjectName(_fromUtf8("FT_Label_read"))
        self.Pose_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Pose_Label_read.setGeometry(QtCore.QRect(250, 50, 81, 17))
        self.Pose_Label_read.setObjectName(_fromUtf8("Pose_Label_read"))
        self.Fx_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Fx_read.setGeometry(QtCore.QRect(50, 80, 104, 31))
        self.Fx_read.setReadOnly(True)
        self.Fx_read.setObjectName(_fromUtf8("Fx_read"))
        self.Fy_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Fy_read.setGeometry(QtCore.QRect(50, 120, 104, 31))
        self.Fy_read.setReadOnly(True)
        self.Fy_read.setObjectName(_fromUtf8("Fy_read"))
        self.Fz_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Fz_read.setGeometry(QtCore.QRect(50, 160, 104, 31))
        self.Fz_read.setReadOnly(True)
        self.Fz_read.setObjectName(_fromUtf8("Fz_read"))
        self.Mx_read6 = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Mx_read6.setGeometry(QtCore.QRect(50, 200, 104, 31))
        self.Mx_read6.setReadOnly(True)
        self.Mx_read6.setObjectName(_fromUtf8("Mx_read6"))
        self.My_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.My_read.setGeometry(QtCore.QRect(50, 240, 104, 31))
        self.My_read.setReadOnly(True)
        self.My_read.setObjectName(_fromUtf8("My_read"))
        self.Mz_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Mz_read.setGeometry(QtCore.QRect(50, 280, 104, 31))
        self.Mz_read.setReadOnly(True)
        self.Mz_read.setObjectName(_fromUtf8("Mz_read"))
        self.X_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.X_read.setGeometry(QtCore.QRect(230, 80, 104, 31))
        self.X_read.setReadOnly(True)
        self.X_read.setObjectName(_fromUtf8("X_read"))
        self.My_read_2 = QtGui.QTextEdit(self.RobotStatus_Group)
        self.My_read_2.setGeometry(QtCore.QRect(230, 120, 104, 31))
        self.My_read_2.setReadOnly(True)
        self.My_read_2.setObjectName(_fromUtf8("My_read_2"))
        self.Mz_read_2 = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Mz_read_2.setGeometry(QtCore.QRect(230, 160, 104, 31))
        self.Mz_read_2.setReadOnly(True)
        self.Mz_read_2.setObjectName(_fromUtf8("Mz_read_2"))
        self.Ry_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Ry_read.setGeometry(QtCore.QRect(230, 240, 104, 31))
        self.Ry_read.setReadOnly(True)
        self.Ry_read.setObjectName(_fromUtf8("Ry_read"))
        self.Rz_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Rz_read.setGeometry(QtCore.QRect(230, 280, 104, 31))
        self.Rz_read.setReadOnly(True)
        self.Rz_read.setObjectName(_fromUtf8("Rz_read"))
        self.Rx_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.Rx_read.setGeometry(QtCore.QRect(230, 200, 104, 31))
        self.Rx_read.setReadOnly(True)
        self.Rx_read.setObjectName(_fromUtf8("Rx_read"))
        self.X_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.X_Label_read.setGeometry(QtCore.QRect(210, 90, 21, 17))
        self.X_Label_read.setObjectName(_fromUtf8("X_Label_read"))
        self.Y_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Y_Label_read.setGeometry(QtCore.QRect(210, 130, 21, 17))
        self.Y_Label_read.setObjectName(_fromUtf8("Y_Label_read"))
        self.Z_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Z_Label_read.setGeometry(QtCore.QRect(210, 170, 21, 17))
        self.Z_Label_read.setObjectName(_fromUtf8("Z_Label_read"))
        self.Rx_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Rx_Label_read.setGeometry(QtCore.QRect(200, 210, 21, 17))
        self.Rx_Label_read.setObjectName(_fromUtf8("Rx_Label_read"))
        self.Ry_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Ry_Label_read.setGeometry(QtCore.QRect(200, 250, 21, 17))
        self.Ry_Label_read.setObjectName(_fromUtf8("Ry_Label_read"))
        self.Rz_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Rz_Label_read.setGeometry(QtCore.QRect(200, 290, 21, 17))
        self.Rz_Label_read.setObjectName(_fromUtf8("Rz_Label_read"))
        self.Fy_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Fy_Label_read.setGeometry(QtCore.QRect(20, 130, 21, 17))
        self.Fy_Label_read.setObjectName(_fromUtf8("Fy_Label_read"))
        self.My_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.My_Label_read.setGeometry(QtCore.QRect(20, 250, 21, 17))
        self.My_Label_read.setObjectName(_fromUtf8("My_Label_read"))
        self.Fx_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Fx_Label_read.setGeometry(QtCore.QRect(20, 90, 21, 16))
        self.Fx_Label_read.setObjectName(_fromUtf8("Fx_Label_read"))
        self.Mx_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Mx_Label_read.setGeometry(QtCore.QRect(20, 210, 21, 17))
        self.Mx_Label_read.setObjectName(_fromUtf8("Mx_Label_read"))
        self.Mz_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Mz_Label_read.setGeometry(QtCore.QRect(20, 290, 21, 16))
        self.Mz_Label_read.setObjectName(_fromUtf8("Mz_Label_read"))
        self.Fz_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Fz_Label_read.setGeometry(QtCore.QRect(20, 170, 21, 17))
        self.Fz_Label_read.setObjectName(_fromUtf8("Fz_Label_read"))
        self.F_Unit_Read = QtGui.QLabel(self.RobotStatus_Group)
        self.F_Unit_Read.setGeometry(QtCore.QRect(160, 90, 21, 16))
        self.F_Unit_Read.setObjectName(_fromUtf8("F_Unit_Read"))
        self.F_Unit_Read_2 = QtGui.QLabel(self.RobotStatus_Group)
        self.F_Unit_Read_2.setGeometry(QtCore.QRect(160, 130, 21, 16))
        self.F_Unit_Read_2.setObjectName(_fromUtf8("F_Unit_Read_2"))
        self.F_Unit_Read_3 = QtGui.QLabel(self.RobotStatus_Group)
        self.F_Unit_Read_3.setGeometry(QtCore.QRect(160, 170, 21, 16))
        self.F_Unit_Read_3.setObjectName(_fromUtf8("F_Unit_Read_3"))
        self.M_Unit_Read = QtGui.QLabel(self.RobotStatus_Group)
        self.M_Unit_Read.setGeometry(QtCore.QRect(160, 210, 31, 16))
        self.M_Unit_Read.setObjectName(_fromUtf8("M_Unit_Read"))
        self.M_Unit_Read_2 = QtGui.QLabel(self.RobotStatus_Group)
        self.M_Unit_Read_2.setGeometry(QtCore.QRect(160, 250, 31, 16))
        self.M_Unit_Read_2.setObjectName(_fromUtf8("M_Unit_Read_2"))
        self.M_Unit_Read_3 = QtGui.QLabel(self.RobotStatus_Group)
        self.M_Unit_Read_3.setGeometry(QtCore.QRect(160, 290, 31, 16))
        self.M_Unit_Read_3.setObjectName(_fromUtf8("M_Unit_Read_3"))
        self.X_Unit_read = QtGui.QLabel(self.RobotStatus_Group)
        self.X_Unit_read.setGeometry(QtCore.QRect(340, 90, 31, 16))
        self.X_Unit_read.setObjectName(_fromUtf8("X_Unit_read"))
        self.X_Unit_read_2 = QtGui.QLabel(self.RobotStatus_Group)
        self.X_Unit_read_2.setGeometry(QtCore.QRect(340, 130, 31, 16))
        self.X_Unit_read_2.setObjectName(_fromUtf8("X_Unit_read_2"))
        self.X_Unit_read_3 = QtGui.QLabel(self.RobotStatus_Group)
        self.X_Unit_read_3.setGeometry(QtCore.QRect(340, 170, 31, 16))
        self.X_Unit_read_3.setObjectName(_fromUtf8("X_Unit_read_3"))
        self.R_Unit_Read = QtGui.QLabel(self.RobotStatus_Group)
        self.R_Unit_Read.setGeometry(QtCore.QRect(340, 210, 31, 16))
        self.R_Unit_Read.setObjectName(_fromUtf8("R_Unit_Read"))
        self.R_Unit_Read_2 = QtGui.QLabel(self.RobotStatus_Group)
        self.R_Unit_Read_2.setGeometry(QtCore.QRect(340, 250, 31, 16))
        self.R_Unit_Read_2.setObjectName(_fromUtf8("R_Unit_Read_2"))
        self.R_Unit_Read_3 = QtGui.QLabel(self.RobotStatus_Group)
        self.R_Unit_Read_3.setGeometry(QtCore.QRect(340, 290, 31, 16))
        self.R_Unit_Read_3.setObjectName(_fromUtf8("R_Unit_Read_3"))
        self.J1_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J1_read.setGeometry(QtCore.QRect(40, 340, 71, 31))
        self.J1_read.setObjectName(_fromUtf8("J1_read"))
        self.J2_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J2_read.setGeometry(QtCore.QRect(140, 340, 71, 31))
        self.J2_read.setObjectName(_fromUtf8("J2_read"))
        self.J3_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J3_read.setGeometry(QtCore.QRect(240, 340, 71, 31))
        self.J3_read.setObjectName(_fromUtf8("J3_read"))
        self.J4_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J4_read.setGeometry(QtCore.QRect(340, 340, 71, 31))
        self.J4_read.setObjectName(_fromUtf8("J4_read"))
        self.J5_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J5_read.setGeometry(QtCore.QRect(40, 380, 71, 31))
        self.J5_read.setObjectName(_fromUtf8("J5_read"))
        self.J6_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J6_read.setGeometry(QtCore.QRect(140, 380, 71, 31))
        self.J6_read.setObjectName(_fromUtf8("J6_read"))
        self.J7_read = QtGui.QTextEdit(self.RobotStatus_Group)
        self.J7_read.setGeometry(QtCore.QRect(240, 380, 71, 31))
        self.J7_read.setObjectName(_fromUtf8("J7_read"))
        self.J1_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J1_Read_Label.setGeometry(QtCore.QRect(20, 350, 21, 17))
        self.J1_Read_Label.setObjectName(_fromUtf8("J1_Read_Label"))
        self.J2_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J2_Read_Label.setGeometry(QtCore.QRect(120, 350, 21, 17))
        self.J2_Read_Label.setObjectName(_fromUtf8("J2_Read_Label"))
        self.J3_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J3_Read_Label.setGeometry(QtCore.QRect(220, 350, 21, 17))
        self.J3_Read_Label.setObjectName(_fromUtf8("J3_Read_Label"))
        self.J4_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J4_Read_Label.setGeometry(QtCore.QRect(320, 350, 21, 17))
        self.J4_Read_Label.setObjectName(_fromUtf8("J4_Read_Label"))
        self.J5_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J5_Read_Label.setGeometry(QtCore.QRect(20, 390, 21, 16))
        self.J5_Read_Label.setObjectName(_fromUtf8("J5_Read_Label"))
        self.Joint_Label_read = QtGui.QLabel(self.RobotStatus_Group)
        self.Joint_Label_read.setGeometry(QtCore.QRect(150, 320, 131, 17))
        self.Joint_Label_read.setObjectName(_fromUtf8("Joint_Label_read"))
        self.J6_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J6_Read_Label.setGeometry(QtCore.QRect(120, 390, 21, 17))
        self.J6_Read_Label.setObjectName(_fromUtf8("J6_Read_Label"))
        self.J7_Read_Label = QtGui.QLabel(self.RobotStatus_Group)
        self.J7_Read_Label.setGeometry(QtCore.QRect(220, 390, 21, 17))
        self.J7_Read_Label.setObjectName(_fromUtf8("J7_Read_Label"))
        self.Button_Init_Node = QtGui.QPushButton(self.centralwidget)
        self.Button_Init_Node.setGeometry(QtCore.QRect(30, 820, 141, 71))
        self.Button_Init_Node.setObjectName(_fromUtf8("Button_Init_Node"))
        self.Button_Init_Node.setCheckable(True)

        self.button_pause_resume = QtGui.QPushButton(self.centralwidget)
        self.button_pause_resume.setGeometry(QtCore.QRect(190, 820, 141, 71))
        self.button_pause_resume.setObjectName(_fromUtf8("button_pause_resume"))
        self.objMonitorGroup = QtGui.QGroupBox(self.centralwidget)
        self.objMonitorGroup.setGeometry(QtCore.QRect(20, 470, 581, 201))
        self.objMonitorGroup.setObjectName(_fromUtf8("objMonitorGroup"))
        self.TorqueLabel = QtGui.QLabel(self.objMonitorGroup)
        self.TorqueLabel.setGeometry(QtCore.QRect(20, 50, 51, 20))
        self.TorqueLabel.setObjectName(_fromUtf8("TorqueLabel"))
        self.PositionLabel = QtGui.QLabel(self.objMonitorGroup)
        self.PositionLabel.setGeometry(QtCore.QRect(20, 110, 51, 20))
        self.PositionLabel.setObjectName(_fromUtf8("PositionLabel"))
        self.J1 = QtGui.QLabel(self.objMonitorGroup)
        self.J1.setGeometry(QtCore.QRect(100, 20, 61, 20))
        self.J1.setObjectName(_fromUtf8("J1"))
        self.J2 = QtGui.QLabel(self.objMonitorGroup)
        self.J2.setGeometry(QtCore.QRect(180, 20, 81, 20))
        self.J2.setObjectName(_fromUtf8("J2"))
        self.J3 = QtGui.QLabel(self.objMonitorGroup)
        self.J3.setGeometry(QtCore.QRect(280, 20, 91, 20))
        self.J3.setObjectName(_fromUtf8("J3"))
        self.J4 = QtGui.QLabel(self.objMonitorGroup)
        self.J4.setGeometry(QtCore.QRect(390, 20, 71, 20))
        self.J4.setObjectName(_fromUtf8("J4"))
        self.J5 = QtGui.QLabel(self.objMonitorGroup)
        self.J5.setGeometry(QtCore.QRect(480, 20, 91, 20))
        self.J5.setObjectName(_fromUtf8("J5"))
        self.Velocity_Label = QtGui.QLabel(self.objMonitorGroup)
        self.Velocity_Label.setGeometry(QtCore.QRect(20, 160, 51, 20))
        self.Velocity_Label.setObjectName(_fromUtf8("Velocity_Label"))
        self.cantataBoxA = QtGui.QTextEdit(self.objMonitorGroup)
        self.cantataBoxA.setGeometry(QtCore.QRect(90, 50, 71, 31))
        self.cantataBoxA.setObjectName(_fromUtf8("cantataBoxA"))
        self.chocoBoxA = QtGui.QTextEdit(self.objMonitorGroup)
        self.chocoBoxA.setGeometry(QtCore.QRect(180, 50, 71, 31))
        self.chocoBoxA.setObjectName(_fromUtf8("chocoBoxA"))
        self.doleBoxA = QtGui.QTextEdit(self.objMonitorGroup)
        self.doleBoxA.setGeometry(QtCore.QRect(280, 50, 71, 31))
        self.doleBoxA.setObjectName(_fromUtf8("doleBoxA"))
        self.starBoxA = QtGui.QTextEdit(self.objMonitorGroup)
        self.starBoxA.setGeometry(QtCore.QRect(380, 50, 71, 31))
        self.starBoxA.setObjectName(_fromUtf8("starBoxA"))
        self.danonBoxA = QtGui.QTextEdit(self.objMonitorGroup)
        self.danonBoxA.setGeometry(QtCore.QRect(480, 50, 71, 31))
        self.danonBoxA.setObjectName(_fromUtf8("danonBoxA"))
        self.chocoBoxB = QtGui.QTextEdit(self.objMonitorGroup)
        self.chocoBoxB.setGeometry(QtCore.QRect(180, 100, 71, 31))
        self.chocoBoxB.setObjectName(_fromUtf8("chocoBoxB"))
        self.starBoxB = QtGui.QTextEdit(self.objMonitorGroup)
        self.starBoxB.setGeometry(QtCore.QRect(380, 100, 71, 31))
        self.starBoxB.setObjectName(_fromUtf8("starBoxB"))
        self.doleBoxB = QtGui.QTextEdit(self.objMonitorGroup)
        self.doleBoxB.setGeometry(QtCore.QRect(280, 100, 71, 31))
        self.doleBoxB.setObjectName(_fromUtf8("doleBoxB"))
        self.cantataBoxB = QtGui.QTextEdit(self.objMonitorGroup)
        self.cantataBoxB.setGeometry(QtCore.QRect(90, 100, 71, 31))
        self.cantataBoxB.setObjectName(_fromUtf8("cantataBoxB"))
        self.danonBoxB = QtGui.QTextEdit(self.objMonitorGroup)
        self.danonBoxB.setGeometry(QtCore.QRect(480, 100, 71, 31))
        self.danonBoxB.setObjectName(_fromUtf8("danonBoxB"))
        self.chocoTotal = QtGui.QTextEdit(self.objMonitorGroup)
        self.chocoTotal.setGeometry(QtCore.QRect(180, 150, 71, 31))
        self.chocoTotal.setObjectName(_fromUtf8("chocoTotal"))
        self.starTotal = QtGui.QTextEdit(self.objMonitorGroup)
        self.starTotal.setGeometry(QtCore.QRect(380, 150, 71, 31))
        self.starTotal.setObjectName(_fromUtf8("starTotal"))
        self.doleTotal = QtGui.QTextEdit(self.objMonitorGroup)
        self.doleTotal.setGeometry(QtCore.QRect(280, 150, 71, 31))
        self.doleTotal.setObjectName(_fromUtf8("doleTotal"))
        self.cantataTotal = QtGui.QTextEdit(self.objMonitorGroup)
        self.cantataTotal.setGeometry(QtCore.QRect(90, 150, 71, 31))
        self.cantataTotal.setObjectName(_fromUtf8("cantataTotal"))
        self.danonTotal = QtGui.QTextEdit(self.objMonitorGroup)
        self.danonTotal.setGeometry(QtCore.QRect(480, 150, 71, 31))
        self.danonTotal.setObjectName(_fromUtf8("danonTotal"))
        self.CommandRobot_Group = QtGui.QGroupBox(self.centralwidget)
        self.CommandRobot_Group.setGeometry(QtCore.QRect(470, 20, 181, 421))
        self.CommandRobot_Group.setObjectName(_fromUtf8("CommandRobot_Group"))
        self.X_command = QtGui.QTextEdit(self.CommandRobot_Group)
        self.X_command.setGeometry(QtCore.QRect(30, 80, 104, 31))
        self.X_command.setReadOnly(False)
        self.X_command.setObjectName(_fromUtf8("X_command"))
        self.Ry_Label_command = QtGui.QLabel(self.CommandRobot_Group)
        self.Ry_Label_command.setGeometry(QtCore.QRect(0, 250, 21, 17))
        self.Ry_Label_command.setObjectName(_fromUtf8("Ry_Label_command"))
        self.Y_Label_command = QtGui.QLabel(self.CommandRobot_Group)
        self.Y_Label_command.setGeometry(QtCore.QRect(0, 130, 21, 17))
        self.Y_Label_command.setObjectName(_fromUtf8("Y_Label_command"))
        self.Z_command = QtGui.QTextEdit(self.CommandRobot_Group)
        self.Z_command.setGeometry(QtCore.QRect(30, 160, 104, 31))
        self.Z_command.setReadOnly(False)
        self.Z_command.setObjectName(_fromUtf8("Z_command"))
        self.Y_command = QtGui.QTextEdit(self.CommandRobot_Group)
        self.Y_command.setGeometry(QtCore.QRect(30, 120, 104, 31))
        self.Y_command.setReadOnly(False)
        self.Y_command.setObjectName(_fromUtf8("Y_command"))
        self.Rz_command = QtGui.QTextEdit(self.CommandRobot_Group)
        self.Rz_command.setGeometry(QtCore.QRect(30, 280, 104, 31))
        self.Rz_command.setReadOnly(False)
        self.Rz_command.setObjectName(_fromUtf8("Rz_command"))
        self.X_Label_command = QtGui.QLabel(self.CommandRobot_Group)
        self.X_Label_command.setGeometry(QtCore.QRect(0, 90, 21, 17))
        self.X_Label_command.setObjectName(_fromUtf8("X_Label_command"))
        self.Ry_command = QtGui.QTextEdit(self.CommandRobot_Group)
        self.Ry_command.setGeometry(QtCore.QRect(30, 240, 104, 31))
        self.Ry_command.setReadOnly(False)
        self.Ry_command.setObjectName(_fromUtf8("Ry_command"))
        self.Rx_command = QtGui.QTextEdit(self.CommandRobot_Group)
        self.Rx_command.setGeometry(QtCore.QRect(30, 200, 104, 31))
        self.Rx_command.setReadOnly(False)
        self.Rx_command.setObjectName(_fromUtf8("Rx_command"))
        self.Rx_Label_command = QtGui.QLabel(self.CommandRobot_Group)
        self.Rx_Label_command.setGeometry(QtCore.QRect(0, 210, 21, 17))
        self.Rx_Label_command.setObjectName(_fromUtf8("Rx_Label_command"))
        self.Rz_Label_command = QtGui.QLabel(self.CommandRobot_Group)
        self.Rz_Label_command.setGeometry(QtCore.QRect(0, 290, 21, 16))
        self.Rz_Label_command.setObjectName(_fromUtf8("Rz_Label_command"))
        self.Z_Label_command = QtGui.QLabel(self.CommandRobot_Group)
        self.Z_Label_command.setGeometry(QtCore.QRect(0, 170, 21, 17))
        self.Z_Label_command.setObjectName(_fromUtf8("Z_Label_command"))
        self.Update_Button = QtGui.QPushButton(self.CommandRobot_Group)
        self.Update_Button.setGeometry(QtCore.QRect(40, 350, 98, 27))
        self.Update_Button.setObjectName(_fromUtf8("Update_Button"))
        self.Retrieve_Button = QtGui.QPushButton(self.CommandRobot_Group)
        self.Retrieve_Button.setGeometry(QtCore.QRect(40, 380, 98, 27))
        self.Retrieve_Button.setObjectName(_fromUtf8("Retrieve_Button"))
        self.X_Unit_Command = QtGui.QLabel(self.CommandRobot_Group)
        self.X_Unit_Command.setGeometry(QtCore.QRect(140, 90, 31, 16))
        self.X_Unit_Command.setObjectName(_fromUtf8("X_Unit_Command"))
        self.X_Unit_Command_2 = QtGui.QLabel(self.CommandRobot_Group)
        self.X_Unit_Command_2.setGeometry(QtCore.QRect(140, 130, 31, 16))
        self.X_Unit_Command_2.setObjectName(_fromUtf8("X_Unit_Command_2"))
        self.X_Unit_Command_3 = QtGui.QLabel(self.CommandRobot_Group)
        self.X_Unit_Command_3.setGeometry(QtCore.QRect(140, 170, 31, 16))
        self.X_Unit_Command_3.setObjectName(_fromUtf8("X_Unit_Command_3"))
        self.R_Unit_Command = QtGui.QLabel(self.CommandRobot_Group)
        self.R_Unit_Command.setGeometry(QtCore.QRect(140, 210, 31, 16))
        self.R_Unit_Command.setObjectName(_fromUtf8("R_Unit_Command"))
        self.R_Unit_Command_2 = QtGui.QLabel(self.CommandRobot_Group)
        self.R_Unit_Command_2.setGeometry(QtCore.QRect(140, 250, 31, 16))
        self.R_Unit_Command_2.setObjectName(_fromUtf8("R_Unit_Command_2"))
        self.R_Unit_Command_3 = QtGui.QLabel(self.CommandRobot_Group)
        self.R_Unit_Command_3.setGeometry(QtCore.QRect(140, 290, 31, 16))
        self.R_Unit_Command_3.setObjectName(_fromUtf8("R_Unit_Command_3"))
        self.Pose_Label_Command = QtGui.QLabel(self.CommandRobot_Group)
        self.Pose_Label_Command.setGeometry(QtCore.QRect(10, 30, 81, 17))
        self.Pose_Label_Command.setObjectName(_fromUtf8("Pose_Label_Command"))
        self.read_cur_pose_button = QtGui.QPushButton(self.CommandRobot_Group)
        self.read_cur_pose_button.setGeometry(QtCore.QRect(30, 320, 111, 27))
        self.read_cur_pose_button.setObjectName(_fromUtf8("read_cur_pose_button"))
        self.LogText = QtGui.QTextEdit(self.centralwidget)
        self.LogText.setGeometry(QtCore.QRect(30, 700, 311, 101))
        self.LogText.setObjectName(_fromUtf8("LogText"))
        self.demoGroup = QtGui.QGroupBox(self.centralwidget)
        self.demoGroup.setGeometry(QtCore.QRect(690, 20, 321, 451))
        self.demoGroup.setObjectName(_fromUtf8("demoGroup"))
        self.pickCantata = QtGui.QPushButton(self.demoGroup)
        self.pickCantata.setGeometry(QtCore.QRect(20, 100, 91, 51))
        self.pickCantata.setObjectName(_fromUtf8("pickCantata"))
        self.pickStarbucks = QtGui.QPushButton(self.demoGroup)
        self.pickStarbucks.setGeometry(QtCore.QRect(20, 160, 91, 51))
        self.pickStarbucks.setObjectName(_fromUtf8("pickStarbucks"))
        self.pickChocoemon = QtGui.QPushButton(self.demoGroup)
        self.pickChocoemon.setGeometry(QtCore.QRect(120, 100, 91, 51))
        self.pickChocoemon.setObjectName(_fromUtf8("pickChocoemon"))
        self.pickDanonGreek = QtGui.QPushButton(self.demoGroup)
        self.pickDanonGreek.setGeometry(QtCore.QRect(120, 160, 101, 51))
        self.pickDanonGreek.setObjectName(_fromUtf8("pickDanonGreek"))
        self.pickDoleMango = QtGui.QPushButton(self.demoGroup)
        self.pickDoleMango.setGeometry(QtCore.QRect(220, 100, 91, 51))
        self.pickDoleMango.setObjectName(_fromUtf8("pickDoleMango"))
        self.cur_dir_label = QtGui.QLabel(self.demoGroup)
        self.cur_dir_label.setGeometry(QtCore.QRect(10, 20, 261, 21))
        self.cur_dir_label.setObjectName(_fromUtf8("cur_dir_label"))
        self.currentDir_text = QtGui.QTextEdit(self.demoGroup)
        self.currentDir_text.setGeometry(QtCore.QRect(40, 50, 151, 31))
        self.currentDir_text.setObjectName(_fromUtf8("currentDir_text"))
        self.switchAtoB_BtoA = QtGui.QPushButton(self.demoGroup)
        self.switchAtoB_BtoA.setGeometry(QtCore.QRect(210, 50, 98, 27))
        self.switchAtoB_BtoA.setObjectName(_fromUtf8("switchAtoB_BtoA"))
        self.Accel_traj_2 = QtGui.QLabel(self.demoGroup)
        self.Accel_traj_2.setGeometry(QtCore.QRect(10, 220, 171, 21))
        self.Accel_traj_2.setObjectName(_fromUtf8("Accel_traj_2"))
        self.curPallet_order = QtGui.QTextEdit(self.demoGroup)
        self.curPallet_order.setGeometry(QtCore.QRect(190, 220, 51, 31))
        self.curPallet_order.setObjectName(_fromUtf8("curPallet_order"))
        self.init_pallet_order = QtGui.QPushButton(self.demoGroup)
        self.init_pallet_order.setGeometry(QtCore.QRect(250, 220, 61, 27))
        self.init_pallet_order.setObjectName(_fromUtf8("init_pallet_order"))
        self.Accel_traj_3 = QtGui.QLabel(self.demoGroup)
        self.Accel_traj_3.setGeometry(QtCore.QRect(10, 250, 141, 21))
        self.Accel_traj_3.setObjectName(_fromUtf8("Accel_traj_3"))
        self.cur_objInfo = QtGui.QTextEdit(self.demoGroup)
        self.cur_objInfo.setGeometry(QtCore.QRect(0, 280, 311, 101))
        self.cur_objInfo.setObjectName(_fromUtf8("cur_objInfo"))
        self.randomAtoB = QtGui.QPushButton(self.demoGroup)
        self.randomAtoB.setGeometry(QtCore.QRect(110, 390, 91, 51))
        self.randomAtoB.setObjectName(_fromUtf8("randomAtoB"))
        self.randomBtoA = QtGui.QPushButton(self.demoGroup)
        self.randomBtoA.setGeometry(QtCore.QRect(220, 390, 91, 51))
        self.randomBtoA.setObjectName(_fromUtf8("randomBtoA"))
        self.Accel_traj_4 = QtGui.QLabel(self.demoGroup)
        self.Accel_traj_4.setGeometry(QtCore.QRect(10, 390, 91, 51))
        self.Accel_traj_4.setObjectName(_fromUtf8("Accel_traj_4"))
        self.line = QtGui.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(432, 20, 21, 431))
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.line_2 = QtGui.QFrame(self.centralwidget)
        self.line_2.setGeometry(QtCore.QRect(660, 20, 21, 431))
        self.line_2.setFrameShape(QtGui.QFrame.VLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.line_3 = QtGui.QFrame(self.centralwidget)
        self.line_3.setGeometry(QtCore.QRect(1020, 20, 21, 431))
        self.line_3.setFrameShape(QtGui.QFrame.VLine)
        self.line_3.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_3.setObjectName(_fromUtf8("line_3"))
        self.line_4 = QtGui.QFrame(self.centralwidget)
        self.line_4.setGeometry(QtCore.QRect(27, 460, 641, 20))
        self.line_4.setFrameShape(QtGui.QFrame.HLine)
        self.line_4.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_4.setObjectName(_fromUtf8("line_4"))
        self.line_5 = QtGui.QFrame(self.centralwidget)
        self.line_5.setGeometry(QtCore.QRect(30, 660, 571, 20))
        self.line_5.setFrameShape(QtGui.QFrame.HLine)
        self.line_5.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_5.setObjectName(_fromUtf8("line_5"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(40, 680, 111, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.paramCalibGroup = QtGui.QGroupBox(self.centralwidget)
        self.paramCalibGroup.setGeometry(QtCore.QRect(600, 480, 361, 171))
        self.paramCalibGroup.setObjectName(_fromUtf8("paramCalibGroup"))
        self.caibBoxA = QtGui.QPushButton(self.paramCalibGroup)
        self.caibBoxA.setGeometry(QtCore.QRect(10, 50, 111, 31))
        self.caibBoxA.setObjectName(_fromUtf8("caibBoxA"))
        self.calibBoxB = QtGui.QPushButton(self.paramCalibGroup)
        self.calibBoxB.setGeometry(QtCore.QRect(10, 90, 111, 31))
        self.calibBoxB.setObjectName(_fromUtf8("calibBoxB"))
        self.Accel_traj_5 = QtGui.QLabel(self.paramCalibGroup)
        self.Accel_traj_5.setGeometry(QtCore.QRect(40, 20, 271, 21))
        self.Accel_traj_5.setObjectName(_fromUtf8("Accel_traj_5"))
        self.boxA_Xoffset = QtGui.QTextEdit(self.paramCalibGroup)
        self.boxA_Xoffset.setGeometry(QtCore.QRect(160, 50, 71, 31))
        self.boxA_Xoffset.setObjectName(_fromUtf8("boxA_Xoffset"))
        self.boxA_Yoffset = QtGui.QTextEdit(self.paramCalibGroup)
        self.boxA_Yoffset.setGeometry(QtCore.QRect(250, 50, 71, 31))
        self.boxA_Yoffset.setObjectName(_fromUtf8("boxA_Yoffset"))
        self.boxB_Xoffset = QtGui.QTextEdit(self.paramCalibGroup)
        self.boxB_Xoffset.setGeometry(QtCore.QRect(160, 90, 71, 31))
        self.boxB_Xoffset.setObjectName(_fromUtf8("boxB_Xoffset"))
        self.boxB_Yoffset = QtGui.QTextEdit(self.paramCalibGroup)
        self.boxB_Yoffset.setGeometry(QtCore.QRect(250, 90, 71, 31))
        self.boxB_Yoffset.setObjectName(_fromUtf8("boxB_Yoffset"))
        self.Accel_traj_6 = QtGui.QLabel(self.paramCalibGroup)
        self.Accel_traj_6.setGeometry(QtCore.QRect(20, 140, 101, 21))
        self.Accel_traj_6.setObjectName(_fromUtf8("Accel_traj_6"))
        self.gripperLength = QtGui.QTextEdit(self.paramCalibGroup)
        self.gripperLength.setGeometry(QtCore.QRect(160, 140, 71, 31))
        self.gripperLength.setObjectName(_fromUtf8("gripperLength"))
        self.Accel_traj_7 = QtGui.QLabel(self.paramCalibGroup)
        self.Accel_traj_7.setGeometry(QtCore.QRect(240, 150, 31, 21))
        self.Accel_traj_7.setObjectName(_fromUtf8("Accel_traj_7"))
        self.Accel_traj_8 = QtGui.QLabel(self.paramCalibGroup)
        self.Accel_traj_8.setGeometry(QtCore.QRect(330, 60, 31, 21))
        self.Accel_traj_8.setObjectName(_fromUtf8("Accel_traj_8"))
        self.Accel_traj_9 = QtGui.QLabel(self.paramCalibGroup)
        self.Accel_traj_9.setGeometry(QtCore.QRect(330, 100, 31, 21))
        self.Accel_traj_9.setObjectName(_fromUtf8("Accel_traj_9"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(350, 680, 51, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(650, 680, 51, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1030, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.Button_Init_Node.clicked.connect(self.robot_Button_Clicked)
        self.button_pause_resume.clicked.connect(self.command_UR5)
        self.switchAtoB_BtoA.clicked.connect(self.switchMode)
        self.Update_Button.clicked.connect(self.update_command_clicked)
        self.Retrieve_Button.clicked.connect(self.retrieve_command_clicked)
        self.read_cur_pose_button.clicked.connect(self.read_cur_pose_clicked)
        ######################################################################
        self.init_pallet_order.clicked.connect(self.init_palletizing_num)
        ######################################################################
        self.randomBtoA.clicked.connect(self.pickplace_BtoA)
        self.randomAtoB.clicked.connect(self.pickplace_AtoB)
        ######################################################################
        self.pickChocoemon.clicked.connect(self.pickChocoemonBtn)
        self.pickDanonGreek.clicked.connect(self.pickDanonGreekBtn)
        self.pickDoleMango.clicked.connect(self.pickDoleMangoBtn)
        self.pickStarbucks.clicked.connect(self.pickStarbucksBtn)
        self.pickCantata.clicked.connect(self.pickCantataBtn)
        ######################################################################
        self.caibBoxA.clicked.connect(self.onClickedCalibBoxA)
        self.calibBoxB.clicked.connect(self.onClickedCalibBoxB)


        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8(base_dir+'/choco.jpg')),QtGui.QIcon.Normal, QtGui.QIcon.On)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(_fromUtf8(base_dir+'/cantata.jpeg')),QtGui.QIcon.Normal, QtGui.QIcon.On)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(_fromUtf8(base_dir + '/starbucks.jpg')), QtGui.QIcon.Normal, QtGui.QIcon.On)
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(_fromUtf8(base_dir + '/danone.jpeg')), QtGui.QIcon.Normal, QtGui.QIcon.On)
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(_fromUtf8(base_dir + '/dole.jpeg')), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.pickChocoemon.setIcon(icon1)
        self.pickChocoemon.setIconSize(QSize(42,42))
        self.pickCantata.setIcon(icon2)
        self.pickCantata.setIconSize(QSize(42,42))
        self.pickStarbucks.setIcon(icon3)
        self.pickStarbucks.setIconSize(QSize(42,42))
        self.pickDanonGreek.setIcon(icon4)
        self.pickDanonGreek.setIconSize(QSize(42,42))
        self.pickDoleMango.setIcon(icon5)
        self.pickDoleMango.setIconSize(QSize(42,42))

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "DeepRL_Sawyer", None))
        self.RobotStatus_Group.setTitle(_translate("MainWindow", "Robot Status (read from Sawyer)", None))
        self.FT_Label_read.setText(_translate("MainWindow", "F/T info.", None))
        self.Pose_Label_read.setText(_translate("MainWindow", "Pose info.", None))
        self.Fx_read.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.X_Label_read.setText(_translate("MainWindow", "X", None))
        self.Y_Label_read.setText(_translate("MainWindow", "Y", None))
        self.Z_Label_read.setText(_translate("MainWindow", "Z", None))
        self.Rx_Label_read.setText(_translate("MainWindow", "Rx", None))
        self.Ry_Label_read.setText(_translate("MainWindow", "Ry", None))
        self.Rz_Label_read.setText(_translate("MainWindow", "Rz", None))
        self.Fy_Label_read.setText(_translate("MainWindow", "Fy", None))
        self.My_Label_read.setText(_translate("MainWindow", "My", None))
        self.Fx_Label_read.setText(_translate("MainWindow", "Fx", None))
        self.Mx_Label_read.setText(_translate("MainWindow", "Mx", None))
        self.Mz_Label_read.setText(_translate("MainWindow", "Mz", None))
        self.Fz_Label_read.setText(_translate("MainWindow", "Fz", None))
        self.F_Unit_Read.setText(_translate("MainWindow", "N", None))
        self.F_Unit_Read_2.setText(_translate("MainWindow", "N", None))
        self.F_Unit_Read_3.setText(_translate("MainWindow", "N", None))
        self.M_Unit_Read.setText(_translate("MainWindow", "Nm", None))
        self.M_Unit_Read_2.setText(_translate("MainWindow", "Nm", None))
        self.M_Unit_Read_3.setText(_translate("MainWindow", "Nm", None))
        self.X_Unit_read.setText(_translate("MainWindow", "mm", None))
        self.X_Unit_read_2.setText(_translate("MainWindow", "mm", None))
        self.X_Unit_read_3.setText(_translate("MainWindow", "mm", None))
        self.R_Unit_Read.setText(_translate("MainWindow", "deg", None))
        self.R_Unit_Read_2.setText(_translate("MainWindow", "deg", None))
        self.R_Unit_Read_3.setText(_translate("MainWindow", "deg", None))
        self.J1_Read_Label.setText(_translate("MainWindow", "J1", None))
        self.J2_Read_Label.setText(_translate("MainWindow", "J2", None))
        self.J3_Read_Label.setText(_translate("MainWindow", "J3", None))
        self.J4_Read_Label.setText(_translate("MainWindow", "J4", None))
        self.J5_Read_Label.setText(_translate("MainWindow", "J5", None))
        self.Joint_Label_read.setText(_translate("MainWindow", "Joint Position (deg)", None))
        self.J6_Read_Label.setText(_translate("MainWindow", "J6", None))
        self.J7_Read_Label.setText(_translate("MainWindow", "J7", None))
        self.Button_Init_Node.setText(_translate("MainWindow", "Enable / Disable \n"
" Robot", None))
        self.button_pause_resume.setText(_translate("MainWindow", "Command UR5", None))
        self.objMonitorGroup.setTitle(_translate("MainWindow", "Object monitor", None))
        self.TorqueLabel.setText(_translate("MainWindow", "BoxA", None))
        self.PositionLabel.setText(_translate("MainWindow", "BoxB", None))
        self.J1.setText(_translate("MainWindow", "Cantata", None))
        self.J2.setText(_translate("MainWindow", "Chocoemon", None))
        self.J3.setText(_translate("MainWindow", "DoleMango", None))
        self.J4.setText(_translate("MainWindow", "Starbucks", None))
        self.J5.setText(_translate("MainWindow", "DanonGreek", None))
        self.Velocity_Label.setText(_translate("MainWindow", "Total", None))
        self.CommandRobot_Group.setTitle(_translate("MainWindow", "Commands to robot ", None))
        self.Ry_Label_command.setText(_translate("MainWindow", "Ry", None))
        self.Y_Label_command.setText(_translate("MainWindow", "Y", None))
        self.X_Label_command.setText(_translate("MainWindow", "X", None))
        self.Rx_Label_command.setText(_translate("MainWindow", "Rx", None))
        self.Rz_Label_command.setText(_translate("MainWindow", "Rz", None))
        self.Z_Label_command.setText(_translate("MainWindow", "Z", None))
        self.Update_Button.setText(_translate("MainWindow", "Update", None))
        self.Retrieve_Button.setText(_translate("MainWindow", "Retrieve", None))
        self.X_Unit_Command.setText(_translate("MainWindow", "mm", None))
        self.X_Unit_Command_2.setText(_translate("MainWindow", "mm", None))
        self.X_Unit_Command_3.setText(_translate("MainWindow", "mm", None))
        self.R_Unit_Command.setText(_translate("MainWindow", "deg", None))
        self.R_Unit_Command_2.setText(_translate("MainWindow", "deg", None))
        self.R_Unit_Command_3.setText(_translate("MainWindow", "deg", None))
        self.Pose_Label_Command.setText(_translate("MainWindow", "Pose info.", None))
        self.read_cur_pose_button.setText(_translate("MainWindow", "Read Cur. Pose", None))
        self.demoGroup.setTitle(_translate("MainWindow", "Pick and Place demo", None))
        #         self.pickCantata.setText(_translate("MainWindow", "Pick \n"
        # "Cantata", None))
        #         self.pickStarbucks.setText(_translate("MainWindow", "Pick\n"
        # "  Starbucks", None))
        #         self.pickChocoemon.setText(_translate("MainWindow", "Pick \n"
        # " Chocoemon", None))
        #         self.pickDanonGreek.setText(_translate("MainWindow", "Pick \n"
        # " Danon Greek", None))
        #         self.pickDoleMango.setText(_translate("MainWindow", "Pick \n"
        # " Dole Mango", None))
        self.cur_dir_label.setText(_translate("MainWindow", "Current pick&palletizing direction", None))
        self.switchAtoB_BtoA.setText(_translate("MainWindow", "Switch", None))
        self.Accel_traj_2.setText(_translate("MainWindow", "Current Palletizing order:", None))
        self.init_pallet_order.setText(_translate("MainWindow", "Init.", None))
        self.Accel_traj_3.setText(_translate("MainWindow", "Current object info.", None))
        self.randomAtoB.setText(_translate("MainWindow", "A -> B", None))
        self.randomBtoA.setText(_translate("MainWindow", "B -> A", None))
        self.Accel_traj_4.setText(_translate("MainWindow", "Randomized\n"
"palletizing", None))
        self.label.setText(_translate("MainWindow", "Log messages", None))
        self.paramCalibGroup.setTitle(_translate("MainWindow", "Parameter calibrator", None))
        self.caibBoxA.setText(_translate("MainWindow", "Set Sawyer Waypoints", None))
        self.calibBoxB.setText(_translate("MainWindow", "Calibrate boxB", None))
        self.Accel_traj_5.setText(_translate("MainWindow", "box offset:                          X                Y", None))
        self.Accel_traj_6.setText(_translate("MainWindow", "gripper length:", None))
        self.Accel_traj_7.setText(_translate("MainWindow", " mm", None))
        self.Accel_traj_8.setText(_translate("MainWindow", " mm", None))
        self.Accel_traj_9.setText(_translate("MainWindow", " mm", None))
        self.label_2.setText(_translate("MainWindow", "BoxA", None))
        self.label_3.setText(_translate("MainWindow", "BoxB", None))


if __name__ == "__main__":
    isEnabled = False
    isAtNeutral = False



    import sys

    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    timer = QtCore.QTimer()
    timer.timeout.connect(ui.read_status_from_Intera)
    timer.start(100)
    sys.exit(app.exec_())