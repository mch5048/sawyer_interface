#!/usr/bin/python

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Integ_UI.ui'
#
# Created: Sat Apr 14 16:45:16 2018
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import QElapsedTimer
from PyQt4.QtCore import QThread
from PyQt4.QtCore import SIGNAL
from PyQt4 import QtCore
from time import sleep

import numpy as np
import matplotlib.pyplot as plt
import scipy.misc
#
# from skimage import filter


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

import urx  # control UR5
import roslib; roslib.load_manifest('ur_driver')
import roslib
# roslib.load_manifest('robotiq_c_model_control')
# from robotiq_c_model_control.msg import CModel_robot_output as outputMsg
# from robotiq_c_model_control.msg import CModel_robot_input as inputMsg
import time
from math import atan
from math import pi
import rospy
import numpy as np
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from object_detection_yolov2.msg import *# import custom messages

import transforms3d
from transforms3d import euler
from transforms3d.derivations.eulerangles import *
from sensor_msgs.msg import Image as Image_msg
import cv2
from cv_bridge import CvBridge
# from berkeley_sawyer.srv import *
from intera_motion_interface import motion_waypoint
from intera_motion_interface import motion_trajectory
# from ros_faster_rcnn.msg import * # Detection, DetectionArray, DetectionFull
from new_sawyer_interface.msg import *
from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from std_msgs.msg import *
from std_srvs.srv import *
import math
from math import degrees as deg
from math import radians as rad
import PyKDL
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
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
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
from math import degrees as deg
import intera_interface
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
from operator import itemgetter
from intera_interface import Limb
from intera_interface import Cuff
import random
from random import randint

from robotiq_85_msgs.msg import GripperCmd, GripperStat

from new_sawyer_interface.msg import Cmd
from new_sawyer_interface.srv import *

from ros_grasp_detection.msg import *


from mask_rcnn_ros.msg import *

from std_msgs.msg import Float32
CS = False
JS = True
k=0
isEnabled = False
isTimer2_ok = False
isAtNeutral = False
hasRecorded = False
isFinished = False
isGripperOpen = True
globalSaveCounter = 0
numOfwaypoints = 0
numDemo = 0

import sys
sys.path.insert(1, '../include')

import pyrealsense2 as rs
from sensor_msgs.msg import Image, PointCloud2, PointField

RUNNING = False
IMAGE = Image()
DUMMY_FIELD_PREFIX = '__'


import pandas as pd
from sklearn.decomposition import PCA





import urx
reload(sys)
sys.setdefaultencoding('utf8')
import os
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
                                              max_rotational_speed=3.5,
                                              max_rotational_accel=3.5,
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

        # Publisher
        self.robotiqPub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
        QThread.__init__(self)
        # Subscriber
        rospy.Subscriber("/gripper/stat", GripperStat, self._update_gripper_stat, queue_size=10)

        # Way Points
        self.init_wpr = (0.484, -0.0887, 0.178, 2.29, -2.17, -0.0778)
        self.GotoBigframe = (0.540, 0.0889, 0.0781, 2.21, -2.19, 0.0067)
        self.PickBigframe = (0.540, 0.0889, 0.0106, 2.21, -2.19, 0.0067)
        self.Bigframe2Saywer = (0.588, 0.368, 0.0781, 3.12,-0.012, 0.0379)
        self.PutBigframe2Saywer = (0.588, 0.368, 0.0106,3.12, -0.012, 0.0379)
        self.PutBigframe2Saywer_Shaft1 = (0.588, 0.368, 0.1206,3.12, -0.012, 0.0379)
        self.PutBigframe2Saywer_Shaft2 = (0.588, 0.368, 0.0854,3.86, -0.0329, 0.041)
        self.PutBigframe2Saywer_Shaft3 = (0.588, 0.368, 0.0503,3.86, -0.0329, 0.041)
        self.GotoSmallframe = (0.343, -0.0616, 0.146, 0.0094, -2.63, -1.70)
        self.PickSmallframe = (0.343, -0.0616, 0.0179, 0.0094, -2.63, -1.70)
        self.Smallframe2Saywer = (0.625, 0.629, 0.146, 0.0093, -2.63, -1.70)
        self.PutSmallframe2Saywer = (0.625, 0.629, 0.0198, 0.0093, -2.63, -1.70)
        ### new demo 0530 ###### new demo 0530 ###### new demo 0530 ###### new demo 0530 ###
        ### new demo 0530 ###### new demo 0530 ###### new demo 0530 ###### new demo 0530 ###

        self.box_approach_height = 0.14

        # self.bin_homePose = (0.5077, 0.1230, 0.2231, pi, 0.0, -pi/2)
        self.bin_homePose = (0.5077, 0.1230, 0.2231, 2.29, -2.17, -0.0778)
        self.bin_viaPose_1 = (0.400, 0.313, 0.2531, 2.29, -2.17, -0.0778)
        self.bin_viaPose_2 = (0.2896, 0.4213, 0.2531, 2.29, -2.17, -0.0778)
        self.bin_viaPose_3 = (0.1432, 0.4529, 0.275, 2.29, -2.17, -0.0778)
        self.bin_viaPose_list = [self.bin_viaPose_1, self.bin_viaPose_2, self.bin_viaPose_3]
        self.bin_viaPose_list_reversed = [self.bin_viaPose_3, self.bin_viaPose_2,self.bin_viaPose_1]
        self.target_box1 = (0.2408, 0.5684, self.box_approach_height, 3.1415, 0.0, 0.0)
        self.target_box2 = (0.0253, 0.403, self.box_approach_height, 3.1415, 0.0, 0.0)
        self.target_box3 = (0.2361, 0.2845, self.box_approach_height, 3.1415, 0.0, 0.0)
        # self.target_box4 = (0.153, 0.400, self.box_approach_height, 3.1415, 0.0, 0.0)
        # self.target_box4 = (0.5571, 0.4528, self.box_approach_height, 3.14, 0.0, 0.0)

        # UR Setting & Move to Initial Point
        self.ur5 = urx.Robot("192.168.1.12")
        self.ur5.set_tcp((0.0, 0.0, 0.242, 0.0, 0.0, 0.0))
        self.ur_monitor = self.ur5.get_realtime_monitor()
        rospy.sleep(0.2)
        self.lin_accel = 1.4
        self.lin_vel = 0.7
        self.lin_z_vel = 0.5
        self.lin_z_accel = 0.5

        self.joint_accel = 10
        self.joint_vel = 2.5*pi
        # print('Go to initial Point!!!!')
        self._gripper_cmd = GripperCmd()
        self._gripper_stat = GripperStat()
        self._gripper_cmd.position = 0.085 * 2.0 /4.0
        self._gripper_cmd.speed = 0.02
        self._gripper_cmd.force = 150.0
        self.robotiqPub.publish(self._gripper_cmd)
        rospy.sleep(2)
        self.ur5.movel(self.init_wpr,self.lin_accel,self.lin_vel)

        # ur5.c([0,0, -0.0,0,-0,-1.5], acc=0.5, vel=0.5)

        # Gripper Command & State
        self.grasp_approach_height = 0.15
        self.gripper_pos_stat = 0.0




    def homing(self):
        self.ur5.movel(self.init_wpr,self.lin_accel,self.lin_vel)

    def _update_gripper_stat(self, stat):
        self._gripper_stat = stat
        self.gripper_pos_stat = self._gripper_stat.position

        # print (self._gripper_stat)




    def robotiq_openclose(self, ratio=0.0):
        print ('robotiq is moving to %0.2f' % (0.85 * ratio /4.0))
        self._gripper_cmd.position = 0.085 * ratio /4.0
        self._gripper_cmd.speed = 0.03
        self._gripper_cmd.force = 200.0
        self.robotiqPub.publish(self._gripper_cmd)




    def BringBigframe2Saywer(self):
        self.ur5.movel(self.GotoBigframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(2)
        QApplication.processEvents()
        rospy.sleep(1)
        self.ur5.movel(self.PickBigframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(0)
        rospy.sleep(1)
        self.ur5.movel(self.GotoBigframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.Bigframe2Saywer,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.PutBigframe2Saywer,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(2)
        rospy.sleep(1)
        QApplication.processEvents()
        self.ur5.movel(self.PutBigframe2Saywer_Shaft1,self.lin_accel,self.lin_vel)
        self.ur5.movel(self.PutBigframe2Saywer_Shaft2,self.lin_accel,self.lin_vel)
        self.ur5.movel(self.PutBigframe2Saywer_Shaft3,self.lin_accel,self.lin_vel)
        self.robotiq_openclose(0)
        QApplication.processEvents()



    def TakeAwayBigframefromSaywer(self):
        self.ur5.movel(self.Bigframe2Saywer,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.GotoBigframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.PickBigframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(2.3)
        QApplication.processEvents()
        rospy.sleep(1)
        self.ur5.movel(self.GotoBigframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()

    def Hold_Shaft(self):

        self.ur5.movel(self.PickBigframe,self.lin_accel,self.lin_vel)
        self.robotiq_openclose(0)
        rospy.sleep(1.0)

    def BringSmallframe2Saywer(self):
        self.ur5.movel(self.GotoSmallframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(3)
        QApplication.processEvents()
        rospy.sleep(1)
        self.ur5.movel(self.PickSmallframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(0)
        QApplication.processEvents()
        rospy.sleep(1.5)
        self.ur5.movel(self.GotoSmallframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.Smallframe2Saywer,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.PutSmallframe2Saywer,self.lin_accel,self.lin_vel)
        QApplication.processEvents()

    def TakeAwaySmallframefromSaywer(self):
        self.ur5.movel(self.Smallframe2Saywer,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.GotoSmallframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.ur5.movel(self.PickSmallframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()
        self.robotiq_openclose(3)
        QApplication.processEvents()
        rospy.sleep(2)
        self.ur5.movel(self.GotoSmallframe,self.lin_accel,self.lin_vel)
        QApplication.processEvents()

    def StartMoving(self):
        self.BringBigframe2Saywer()
        self.TakeAwayBigframefromSaywer()
        self.BringSmallframe2Saywer()
        self.TakeAwaySmallframefromSaywer()
        self.ur5.close()


    def moveToDetectionPose(self):
        self.ur5.movel(self.bin_homePose, self.lin_accel, self.lin_vel)
        QApplication.processEvents()




    def moveToDetectionPose_base(self, position=None, yaw=None, position_list=None):
        roll = 3.14
        pitch = 0.0
        if yaw == None:
            yaw = 0.0

        yaw = rad(yaw)
        if position:
            movel_tuple = (position[0], position[1], position[2] + self.grasp_approach_height, roll, pitch, yaw)
            self.ur5.movel(movel_tuple, self.lin_accel,self.lin_vel)
            QApplication.processEvents()



    def moveToDetectionPose_tool(self, position=None, roll=0.0, pitch=0.0, yaw=None):

        if yaw == None:
            yaw = 0.0

        yaw = rad(yaw)
        # tool frame's z-axis is opposite to that of base frame
        movel_tool_tuple = (position[0]/1000, position[1]/1000, position[2]/1000 - self.grasp_approach_height, roll, pitch, yaw)

        self.ur5.movel_tool(movel_tool_tuple, self.lin_accel,self.lin_vel)
        QApplication.processEvents()

    def move_x(self, trans_x=None):
        pose = self.ur5.getl()
        pose[0] += trans_x
        self.ur5.movel(pose, self.lin_accel, self.lin_vel)


    def move_y(self, trans_y=None):
        pose = self.ur5.getl()
        pose[1] += trans_y
        self.ur5.movel(pose, self.lin_accel, self.lin_vel)

    def move_z(self, trans_z=None):
        pose = self.ur5.getl()
        pose[2] += trans_z
        if trans_z >=0:
            self.ur5.movel(pose, self.lin_accel, self.lin_vel)
        else:
            self.ur5.movel(pose, self.lin_z_accel, self.lin_z_vel)

    def rotate_yaw(self, yaw=None):
        pose = self.ur5.getl()
        pose[5] += yaw
        self.ur5.movel(pose, self.lin_accel, self.lin_vel)




class Ui_MainWindow(object):
    def __init__(self):

        print("Initializing node...")
        rospy.init_node("ai_demo")
        self.limb = intera_interface.Limb("right")
        self.cuff = intera_interface.Cuff("right")
        self.th = trajectorySender(parent=self)
        self.th2 = urMotion(parent=self)
        self.th.start()
        self.th2.start()
        self.head_display = intera_interface.HeadDisplay()
        self.head_display.display_image(base_dir + "/head.png")
        self.bridge = CvBridge()

        self.initial_position = {'right_j0': rad(-2.553), 'right_j1': rad(-68.290), 'right_j2': rad(4.096),
                                 'right_j3': rad(127.056), 'right_j4': rad(-5.501), 'right_j5': rad(25.005),
                                 'right_j6': rad(81.015)}
        self.retrieve_position = {'right_j0': rad(-2.553), 'right_j1': rad(-68.290), 'right_j2': rad(4.096),
                                 'right_j3': rad(127.056), 'right_j4': rad(-5.501), 'right_j5': rad(25.005),
                                 'right_j6': rad(81.015)}

        self.initPos = {'x': 446.980,'y': 160.561,'z': 205.069,'Roll': -177.0,'Pitch': 5.0,'Yaw': -154.0}
        self.viaPos1 = {'x': 343.757,'y': -352.561,'z': 554.069,'Roll': 179.0,'Pitch': -19.0,'Yaw': 130.0}
        self.viaPos2 = {'x': 72.757,'y': -482.561,'z': 262.069,'Roll': 178.0,'Pitch': -2.60,'Yaw': 86.0}
        self.penPose = {'x': -181.0,'y': -632.211,'z': 0.0 ,'Roll': -172.233,'Pitch': 2.779,'Yaw': 89.610}
        self.botPose = {'x': -336.915,'y': -581.022,'z': 0.0 ,'Roll': 173.092,'Pitch': 9.348,'Yaw': 4.323}
        self.retract1 = {'x': 629.979,'y': -92.132,'z': 278.118,'Roll': -163.0,'Pitch': 8.40,'Yaw': -139.0}
        self.retract2 = {'x': 601.979,'y': 16.132,'z': 284.118,'Roll': -177.0,'Pitch': 24.60,'Yaw': -153.0}
        self.retract3 = {'x': 488.979,'y': 95.132,'z': 262.118,'Roll': 178.0,'Pitch': 18.60,'Yaw': -159.0}
        self.retract4 = {'x': 457.979,'y': 40.132,'z': 278.118,'Roll': -176.0,'Pitch': 5.60,'Yaw': 179.0}
        # self.retract2 = {'x': 573.979,'y': -145.132,'z': 337.118,'Roll': 171.0,'Pitch': 11.60,'Yaw': 96.0}
        self.waypoint_count = 0



        ### ROS node definition
        # rospy.Subscriber("/rcnn/right/image_color", Image_msg, self.receiveImgMsg1)
        # rospy.Subscriber("/rcnn/left/image_color", Image_msg, self.receiveImgMsg2)
        # self.imgToRCNNPub1 = rospy.Publisher("ImgPtGreyToRCNN1",Image_msg, queue_size=10)
        # self.imgToRCNNPub2 = rospy.Publisher("ImgPtGreyToRCNN2",Image_msg, queue_size=10)

        rospy.Subscriber("/detection_full1", DetectionFull, self.rcnnDetection1)

        rospy.Subscriber("/objects", positionNpose, self.poseEstimateCallback)
        # rospy.Subscriber("/rcnn/res/full1", DetectionFull, self.rcnnDetection2)

        self.targetObjAreaDictList = {'p': [], 'class': [], 'roi': [], 'item': [], 'image': [], 'pcl_xyz': [], 'theta': []}
        self.assembleFrameDictList = {'p': [], 'class': [], 'roi': [], 'item': [], 'image': [], 'pcl_xyz': [], 'theta': []}
        ## by RCNN1 &  pt grey1
        self.curDetectedAssemblePartsbyRCNN = {'bottle':[], 'glue':[], 'cell phone':[], 'piston shaft':[], 'gear': [], 'pen':[]}
        self.curDetectedTwoStepPartsbyRCNN = {'square':[], 'circle':[], 'hexagon':[], 'clover':[]}

        ## by RCNN2 &  pt grey2
        self.curDetectedTwoStepFramebyRCNN = {'square_frame':[], 'circle_frame':[], 'hexagon_frame':[], 'clover_frame':[]}
        self.curDetectedBaseFramesbyRCNN = {'small_frames':[], 'big_frames':[]}

        # self.totalObjectListInPartArea = []
        # self.totalObjectListInPartAreaTarget = []
        # self.totalObjectListInFrameArea = []
        # self.totalObjectListInFrameAreaTarget = []

        self.infoToAlexPub = rospy.Publisher("/aaa",MsgToAlex , queue_size=10)
        self.croppedRoIPub = rospy.Publisher("/croppedRoI", Image , queue_size=10)

        # rospy.Subscriber("/alexnet/coordinate_transform/GraspPatchToCoord", msgGraspPatchToCoord, self.alexNetRcvr) # image_x, image_y

        self.publishSchunk = rospy.Publisher('/wsg_32_driver/goal_position', Cmd, queue_size=30)
        self.robotiqPub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
        self.objCoordWrtCam1 = [0.0, 0.0, 0.0] # @ part area
        self.objCoordWrtCam2 = [0.0, 0.0, 0.0]

        # self.offsetRobToCam1 = [-140.3, -454.9, 652.1]
        self.offsetRobToCam1 = [-142.1, -521.5, 652.1]
        self.offsetRobToCam2 = [615.2, -491.8, 905.6]

        self.imgCoord1 = [0.0, 0.0]
        self.camCoord1 = [0.0, 0.0, 652.1]
        self.imgCoord2 = [0.0, 0.0]
        self.camCoord2 = [0.0, 0.0, 905.6]
        self.robPose1 = [0.0, 0.0, 0.0]
        self.robPose2 = [0.0, 0.0, 0.0]
        self.approachHeight = 0.2
        self.retrieveHeight = 0.2

        self.joint_angles_record = []

        self.joint_0_record = []
        self.joint_1_record = []
        self.joint_2_record = []
        self.joint_3_record = []
        self.joint_4_record = []
        self.joint_5_record = []
        self.joint_6_record = []
        self.gripper_record = []

        self.joint_0_vel_record = []
        self.joint_1_vel_record = []
        self.joint_2_vel_record = []
        self.joint_3_vel_record = []
        self.joint_4_vel_record = []
        self.joint_5_vel_record = []
        self.joint_6_vel_record = []

        self.joint_positon_btwn_Vel = 0.3

        self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        self.pose = PoseStamped()

        self.dctdobj_ary1 = np.zeros((20, 3))  # will be used for RoI
        self.dctdobj_ary2 = np.zeros((20, 3))  #

        self.roiWidth = 224
        self.roiHeight = 224
        # self.roiWidth = 150
        # self.roiHeight = 150
        # Must cvtColor before putting into cv2_to_imgmsg


        # image_grasp1 & image_grasp2 should be published

        self.itemLists = ['bottle', 'glue', 'cell phone', 'piston shaft', 'gear', 'big_frames', 'square', 'sqaure_frame',
                          'circle', 'circle_frame', 'hexagon', 'hexagon_frame', 'clover', 'clover_frame', 'small_frames'
                          ]

        self.imageFlag = False
        self.imageList1 = []
        self.imageList2 = []

        self.gripperlength = 0.152
        self.countBFbaseframe = 0
        self.closed_pos = 0.0 # mm (closed)
        self.open_pos = 50.0 # mm (open)
        self.speed = 50.0 # mm/s
        self.schunk_speed = 60.0 # mm/s
        self.schunk_force = 50.0 # mm/s
        # self.vacuumCommand1 =
        self.schunk_open = 100.0
        self.schunk_release = 68.0
        self.gripper_offset = 0.0
        self.schunk_closed = {'bottle': 38.0 + self.gripper_offset, 'glue': 23.0 + self.gripper_offset,
                              'cell phone' :65.5 + self.gripper_offset, 'piston shaft': 14.0 + self.gripper_offset,
                              'gear': 60.0 + self.gripper_offset, 'square': 38.0 + self.gripper_offset ,
                              'square_frame': 63.0 + self.gripper_offset, 'circle': 44.0 + self.gripper_offset,
                              'circle_frame': 63.0 +  self.gripper_offset, 'hexagon': 43.0 + self.gripper_offset,
                              'hexagon_frame': 63.0 + self.gripper_offset ,'clover': 44.0 + self.gripper_offset,
                              'clover_frame': 63.0 + self.gripper_offset, 'pen': 23.0+ self.gripper_offset}

        self.endpoint_state = self.th.limb.tip_state('right_hand')
        self.targetPose = self.endpoint_state.pose
        self.poseStamped = PoseStamped()

        self._gripper_cmd = GripperCmd()
        self._gripper_stat = GripperStat()

        self.currentAssemObj = ''

        self.grasp_svc = rospy.ServiceProxy('/wsg_32_driver/grasp', Move)
        self.stop_svc = rospy.ServiceProxy('/wsg_32_driver/stop', Empty)
        self.homing_svc = rospy.ServiceProxy('/wsg_32_driver/homing', Empty)
        self.release_svc = rospy.ServiceProxy('/wsg_32_driver/release', Move)
        self.activate_svc = rospy.ServiceProxy('/wsg_32_driver/ack', Empty)
        self.force_svc = rospy.ServiceProxy('/wsg_32_driver/set_force', Conf)
        self.acc_svc = rospy.ServiceProxy('/wsg_32_driver/set_acceleration', Conf)

        self.wait_after_grasping = 1.0
        self.wait_before_grasping = 1.0

        self.jerk_pose_dict = {'bottle': [0.0,0.0,0.0], 'glue': [0.0,0.0,0.0],'cell phone': [0.0,0.0,0.0], 'piston shaft':[0.0,0.0,0.0],
                              'gear': [0.0,0.0,0.0], 'square': [0.0,0.0,0.0],'sqaure_frame': [0.0,0.0,0.0], 'circle': [0.0,0.0,0.0],
                              'circle_frame': [0.0,0.0,0.0], 'hexagon': [0.0,0.0,0.0],'hexagon_frame': [0.0,0.0,0.0],'clover': [0.0,0.0,0.0],
                              'clover_frame': [0.0,0.0,0.0]}

        self.isBigFramePrepared = False
        self.isSmallFramePrepared = False
        self.urGraspingHeight = 0.15

        self.urPlacingHeight = 0.18
        self.urRetrievingHeight = 0.15

        # # reference
        left = -235
        right = 298
        top = -195
        bottom = 185
        self.safetyMarginTB = 10
        # self.safetyMarginTB = 95
        self.safetyMarginLR = 10
        # self.safetyMarginLR = 125


        # self.bin_left_margin = -225 #+ self.safetyMargin
        # self.bin_right_margin = 255 #- self.safetyMargin
        # self.bin_top_margin = -185 #+ self.safetyMargin
        # self.bin_bottom_margin = 175  #- self.safetyMargin

        self.demo_margin = 40
        # self.bin_left_margin = -195 + self.demo_margin #+ self.safetyMargin
        # self.bin_right_margin = 225 - self.demo_margin #- self.safetyMargin
        # self.bin_top_margin = -155 + self.demo_margin#+ self.safetyMargin
        # self.bin_bottom_margin = 145 -self.demo_margin #- self.safetyMargin
        self.bin_left_margin = -220 + self.demo_margin #+ self.safetyMargin
        self.bin_right_margin = 230 - self.demo_margin #- self.safetyMargin
        self.bin_top_margin = -170 + self.demo_margin#+ self.safetyMargin
        self.bin_bottom_margin = 170 -self.demo_margin #- self.safetyMargin
        # self.bin_left_margin = -170
        # self.bin_right_margin = 190
        # self.bin_top_margin = 106
        # self.bin_bottom_margin = -114

        self.object_approach_metric = {'glue': 15/1000,'pen': 15/1000, 'gear': 6/1000, 'bottle': 25/1000}

        self.dict_object = {0:'bottle', 1:'pen', 2:'gear', 3: 'glue'}
        # self.pca = PCA(n_components=2)



        self.item_target_box = {'box1': ['cell phone', 'remote controller', '(mouse)', 'gun', 'piston shaft', 'gear'],
                                'box2': ['pink pig', '(blue pig)', 'brown dog', 'toy car','baseball'],
                                'box3': ['bottle', 'yellow cup', 'purple cup', 'brush', 'sauce bottle', '(toothpaste)', 'glue', 'blue opener']}

    def pubDataToAlexnet(self, position=None):
        pass
        # process the bounding box to pass over to AlexNet
        # self.detectionfull1 = data
        # self.detectionmsg1 = self.detectionfull1.detections  # there could be multiple of detected objects, 'data' is list type
        # self.detectionimg1 = self.detectionfull1.image
        # self.cv_dctimage1 = self.bridge.imgmsg_to_cv2(self.detectionimg1, "bgr8")  # 1280 * 1024
        # self.cv_dspimage1 = cv2.cvtColor(self.cv_dctimage1, cv2.COLOR_BGR2RGB)  # 1280 * 1024
        #
        # self.np_image1 = np.asarray(self.cv_dctimage1)
        #
        # for i in range(0, self.detectionmsg1.size):  # loop over all the detected images
        #     self.dctdobj_ary1[i][0] = self.detectionmsg1.data[i].x + self.detectionmsg1.data[i].width / 2
        #     self.dctdobj_ary1[i][1] = self.detectionmsg1.data[i].y + self.detectionmsg1.data[i].height / 2
        #
        #     self.roiCoord1 = [self.dctdobj_ary1[i][0] - 50, self.dctdobj_ary1[i][1] - 50]
        #
        #     if self.roiCoord1[0] < 0:
        #         self.roiCoord1[0] = 0
        #     if self.roiCoord1[1] < 0:
        #         self.roiCoord1[1] = 0
        #
        #     num_rows1, num_cols1 = self.np_image1.shape
        #
        #     if (self.roiCoord1[0] + 100 > num_cols1):
        #         self.roiWidth1 = num_cols1 - self.roiCoord1[0]
        #
        #     if (self.roiCoord1[1] + 100 > num_rows1):
        #         self.roiHeight1 = num_cols1 - self.roiCoord1[1]
        #
        #     ### Setup RoI on the image input
        #     ### Refer by search on Google with the following keywords; numpy array, opencv, region of interest
        #
        #     self.np_RoI_img1 = self.np_image1[self.roiCoord1[0]:self.roiWidth1, self.roiCoord1[1]:self.roiHeight1]
        #     self.image_grasp1 = self.np_RoI_img1
        #
        #     self.image_grasp1 = cv2.resize(self.image_grasp1, None, fx=2.0, fy=2.0)
        #     self.image_grasp1 = cv2.resize(self.image_grasp1, (200, 200))
        #
        #     # now it should be sent to AlexNet!, selectively
        #
        #     self.dctdobj_cls1 = self.detectionmsg1.data[i].object_class
        #
        #     self.objcls_pt1 = [self.detectionmsg1.data[i].x, self.detectionmsg1.data[i].y]
        #
        #     cv2.rectangle(self.cv_dspimage1,
        #                   (self.objcls_pt1[0], self.objcls_pt1[1]),
        #                   (self.detectionmsg1.data[i].x + self.detectionmsg1.data[i].width,
        #                    self.detectionmsg1.data[i].y + self.detectionmsg1.data[i].height),
        #                   (0, 255, 0),
        #                   3)
        #
        #     cv2.putText(self.cv_dspimage1, self.dctdobj_cls1, (self.objcls_pt1[0], self.objcls_pt1[1]),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        #
        # if position == None:
        #     position = [0.0, 0.0]
        # else:
        #     data = MsgToAlex() # 0: prob, 1: roi [x, y], 2: class 3: image Image()
        #     data.image = self.self.curDetectedAssemblePartsbyRCNN['bottle'][0][3] #
        #     data.roi_x = self.self.curDetectedAssemblePartsbyRCNN['bottle'][0][1][0] #
        #     data.roi_y = self.self.curDetectedAssemblePartsbyRCNN['bottle'][0][1][1] #
        #     self.infoToAlexPub.publish(data)

    def alexNetRcvr(self, data):

        self.rot_z_theta = data.theta


        # receives x, y ,theta of requested object

    def robotiq_close(self):
        print ('robotiq closes')
        self._gripper_cmd.position = 0.0
        self._gripper_cmd.speed = 0.05
        self._gripper_cmd.force = 140.0
        self.robotiqPub.publish(self._gripper_cmd)

    def robotiq_open(self, ratio=4.0):
        print ('robotiq opens %0.2f' % (0.85 * ratio / 4.0))
        self._gripper_cmd.position = 0.085 * ratio / 4.0
        self._gripper_cmd.speed = 0.02
        self._gripper_cmd.force = 100.0

        self.robotiqPub.publish(self._gripper_cmd)

    def gripper_on(self, object=''):
        try:
            rospy.wait_for_service('/wsg_32_driver/set_force', timeout=3)
            rospy.wait_for_service('/wsg_32_driver/grasp', timeout=3)
            acc_resp = self.force_svc(self.schunk_force)
            self.grasp_resp = self.grasp_svc(self.schunk_closed[object], self.schunk_speed)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('get action service call failed')

        return self.grasp_resp

    def init_gripper(self):
        try:
            rospy.wait_for_service('/wsg_32_driver/stop', timeout=3)
            rospy.wait_for_service('/wsg_32_driver/ack', timeout=3)
            rospy.wait_for_service('/wsg_32_driver/homing', timeout=3)
            stop_resp = self.stop_svc()
            ack_resp = self.activate_svc()
            homing_resp = self.homing_svc()
            print('Schunk has initialized')
        except (rospy.ServiceException ,rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('get action service call failed')


    def gripper_off(self):
        try:
            rospy.wait_for_service('/wsg_32_driver/release', timeout=3)
            release_resp = self.release_svc(self.schunk_release, self.schunk_speed)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('get action service call failed')

    def overwrite_measured_Values(self):
        if self.selfDEMO_btin.isChecked():
            print('OK to GO')
        else:
            print('measured value will be overwritten')

    # def receiveImgMsg1(self, data):
    #
    #     self.rcvdimg1 = data ## ROS default image
    #     self.img_tstmp1 = rospy.get_time()
    #     self.cv_image1 = self.bridge.imgmsg_to_cv2(self.rcvdimg1, "bgr8") #1280 * 1024
    #     self.cv_image1 = cv2.cvtColor(self.cv_image1, cv2.COLOR_BGR2RGB)
    #     self.publishRGB1toRCNN()
    #
    # def receiveImgMsg2(self, data):
    #
    #     self.rcvdimg2 = data
    #     self.img_tstmp2 = rospy.get_time()
    #     self.cv_image2 = self.bridge.imgmsg_to_cv2(self.rcvdimg2, "bgr8")  # 1280 * 1024
    #     self.cv_image2 = cv2.cvtColor(self.cv_image2, cv2.COLOR_BGR2RGB)
    #     self.publishRGB2toRCNN()

    # def publishRGB1toRCNN(self):
    #
    #     self.cv_image_crop1 = self.cv_image1
    #     self.rgbImg1 = cv2.cvtColor(self.cv_image1, cv2.COLOR_BGR2RGB)
    #     self.rgbMsgToRCNN1 = self.bridge.cv2_to_imgmsg(self.rgbImg1)
    #     self.imgToRCNNPub1.publish(self.rgbMsgToRCNN1)
    #
    # def publishRGB2toRCNN(self):
    #
    #     self.cv_image_crop2 = self.cv_image2
    #     self.rgbImg2 = cv2.cvtColor(self.cv_image2, cv2.COLOR_BGR2RGB)
    #     self.rgbMsgToRCNN2 = self.bridge.cv2_to_imgmsg(self.rgbImg2)
    #     self.imgToRCNNPub2.publish(self.rgbMsgToRCNN2)

    def robot_Button_Clicked(self):  # Sawyer Enable/Disable

        if self.InitAllNodes.isChecked():
            self.Logs.append(_fromUtf8("Sawyer is being enabled"))
            self.enable_sawyer()
            global isEnabled
            isEnabled = True

            self.th.trajectory.clear_waypoints()
            self.endpoint_state = self.th.limb.tip_state('right_hand')
            self.curPose = self.endpoint_state.pose
            self.poseStamped1A = PoseStamped()
            self.poseStamped1A.pose = self.curPose
            self.th.waypoint_initial.set_cartesian_pose(self.poseStamped1A, 'right_hand', None)


        elif not self.InitAllNodes.isChecked():
            self.Logs.append(_fromUtf8("Sawyer is being disabled"))
            self.disable_sawyer()
            global isEnabled
            isEnabled = False

    def enable_sawyer(self):
        # self.curPallet_order.setText(QtCore.QString.number(self.palletizingStep))
        self.th2.start()
        self.th.start()
        self.RobotState = intera_interface.RobotEnable(CHECK_VERSION)
        self.RobotState = intera_interface.RobotEnable(CHECK_VERSION)
        self.Logs.append('Enabling robot...')

        self.RobotState.enable()
        self.th.limb = intera_interface.Limb("right")
        self.th2.ur5 = urx.Robot("192.168.1.12")

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
            self.th.limb.set_joint_position_speed(0.2)
            self.th.limb.move_to_joint_positions(self.initial_position)
            # self.th.limb.move_to_joint_positions(self.initial_position)
        # self.gripper.open()
        global isGripperOpen
        isGripperOpen = True
        self.th.trajectory.clear_waypoints()


    def disable_sawyer(self):
        self.Logs.append('Disabling robot...')
        # self.gripper_off()
        self.robotiq_open(2.0)
        # self.robotiq_open()
        # trans = self.th2.ur5.get_pose()
        # trans.pos.z +=0.2
        if self.isBigFramePrepared:
            self.th2.TakeAwayBigframefromSaywer()
            self.isBigFramePrepared = False
        if self.isSmallFramePrepared:
            self.th2.TakeAwaySmallframefromSaywer()
            self.isSmallFramePrepared = False
        # self.th2.ur5.set_pose(trans, acc=self.th2.lin_accel, vel= self.th2.lin_vel)
        # self.th2.homing()
        self.th2.ur5.close()
        print('UR exited')
        # self.th.limb.move_to_neutral()
        self.th.limb.set_joint_position_speed(0.2)
        self.th.limb.move_to_joint_positions(self.initial_position)
        self.RobotState.disable()
        global isAtNeutral
        isAtNeutral = True
        # self.gripper.close()
        # global isGripperOpen
        # isGripperOpen = False
        # self.timer3.stop()
        self.th2.quit()
        self.th.quit()




    def lowerX(self, value):

        return value.glo_x

    def higherY(self, value):

        return value.glo_y

    def higherPred(self, value):

        return value.p

    # def monitorCurrentObjects(self): # sort by left most
    #     self.totalObjectListInPartArea = []
    #     self.totalObjectListInFrameArea = []
    #
    #     self.curDetectedAssemblePartsbyRCNN['bottle'] = []
    #     self.curDetectedAssemblePartsbyRCNN['glue'] = []
    #     self.curDetectedAssemblePartsbyRCNN['cellphone'] = []
    #     self.curDetectedAssemblePartsbyRCNN['pistonshaft'] = []
    #     self.curDetectedAssemblePartsbyRCNN['gear'] = []
    #
    #     self.curDetectedTwoStepPartsbyRCNN['square'] = []
    #     self.curDetectedTwoStepPartsbyRCNN['circle'] = []
    #     self.curDetectedTwoStepPartsbyRCNN['hexagon'] = []
    #     self.curDetectedTwoStepPartsbyRCNN['clover'] = []
    #
    #     self.curDetectedTwoStepFramebyRCNN['square_frame'] = []
    #     self.curDetectedTwoStepFramebyRCNN['circle_frame'] = []
    #     self.curDetectedTwoStepFramebyRCNN['hexagon_frame'] = []
    #     self.curDetectedTwoStepFramebyRCNN['clover_frame'] = []
    #
    #     self.curDetectedBaseFramesbyRCNN['small_frames'] = []
    #     self.curDetectedBaseFramesbyRCNN['big_frames'] = []
    #
    #     for item in self.targetObjAreaDictList['item']:
    #         self.totalObjectListInPartArea.append(item)
    #         self.totalObjectListInPartArea = sorted(self.totalObjectListInPartArea, key=itemgetter(0), reverse = True)
    #         if item.label == 'bottle':
    #             self.curDetectedAssemblePartsbyRCNN['bottle'].append(item)
    #             self.curDetectedAssemblePartsbyRCNN['bottle'] = sorted(self.curDetectedAssemblePartsbyRCNN['bottle'], key=itemgetter(0), reverse = True)
    #         elif item.label == 'glue':
    #             self.curDetectedAssemblePartsbyRCNN['glue'].append(item)
    #             self.curDetectedAssemblePartsbyRCNN['glue'] = sorted(self.curDetectedAssemblePartsbyRCNN['glue'], key=itemgetter(0), reverse=True)
    #         elif item.label == 'cellphone':
    #             self.curDetectedAssemblePartsbyRCNN['cellphone'].append(item)
    #             self.curDetectedAssemblePartsbyRCNN['cellphone'] = sorted(self.curDetectedAssemblePartsbyRCNN['cellphone'], key=itemgetter(0) , reverse=True)
    #         elif item.label == 'pistonshaft':
    #             self.curDetectedAssemblePartsbyRCNN['pistonshaft'].append(item)
    #             self.curDetectedAssemblePartsbyRCNN['pistonshaft'] = sorted(self.curDetectedAssemblePartsbyRCNN['pistonshaft'],key=itemgetter(0), reverse=True)
    #         elif item.label == 'gear':
    #             self.curDetectedAssemblePartsbyRCNN['gear'].append(item)
    #             self.curDetectedAssemblePartsbyRCNN['gear'] = sorted(self.curDetectedAssemblePartsbyRCNN['gear'], key=itemgetter(0), reverse=True)
    #
    #
    #     for item in self.targetObjAreaDictList['item']:
    #         self.totalObjectListInBoxB.append(item)
    #         self.totalObjectListInBoxB = sorted(self.totalObjectListInBoxB, key=self.higherPred, reverse=True)
    #         if item.label == 'square':
    #             self.curDetectedTwoStepPartsbyRCNN['square'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['square'] = sorted(self.curDetectedTwoStepPartsbyRCNN['square'], key=self.higherPred, reverse=True)
    #         elif item.label == 'circle':
    #             self.curDetectedTwoStepPartsbyRCNN['circle'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['circle'] = sorted(self.curDetectedTwoStepPartsbyRCNN['circle'], key=self.higherPred, reverse=True)
    #         elif item.label == 'hexagon':
    #             self.curDetectedTwoStepPartsbyRCNN['hexagon'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['hexagon'] = sorted(self.curDetectedTwoStepPartsbyRCNN['hexagon'], key=self.higherPred, reverse=True)
    #         elif item.label == 'clover':
    #             self.curDetectedTwoStepPartsbyRCNN['clover'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['clover'] = sorted(self.curDetectedTwoStepPartsbyRCNN['clover'],key=self.higherPred, reverse=True)
    #     self.totalObjectListInPartAreaTarget = self.totalObjectListInPartArea
    #
    #     for item in self.assembleFrameDictList['item']:
    #         self.totalObjectListInFrameArea.append(item)
    #         self.totalObjectListInFrameArea = sorted(self.totalObjectListInFrameArea, key=self.higherPred, reverse=True)
    #         if item.label == 'square_frame':
    #             self.curDetectedTwoStepPartsbyRCNN['square_frame'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['square_frame'] = sorted(self.curDetectedTwoStepPartsbyRCNN['square_frame'], key=self.higherPred, reverse=True)
    #         elif item.label == 'circle_frame':
    #             self.curDetectedTwoStepPartsbyRCNN['circle_frame'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['circle_frame'] = sorted(self.curDetectedTwoStepPartsbyRCNN['circle_frame'], key=self.higherPred, reverse=True)
    #         elif item.label == 'hexagon_frame':
    #             self.curDetectedTwoStepPartsbyRCNN['hexagon_frame'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['hexagon_frame'] = sorted(self.curDetectedTwoStepPartsbyRCNN['hexagon_frame'], key=self.higherPred, reverse=True)
    #         elif item.label == 'clover_frame':
    #             self.curDetectedTwoStepPartsbyRCNN['clover_frame'].append(item)
    #             self.curDetectedTwoStepPartsbyRCNN['clover_frame'] = sorted(self.curDetectedTwoStepPartsbyRCNN['clover_frame'],key=self.higherPred, reverse=True)
    #
    #     for item in self.assembleFrameDictList['item']:
    #         self.totalObjectListInFrameArea.append(item)
    #         self.totalObjectListInFrameArea = sorted(self.totalObjectListInFrameArea, key=self.higherPred, reverse=True)
    #         if item.label == 'small_frames':
    #             self.curDetectedBaseFramesbyRCNN['small_frames'].append(item)
    #             self.curDetectedBaseFramesbyRCNN['small_frames'] = sorted(self.curDetectedBaseFramesbyRCNN['small_frames'], key=self.higherPred, reverse=True)
    #         elif item.label == 'big_frames':
    #             self.curDetectedBaseFramesbyRCNN['big_frames'].append(item)
    #             self.curDetectedBaseFramesbyRCNN['big_frames'] = sorted(self.curDetectedBaseFramesbyRCNN['big_frames'], key=self.higherPred, reverse=True)

    def poseEstimateCallback(self, data):

        # data: X, Y and theta
        # is data not going to be overwritten??

        print 'Target info received!!'



        self.target_croppedX = data.x
        self.target_croppedY = data.y
        self.target_croppedT = data.theta


    def getGrapsingPose(self, topleftCoord=None):


        graspingPose = [0.0, 0.0, 0.0]

        graspingPose[0] = self.target_croppedX + topleftCoord[0]
        graspingPose[1] = self.target_croppedY + topleftCoord[1]
        graspingPose[2] = self.target_croppedT

        return graspingPose




    def frameTFimageToCam1(self, imgCoord=None, depth=None):
        # pc_avg = np.asanyarray(tuple(self.rec_pc1[tuple(target_pixel_depth)])) * 1000
       # realsense intrinsic parameters
        # Use realsense with point cloud
        ## Check if depth is in mm

        # width: 1280, height: 720, ppx: 647.657, ppy: 353.121, fx: 638.682, fy: 638.682, model
        if imgCoord == None:
            self.imgCoord = [0.0, 0.0]
        else:
            self.imgCoord = imgCoord

        depth = 560.3
        # self.objCoordWrtCam1[0] = ((self.imgCoord[0] - 647.875) * (1 / 638.682)) * depth  # distance btwn object & camera
        self.objCoordWrtCam1[0] = ((self.imgCoord[0] - 647.808) * (1 / 638.975)) * depth  # distance btwn object & camera
        self.objCoordWrtCam1[1] = ((self.imgCoord[1] - 353.122) * (1 / 638.975)) * depth

        return self.objCoordWrtCam1[0],self.objCoordWrtCam1[1]

    # def frameTFimageToCam2(self, imgCoord=None):
    #
    #     if imgCoord == None:
    #         self.imgCoord = [0.0, 0.0]
    #     else:
    #         self.imgCoord = imgCoord
    #         self.objCoordWrtCam1[0] = ((self.imgCoord[0] - 634.823565) * (1 / 1583.572913)) * 905.6 * 0.001 # distance btwn object & camera
    #         self.objCoordWrtCam1[1] = ((self.imgCoord[1] - 569.914781) * (1 / 1586.708187)) * 905.6 * 0.001




    def coordTransform1(self, position):
        # cam coord to robot's ee coord
        # position array of 3 DoF  (X,Y,Z)
        # position[0] = X, position[1] = Y, position[2] = Z in camera coordinate
        currentURPose = [self.th2.ur5.x, self.th2.ur5.y, self.th2.ur5.z]
        # Transformation btwn cam_coord to tool_coord
        r_x = 0.0 # Roll rotation
        # t_x = self.offsetRobToCam1[0]
        # t_x = -32.0
        t_x = -34.0
        # t_y = self.offsetRobToCam1[1]
        # t_y = -70.0
        t_y = -80.0
        # t_z = self.offsetRobToCam1[2]
        t_z = -193.0

        objectPose_toolFrame = PyKDL.Vector(position[0], position[1], position[2])

        rotation_matrix = PyKDL.Rotation.RotX(rad(r_x)) # 3*3
        translation_matrix = PyKDL.Vector(t_x,t_y,t_z)  # 3*1./
        homegen_tf_matrix = PyKDL.Frame(rotation_matrix, translation_matrix)
        objectPose_robotFrame1 = homegen_tf_matrix * objectPose_toolFrame # 3*1
        return objectPose_robotFrame1

    def coordTransform2(self, position):
        # Right-side to the Sawer's control box pheriperal
        # position array of 3 DoF  (X,Y,Z)
        # position[0] = X, position[1] = Y, position[2] = Z in camera coordinate
        r_x = 180.0 # Roll rotation
        r_y = 0.0 # Roll rotation
        r_z = -90.0 # Roll rotation
        t_x = self.offsetRobToCam2[0]
        t_y = self.offsetRobToCam2[1]
        t_z = self.offsetRobToCam2[2]
        objectPose_boxFrame = PyKDL.Vector(position[0], position[1], position[2])

        rotation_matrix = PyKDL.Rotation.RPY(rad(r_x),rad(r_y),rad(r_z)) # 3*3
        translation_matrix = PyKDL.Vector(t_x,t_y,t_z)  # 3*1./
        homegen_tf_matrix = PyKDL.Frame(rotation_matrix, translation_matrix)
        objectPose_robotFrame2 = homegen_tf_matrix * objectPose_boxFrame # 3*1
        return objectPose_robotFrame2

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
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.Logs.append(_fromUtf8('Initial_waypoint_appended'))

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
        self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
        self.waypoint_count += 1
        self.Logs.append(_fromUtf8('Waypoint %d appended' % self.waypoint_count))

    def grippingRetrial(self, numTrial=0, whichobject=''):
        base_retrial_x_offset = 10 / 235000.0  # w.r.t. robot's baseframe
        base_retrial_y_offset = 10 / 160000.0
        # self.gripper_off()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
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
                        # self.gripper_on()
                        rospy.sleep(1.0)
                        self.addMotionWayPoints(self.tempTargetBposRobCoord[0],
                                                self.tempTargetBposRobCoord[1],
                                                self.tempTargetBposRobCoord[2],
                                                z_offset=self.grippingCheck_height)
                        self.th.sendTrajectory()
                QApplication.processEvents()


    def clear_arrays(self):

        self.joint_0_record = []
        self.joint_1_record = []
        self.joint_2_record = []
        self.joint_3_record = []
        self.joint_4_record = []
        self.joint_5_record = []
        self.joint_6_record = []

        print('Successfully cleared numpy arrays')

    def save_demo_values_to_csv(self):
        print('Joint Demo1 recording has started')
        self.joint_0_record[:] = []
        self.joint_1_record[:] = []
        self.joint_2_record[:] = []
        self.joint_3_record[:] = []
        self.joint_4_record[:] = []
        self.joint_5_record[:] = []
        self.joint_6_record[:] = []
        self.timer3 = QtCore.QTimer()
        self.timer3.timeout.connect(self.append_joint_angles_to_list)
        self.timer3.start(10)
        os.chdir('/home/irobot/Downloads/Cheolhui/')

    def append_joint_angles_to_list(self):
        global isGripperOpen
        if self.cuff.cuff_button():
            print('Under demo recording...')
            self.joint_0_record.append(deg(self.limb.joint_angle('right_j0')))
            self.joint_1_record.append(deg(self.limb.joint_angle('right_j1')))
            self.joint_2_record.append(deg(self.limb.joint_angle('right_j2')))
            self.joint_3_record.append(deg(self.limb.joint_angle('right_j3')))
            self.joint_4_record.append(deg(self.limb.joint_angle('right_j4')))
            self.joint_5_record.append(deg(self.limb.joint_angle('right_j5')))
            self.joint_6_record.append(deg(self.limb.joint_angle('right_j6')))

        if self.cuff.lower_button():
            self.endpoint_pose = self.th.limb.endpoint_pose()
            self.jerk_pose_dict[self.currentAssemObj] = self.endpoint_pose['position']
            print('jerk safety pose has been recorded')
            print(self.jerk_pose_dict[self.currentAssemObj])
            rospy.sleep(1.0)

        if self.cuff.upper_button():
            self.joint_0_record_np = np.array(self.joint_0_record)
            self.joint_1_record_np = np.array(self.joint_1_record)
            self.joint_2_record_np = np.array(self.joint_2_record)
            self.joint_3_record_np = np.array(self.joint_3_record)
            self.joint_4_record_np = np.array(self.joint_4_record)
            self.joint_5_record_np = np.array(self.joint_5_record)
            self.joint_6_record_np = np.array(self.joint_6_record)
            self.save_data = np.column_stack((self.joint_0_record_np,
                                              self.joint_1_record_np,
                                              self.joint_2_record_np,
                                              self.joint_3_record_np,
                                              self.joint_4_record_np,
                                              self.joint_5_record_np,
                                              self.joint_6_record_np
                                              ))


            if self.currentAssemObj == 'bottle':
                if os.path.isfile("bottle.csv"):
                    os.remove("bottle.csv")
                np.savetxt("bottle.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'glue':
                if os.path.isfile("glue.csv"):
                    os.remove("glue.csv")
                np.savetxt("glue.csv", self.save_data, delimiter=',', fmt='%.3e')
            elif self.currentAssemObj == 'pen':
                if os.path.isfile("pen.csv"):
                    os.remove("pen.csv")
                np.savetxt("pen.csv", self.save_data, delimiter=',', fmt='%.3e')
            elif self.currentAssemObj == 'cellphone':
                if os.path.isfile("phone.csv"):
                    os.remove("phone.csv")
                np.savetxt("phone.csv", self.save_data, delimiter=',', fmt='%.3e')
            elif self.currentAssemObj == 'pistonshaft':
                if os.path.isfile("piston_shaft.csv"):
                    os.remove("piston_shaft.csv")
                np.savetxt("piston_shaft.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'gear':
                if os.path.isfile("gear.csv"):
                    os.remove("gear.csv")
                np.savetxt("gear.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'square':
                if os.path.isfile("square.csv"):
                    os.remove("square.csv")
                np.savetxt("square.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'circle':
                if os.path.isfile("circle.csv"):
                    os.remove("circle.csv")
                np.savetxt("circle.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'hexagon':
                if os.path.isfile("hexagon.csv"):
                    os.remove("hexagon.csv")
                np.savetxt("hexagon.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'clover':
                if os.path.isfile("clover.csv"):
                    os.remove("clover.csv")
                np.savetxt("clover.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'square_frame':
                if os.path.isfile("square_frame.csv"):
                    os.remove("square_frame.csv")
                np.savetxt("square_frame.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'circle_frame':
                if os.path.isfile("circle_frame.csv"):
                    os.remove("circle_frame.csv")
                np.savetxt("circle_frame.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'hexagon_frame':
                if os.path.isfile("hexagon_frame.csv"):
                    os.remove("hexagon_frame.csv")
                np.savetxt("hexagon_frame.csv", self.save_data, delimiter=',', fmt='%.3e')

            elif self.currentAssemObj == 'clover_frame':
                if os.path.isfile("clover_frame.csv"):
                    os.remove("clover_frame.csv")
                np.savetxt("clover_frame.csv", self.save_data, delimiter=',', fmt='%.3e')

            print("Successfully recorded joint demo8")
            self.timer3.stop()
            self.joint_0_record_np.fill(0)
            self.joint_1_record_np.fill(0)
            self.joint_2_record_np.fill(0)
            self.joint_3_record_np.fill(0)
            self.joint_4_record_np.fill(0)
            self.joint_5_record_np.fill(0)
            self.joint_6_record_np.fill(0)

    def read_learned_values_from_csv(self, item = ''):
        self.cwd = os.getcwd()
        os.chdir('/home/irobot/Downloads/Cheolhui/')
        self.data = np.loadtxt(item + '.csv', delimiter=",", dtype=np.float32)
        self.learned_j0_nd = self.data[0:, 0]
        self.learned_j1_nd = self.data[0:, 1]
        self.learned_j2_nd = self.data[0:, 2]
        self.learned_j3_nd = self.data[0:, 3]
        self.learned_j4_nd = self.data[0:, 4]
        self.learned_j5_nd = self.data[0:, 5]
        self.learned_j6_nd = self.data[0:, 6]


        self.learned_j0 = self.learned_j0_nd.tolist()
        self.learned_j1 = self.learned_j1_nd.tolist()
        self.learned_j2 = self.learned_j2_nd.tolist()
        self.learned_j3 = self.learned_j3_nd.tolist()
        self.learned_j4 = self.learned_j4_nd.tolist()
        self.learned_j5 = self.learned_j5_nd.tolist()
        self.learned_j6 = self.learned_j6_nd.tolist()

        self.n = len(self.learned_j0)

        global JS
        global CS

        if self.n != 0 and JS:
            global isTimer2_ok
            isTimer2_ok = True
            global numDemo
            numDemo = 1
            self.clear_arrays()
            self.import_learned_value_clicked()



    def import_learned_value_clicked(self):
        if self.selfDEMO_btin.isChecked():  ## 0 unchecked otherwise checked
            print('~~~~~~~~~~~~~~~~~~~~~~')
            print('import values for test')
            print('~~~~~~~~~~~~~~~~~~~~~~')
        else:
            print('Warning! :: the csv file is being overwritten')
            print('measured joint values are being recorded')
        # self.limb.move_to_joint_positions({'right_j0': rad(self.learned_j0[0]),
        #                                'right_j1': rad(self.learned_j1[0]),
        #                                'right_j2': rad(self.learned_j2[0]),
        #                                'right_j3': rad(self.learned_j3[0]),
        #                                'right_j4': rad(self.learned_j4[0]),
        #                                'right_j5': rad(self.learned_j5[0]),
        #                                'right_j6': rad(self.learned_j6[0]),
        #                                })
        # self.limb.move_to_neutral()
        global isTimer2_ok
        if isTimer2_ok:
            self.timer2 = QtCore.QTimer()
            self.timer2.timeout.connect(self.update_learned_values_on_robots)
            self.timer2.start(10)
            # self.update_learned_values_on_robots()
        self.update_timstep = 10  # in ms

    def update_learned_values_on_robots(self):
        global k
        try:
            self.limb.set_joint_position_speed(self.joint_positon_btwn_Vel)
            self.limb.set_joint_positions({'right_j0': rad(self.learned_j0[k]),
                                           'right_j1': rad(self.learned_j1[k]),
                                           'right_j2': rad(self.learned_j2[k]),
                                           'right_j3': rad(self.learned_j3[k]),
                                           'right_j4': rad(self.learned_j4[k]),
                                           'right_j5': rad(self.learned_j5[k]),
                                           'right_j6': rad(self.learned_j6[k]),
                                           })

            if k > 1:
                self.joint_0_record.append(deg(self.limb.joint_angle('right_j0')))
                self.joint_1_record.append(deg(self.limb.joint_angle('right_j1')))
                self.joint_2_record.append(deg(self.limb.joint_angle('right_j2')))
                self.joint_3_record.append(deg(self.limb.joint_angle('right_j3')))
                self.joint_4_record.append(deg(self.limb.joint_angle('right_j4')))
                self.joint_5_record.append(deg(self.limb.joint_angle('right_j5')))
                self.joint_6_record.append(deg(self.limb.joint_angle('right_j6')))


        except OSError:
            rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
            rospy.sleep(.5)

        k = k + 1

        if k == len(self.learned_j0):
            self.timer2.stop()
            k = 0
            rospy.sleep(0.5)
            # self.gripper_off()
            rospy.sleep(0.5)
            if self.currentAssemObj=='circle_frame' or self.currentAssemObj=='square_frame' or self.currentAssemObj=='hexagon_frame' or self.currentAssemObj=='clover_frame' or \
                self.currentAssemObj == 'cellphone' or self.currentAssemObj=='bottle' or self.currentAssemObj=='glue' or self.currentAssemObj=='gear' or self.currentAssemObj=='pistonshaft' or self.currentAssemObj=='bottle':
                self.endpoint_state = self.th.limb.tip_state('right_hand')
                self.targetPose = self.endpoint_state.pose
                self.targetPose.position.z += 0.15
                self.poseStamped.pose = self.targetPose
                self.th.waypoint.set_cartesian_pose(self.poseStamped, 'right_hand', None)
                self.th.trajectory.append_waypoint(self.th.waypoint.to_msg())
                self.th.sendTrajectory()
            elif self.currentAssemObj=='circle' or self.currentAssemObj=='square' or self.currentAssemObj=='hexagon' or self.currentAssemObj=='clover':
                self.addMotionWayPoints(self.retract1['x'], self.retract1['y'], self.retract1['z'],
                                        roll=self.retract1['Roll'],pitch=self.retract1['Pitch'], yaw=self.retract1['Yaw'])
            #         self.addMotionWayPoints(self.retract2['x'], self.retract2['y'], self.retract2['z'],
            #                                 roll=self.retract2['Roll'], pitch=self.retract2['Pitch'], yaw=self.retract2['Yaw'])
            #         self.addMotionWayPoints(self.retract3['x'], self.retract3['y'], self.retract3['z'],
            #                                 roll=self.retract3['Roll'], pitch=self.retract3['Pitch'], yaw=self.retract3['Yaw'])
            #         self.addMotionWayPoints(self.retract4['x'], self.retract4['y'], self.retract4['z'],
            #                                 roll=self.retract4['Roll'], pitch=self.retract4['Pitch'], yaw=self.retract4['Yaw'])
                self.th.sendTrajectory()
            #     self.th.limb.set_joint_position_speed(0.3)
            #     self.th.limb.move_to_joint_positions(self.initial_position)

            if self.currentAssemObj == 'pistonshaft':
                # self.th2.Hold_Shaft()
                self.robotiq_close()
            if self.currentAssemObj=='circle_frame':
                self.pickCircle()
            elif self.currentAssemObj == 'square_frame':
                self.pickSquare()
            elif self.currentAssemObj == 'hexagon_frame':
                self.pickHexagon()
            elif self.currentAssemObj == 'clover_frame':
                self.pickClover()
            # if not self.selfDEMO_btin.isChecked():  ## 0 unchecked otherwise checked
            #     print ('Measured joint values have written to csv file')
            #     ## Measured joint values
            #     self.joint_0_record_np = np.array(self.joint_0_record)
            #     self.joint_1_record_np = np.array(self.joint_1_record)
            #     self.joint_2_record_np = np.array(self.joint_2_record)
            #     self.joint_3_record_np = np.array(self.joint_3_record)
            #     self.joint_4_record_np = np.array(self.joint_4_record)
            #     self.joint_5_record_np = np.array(self.joint_5_record)
            #     self.joint_6_record_np = np.array(self.joint_6_record)
            #     self.save_data = np.column_stack((self.joint_0_record_np,
            #                                       self.joint_1_record_np,
            #                                       self.joint_2_record_np,
            #                                       self.joint_3_record_np,
            #                                       self.joint_4_record_np,
            #                                       self.joint_5_record_np,
            #                                       self.joint_6_record_np
            #                                       ))
            #
            #     if self.currentAssemObj == 'bottle' and os.path.isfile("bottle.csv"):
            #         os.remove("bottle.csv")
            #         np.savetxt("bottle.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'glue' and os.path.isfile("glue.csv"):
            #         os.remove("glue.csv")
            #         np.savetxt("glue.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'cellphone' and os.path.isfile("phone.csv"):
            #         os.remove("phone.csv")
            #         np.savetxt("phone.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'pistonshaft' and os.path.isfile("piston_shaft.csv"):
            #         os.remove("piston_shaft.csv")
            #         np.savetxt("piston_shaft.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.piston_shaft == 'gear' and os.path.isfile("gear.csv"):
            #         os.remove("gear.csv")
            #         np.savetxt("gear.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'square' and os.path.isfile("square.csv"):
            #         os.remove("square.csv")
            #         np.savetxt("square.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'circle' and os.path.isfile("circle.csv"):
            #         os.remove("circle.csv")
            #         np.savetxt("circle.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'hexagon' and os.path.isfile("hexagon.csv"):
            #         os.remove("hexagon.csv")
            #         np.savetxt("hexagon.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'clover' and os.path.isfile("clover.csv"):
            #         os.remove("clover.csv")
            #         np.savetxt("clover.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'square_frame' and os.path.isfile("square_frame.csv"):
            #         os.remove("square_frame.csv")
            #         np.savetxt("square_frame.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'circle_frame' and os.path.isfile("circle_frame.csv"):
            #         os.remove("circle_frame.csv")
            #         np.savetxt("circle_frame.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'hexagon_frame' and os.path.isfile("hexagon_frame.csv"):
            #         os.remove("hexagon_frame.csv")
            #         np.savetxt("hexagon_frame.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     elif self.currentAssemObj == 'clover_frame' and os.path.isfile("clover_frame.csv"):
            #         os.remove("clover_frame.csv")
            #         np.savetxt("clover_frame.csv", self.save_data, delimiter=',', fmt='%.3e')
            #     else:
            #         pass

    def startDemo(self):
        self.pickBottle()
        self.pickShaft()
        self.pickPhone()
        self.pickGear()
        self.pickGlue()
        if self.isBigFramePrepared:
            self.th2.TakeAwayBigframefromSaywer()
            self.isBigFramePrepared = False


    def setTargetObject(self, object=''):
        '''sets target object and crop the RoI
        for passing ober to Grapsing regressor'''
        #
        # self.np_image1 = np.asarray(self.cv_dctimage2)
        # self.RoICoordList = []
        # self.RoICoord = [0.0, 0.0]
        # i = 0
        #
        # for item in self.detectionmsg2:  # loop over all the detected images
        #     self.RoICoord[0] = item.x + item.width / 2
        #     self.RoICoord[1] = item.y + item.height / 2
        #     if self.RoICoord[0] < 0:
        #         self.RoICoord[0] = 0
        #     if self.RoICoord[1] < 0:
        #         self.RoICoord[1] = 0
        #
        #     self.assembleFrameDictList['roi'].append(self.RoICoord[0] - 50,
        #                                              self.RoICoord[1] - 50)  # list containing all the info
        #     self.assembleFrameDictList['class'].append(item.object_class)  # list containing all the info
        #     self.assembleFrameDictList['p'].append(item.p)  # list containing all the info
        #     self.tempItemTuple = (self.assembleFrameDictList['p'][i], self.assembleFrameDictList['roi'][i],
        #                           self.assembleFrameDictList['class'][i])
        #     self.assembleFrameDictList['item'].append(self.tempItemTuple)  # list containing all the info
        #
        #     num_rows, num_cols = self.np_image1.shape
        #
        #     if (self.RoICoord[0] + 100 > num_cols):
        #         self.roiWidth = num_cols - self.RoICoord[0]
        #
        #     if (self.RoICoord[1] + 100 > num_rows):
        #         self.roiHeight = num_cols - self.RoICoord[1]
        #
        #     ### Setup RoI on the image input
        #     ### Refer by search on Google with the following keywords; numpy array, opencv, region of interest
        #
        #     self.np_RoI_img1 = self.np_image1[self.RoICoord[0]:self.roiWidth, self.RoICoord[1]:self.roiHeight]
        #     self.image_grasp1 = self.np_RoI_img1
        #
        #     self.totalObjectListInFrameArea.append(self.tempItemTuple)
        #     self.totalObjectListInFrameArea = sorted(self.totalObjectListInFrameArea, key=itemgetter(0), reverse=True)
        #     if item.object_class == 'square_frame':
        #         self.curDetectedTwoStepFramebyRCNN['square_frame'].append(self.tempItemTuple)
        #         self.curDetectedTwoStepFramebyRCNN['square_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['square_frame'],
        #                                                                key=itemgetter(0), reverse=True)
        #     elif item.object_class == 'circle_frame':
        #         self.curDetectedTwoStepFramebyRCNN['circle_frame'].append(self.tempItemTuple)
        #         self.curDetectedTwoStepFramebyRCNN['circle_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['circle_frame'],
        #                                                              key=itemgetter(0), reverse=True)
        #     elif item.object_class == 'hexagon_frame':
        #         self.curDetectedTwoStepFramebyRCNN['hexagon_frame'].append(self.tempItemTuple)
        #         self.curDetectedTwoStepFramebyRCNN['hexagon_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['hexagon_frame'],
        #                                                               key=itemgetter(0), reverse=True)
        #     elif item.object_class == 'clover_frame':
        #         self.curDetectedTwoStepFramebyRCNN['clover_frame'].append(self.tempItemTuple)
        #         self.curDetectedTwoStepFramebyRCNN['clover_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['clover_frame'], key=itemgetter(0), reverse=True)
        #     elif item.object_class == 'small_frames':
        #         self.curDetectedBaseFramesbyRCNN['small_frames'].append(self.tempItemTuple)
        #         self.curDetectedBaseFramesbyRCNN['small_frames'] = sorted(self.curDetectedBaseFramesbyRCNN['small_frames'],
        #                                                              key=itemgetter(0), reverse=True)
        #     elif item.object_class == 'big_frames':
        #         self.curDetectedBaseFramesbyRCNN['big_frames'].append(item)
        #         self.curDetectedBaseFramesbyRCNN['big_frames'] = sorted(self.curDetectedBaseFramesbyRCNN['big_frames'], key=itemgetter(0), reverse=True)
        #     i += 1
        #     self.image_grasp1 = cv2.resize(self.image_grasp1, None, fx=2.0, fy=2.0)
        #     self.image_grasp1 = cv2.resize(self.image_grasp1, (200, 200))





    def pickPen(self):
        pass
        # self.detectObjects1()
        # self.currentAssemObj = 'pen'
        # self.setTargetObject(object=self.currentAssemObj)
        # if not len(self.curDetectedBaseFramesbyRCNN['big_frames']):
        #     self.Logs.append(_fromUtf8('No baseFrame is detected'))
        #     self.countBFbaseframe += 1
        # if self.countBFbaseframe == 2 or len(self.curDetectedAssemblePartsbyRCNN['pen']):
        #     if not self.isBigFramePrepared:
        #         self.th2.BringBigframe2Saywer()
        #         self.isBigFramePrepared = True
        #     self.countBFbaseframe == 0
        #     count = len(self.curDetectedAssemblePartsbyRCNN['pen'])
        #     self.detectObjects1()
        # 
        #     while not len(self.curDetectedAssemblePartsbyRCNN['pen']):
        #         QApplication.processEvents()
        #         self.detectObjects1()
        #         rospy.sleep(1.0)
        #         if len(self.curDetectedAssemblePartsbyRCNN['pen']):
        #             print ('pen is detected')
        #             break
        # 
        #     if len(self.curDetectedAssemblePartsbyRCNN['pen']):
        #         # self.gripper_off()
        #         # call AlexNet for coordinate transform
        #         print (self.curDetectedAssemblePartsbyRCNN['pen'][0])
        #         self.imgCoord1[0] = self.curDetectedAssemblePartsbyRCNN['pen'][0][1][0]
        #         self.imgCoord1[1] = self.curDetectedAssemblePartsbyRCNN['pen'][0][1][1]
        #         print (self.imgCoord1)
        #         self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
        #         self.init_gripper()
        #         # print (self.camCoord1)
        #         self.robPose1 = self.coordTransform1(self.camCoord1)
        #         print (self.robPose1)
        #         self.th.clearTrajectory()
        #         self.addInitPoseWayPoint()
        #         self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
        #         self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
        #         self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
        #                                 z_offset=self.approachHeight)
        #         self.th.sendTrajectory()
        #         self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
        #                                 y_offset=-0.010,
        #                                 z_offset=0.009, yaw=10.0)
        #         self.th.sendTrajectory()
        #         rospy.sleep(self.wait_before_grasping)
        #         # self.gripper_on(object='pen')
        #         rospy.sleep(self.wait_after_grasping)
        #         self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
        #                                 z_offset=self.retrieveHeight)
        #         self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
        #         self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
        #         self.th.sendTrajectory()
        #         QApplication.processEvents()
        #         rospy.sleep(1.0)
        # 
        #         os.chdir('/home/irobot/Downloads/Cheolhui/')
        #         if os.path.isfile(self.currentAssemObj+".csv"):
        #             self.read_learned_values_from_csv(self.currentAssemObj)
        #         else:
        #             print('no assembly trajectory exist. please make one')
        #         self.countBFbaseframe =0



    def marginAreaApproachScheme(self, camCoord=None):

        roll = 0.0
        pitch = 0.0

        # left = -235
        # right = 298
        # top = -195
        # bottom = 185
        # self.safetyMargin = 80


        margin_list = []

        # camCoord -> PCL value in camera coordinate

        if self.bin_left_margin < camCoord[0] < self.bin_left_margin+self.safetyMarginLR: # resides in left margin area
            margin_list.append('left')
            pitch = 30.0 # in deg
        elif (self.bin_right_margin-self.safetyMarginLR) < camCoord[0] < self.bin_right_margin:
            margin_list.append('right')
            pitch = -30
            pass

        if self.bin_bottom_margin - self.safetyMarginTB < camCoord[1] < self.bin_bottom_margin:
            margin_list.append('bottom')
            roll = -30.0
        elif self.bin_top_margin < camCoord[1] < self.bin_top_margin+self.safetyMarginTB:
            margin_list.append('top')
            roll = 30.0
        else:
            pass

        tempStr = ''

        if len(margin_list):
            for string in margin_list:
                tempStr += '_'
                tempStr += string
            print ('Target object resides in %s' %tempStr)

        else:
            print ('Target object resides in safe area')

        return rad(roll), rad(pitch)


    # def detectionLoop(self):





    def pickBottle(self):



        self.th2.moveToDetectionPose()

        QApplication.processEvents()
        self.Logs.append(_fromUtf8("Detection starts!"))
        # print self.robPose1

        # sleep(3.0)
        QApplication.processEvents()

        rospy.sleep(0.5)
        self.detectObjects1()
        print len(self.totalObjectListInPartArea)
        while len(self.totalObjectListInPartArea):

            print ('Total # of detected objects: %d' % len(self.totalObjectListInPartArea))

            # item_key = randint(0,3)
            # totalObjectListInPartArea[0][5]
            rospy.sleep(0.5)
            self.detectObjects1()
            # item_value = self.dict_object[item_key]
            if  len(self.totalObjectListInPartArea)>=2:
                item_value = self.totalObjectListInPartArea[1][2]

                # if len(self.curDetectedAssemblePartsbyRCNN[item_value]):
                if len(self.totalObjectListInPartArea):
                    QApplication.processEvents()

                    rospy.sleep(0.5)
                    self.detectObjects1()

                    print ('Trying for %s' %item_value)

                    count = 0
                    # while not len(self.curDetectedAssemblePartsbyRCNN[item_value]):
                    #     QApplication.processEvents()
                    #     self.th2.moveToDetectionPose()
                    #     rospy.sleep(0.1)
                    #     self.detectObjects1()
                    #
                    #     if len(self.curDetectedAssemblePartsbyRCNN[item_value]):
                    #         print ('%s is detected' %item_value)
                    #         break
                    #     elif count == 2:
                    #         break
                    #     count +=1
                   # if True:
                    # if len(self.curDetectedAssemblePartsbyRCNN[item_value]):
                    if len(self.totalObjectListInPartArea):
                        print('!!')
                        # call AlexNet for coordinate transform
                        # print (self.curDetectedAssemblePartsbyRCNN['bottle'][0])
                        # self.imgCoord1[0] = self.curDetectedAssemblePartsbyRCNN['bottle'][0][1][0]
                        # self.imgCoord1[1] = self.curDetectedAssemblePartsbyRCNN['bottle'][0][1][1]
                        # self.camCoordPcl = self.curDetectedAssemblePartsbyRCNN[item_value][0][4] # fifth element of the tuple



                        self.camCoordPcl = self.totalObjectListInPartArea[0][4] # fifth element of the tuple
                        # rospy.loginfo("Img_coord X: %.3f Y: %.3f",self.camCoord1[0], self.camCoord1[1])

                        # self.camCoord1[2] = camCoordDepth
                        # print (self.imgCoord1)
                        # get  X, Y, theta from Grasping estimator
                        # self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(imgCoord=self.imgCoord1, depth=camCoordDepth)

                        self.robPose1 = self.coordTransform1(self.camCoordPcl) # in ee coord
                        print self.robPose1
                        # print('!!!!')
                        if self.robPose1[2] >= 10.0:
                            # tool_rotation = self.curDetectedAssemblePartsbyRCNN[item_value][0][5]
                            tool_rotation = self.totalObjectListInPartArea[1][5]

                            rospy.loginfo("Cam_coordPcl X: %.3f Y: %.3f || Rob_coord X: %.3f Y: %.3f Z: %.3f || Rotation: %.3f (Deg) ",
                            self.camCoordPcl[0], self.camCoordPcl[1], self.robPose1[0], self.robPose1[1], self.robPose1[2], tool_rotation)

                            roll, pitch = self.marginAreaApproachScheme(self.camCoordPcl)


                            self.th2.moveToDetectionPose_tool(position=self.robPose1, yaw=tool_rotation)

                            QApplication.processEvents()


                            # # or only work with orientation part
                            # o = robot.get_orientation()
                            # o.rotate_yb(pi)
                            # robot.set_orientation(o)
                            # self.th2.move_z(trans_z=-self.urGraspingHeight - self.object_approach_metric['bottle']) # 0.15 down
                            gripper_width = 0
                            decay = 0.85
                            if item_value == 'bottle':
                                gripper_width = 1.7*decay
                                print ('gripper opens about 2')

                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box1)  # move to place
                            elif item_value == 'pen':
                                gripper_width = 1.5*decay
                                print ('gripper opens about 1.5')

                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box2)
                            elif item_value == 'gear':
                                gripper_width = 3*decay
                                print ('gripper opens about 3')

                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box3)
                            elif item_value == 'piston shaft' or item_value == 'blue opener':
                                gripper_width = 1.5*decay
                                print ('gripper opens about 1.5')

                            elif item_value == 'cell phone':
                                gripper_width = 3.5*decay
                                print ('gripper opens about 3.5')
                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box4)
                            elif item_value == 'brown dog' or item_value == 'pink pig' or item_value == '(blue pig)' or item_value == 'toy car':
                                gripper_width = 3.5 * decay
                                print ('gripper opens about 3.5')
                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box4)
                            elif item_value == 'yellow cup' or item_value == '(remote controller)' or item_value == 'purple cup' or item_value == 'remote controller':
                                gripper_width = 2.0 * decay
                                print ('gripper opens about 2.5')
                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box4)
                            elif item_value == 'brush' or item_value == 'sauce bottle' or item_value == '(toothpaste)':
                                gripper_width = 2.5 * decay
                                print ('gripper opens about 2.5')
                                # self.th2.moveToDetectionPose_base(position=self.th2.target_box4)
                            elif item_value == 'gun' or item_value == 'baseball':
                                gripper_width = 2.8 * decay
                                print ('gripper opens about 2.5')
                            else:
                                gripper_width = 3.5*decay
                                print ('gripper opens about 3.5')


                            self.th2.robotiq_openclose(gripper_width) # modifiy the width

                            ## Detection / Tracking loop


                            if item_value == 'pen':
                                self.th2.move_z(trans_z=-self.urGraspingHeight+0.015) # 0.15 down

                            elif item_value == 'piston shaft':
                                self.th2.move_z(trans_z=-self.urGraspingHeight+0.012) # 0.15 down

                            elif item_value == 'cell phone':
                                self.th2.move_z(trans_z=-self.urGraspingHeight+0.01) # 0.15 down

                            elif item_value == 'gear':
                                self.th2.move_z(trans_z=-self.urGraspingHeight + 0.016)  # 0.15 down

                            else: # bottle
                                self.th2.move_z(trans_z=-self.urGraspingHeight - 0.01) # 0.15 down

                            QApplication.processEvents()

                            gripper_width += 0.4
                            self.th2.robotiq_openclose(gripper_width) # expand
                            rospy.sleep(0.3)

                            gripper_width -= 0.35
                            self.th2.robotiq_openclose(gripper_width)  # expand
                            rospy.sleep(0.2)

                            # joints = self.th2.ur5.get_j()  # get current transformation matrix (tool to base)
                            # joints[5] +=pi/4
                            # self.th2.ur5.movej([0, 0, 0, 0, 0, pi / 6], self.th2.joint_accel, self.th2.joint_vel,
                            #                    relative=True)  # apply the new pose
                            # rospy.sleep(0.05)
                            # self.th2.ur5.movej([0, 0, 0, 0, 0, -pi / 3], self.th2.joint_accel, self.th2.joint_vel,
                            #                    relative=True)  # apply the new pose
                            # rospy.sleep(0.05)
                            # self.th2.ur5.movej([0, 0, 0, 0, 0, pi / 6], self.th2.joint_accel, self.th2.joint_vel,
                            #                    relative=True)  # apply the new pose

                            rospy.sleep(0.1)

                            # gripper_width += 0.4
                            # self.th2.robotiq_openclose(gripper_width)  # expand
                            # rospy.sleep(0.3)

                            self.th2.robotiq_openclose(0) # grasp
                            QApplication.processEvents()
                            rospy.sleep(1.0)

                            self.th2.move_z(trans_z=self.urRetrievingHeight) # 0.25 up
                            QApplication.processEvents()

                            # trans = self.th2.ur5.get_pose()  # get current transformation matrix (tool to base)
                            # trans.orient.rotate_xb(-roll)
                            # trans.orient.rotate_yb(-pitch)
                            # self.th2.ur5.set_pose(trans, self.th2.lin_accel, self.th2.lin_vel)  # apply the new pose

                            rospy.sleep(0.5)

                            if abs(self.th2.gripper_pos_stat) <= 0.005: # grasping failed
                                print ('Grasping failed')
                                self.th2.robotiq_openclose(gripper_width)
                                pass
                            else:
                                print ('Grasping succeeded')

                                # self.th2.moveToDetectionPose_base(position=self.th2.bin_viaPose)
                                # command, pose_list, acc = 0.01, vel = 0.01, radius = 0.01, wait = True,
                                self.th2.ur5.movexs(pose_list = self.th2.bin_viaPose_list, acc = self.th2.lin_accel, vel = self.th2.lin_vel,
                                                    radius= 0.10, command = 'movel')
                                # QApplication.processEvents()

                                if item_value in self.item_target_box['box1']:
                                    gripper_width = 3.5
                                    print ('gripper opens about 3.5')
                                    # self.th2.moveToDetectionPose_base(position=self.th2.target_box1)  # move to place
                                    self.th2.moveToDetectionPose_base(position=self.th2.target_box1)  # move to place
                                elif item_value in self.item_target_box['box2']:
                                    gripper_width = 3.5
                                    print ('gripper opens about 3.5')
                                    self.th2.moveToDetectionPose_base(position=self.th2.target_box2)
                                elif item_value in self.item_target_box['box3']:
                                    gripper_width = 3.5
                                    print ('gripper opens about 3.5')
                                    self.th2.moveToDetectionPose_base(position=self.th2.target_box3)
                                else:
                                    gripper_width = 3.5
                                    print ('gripper opens about 3.5')
                                    # self.th2.moveToDetectionPose_base(position=self.th2.target_box1)  # move to place
                                    self.th2.moveToDetectionPose_base(position=self.th2.target_box1)  # move to place


                                # QApplication.processEvents()
                                self.th2.move_z(trans_z=-self.urPlacingHeight)
                                QApplication.processEvents()

                                self.th2.robotiq_openclose(gripper_width) # places
                                rospy.sleep(0.5)
                                # QApplication.processEvents()
                                self.th2.ur5.movexs(pose_list = self.th2.bin_viaPose_list_reversed,acc =  self.th2.lin_accel, vel = self.th2.lin_vel,
                                                    radius=0.10, command = 'movel')
                                self.th2.moveToDetectionPose()
                                QApplication.processEvents()

                    else:
                        pass
                QApplication.processEvents()
                self.th2.moveToDetectionPose()
                rospy.sleep(0.5)
                self.detectObjects1()

    def pickGear(self):
        self.detectObjects1()
        self.currentAssemObj = 'gear'
        if not len(self.curDetectedBaseFramesbyRCNN['big_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedAssemblePartsbyRCNN['gear']):
            if not self.isBigFramePrepared:
                self.th2.BringBigframe2Saywer()
                self.isBigFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedAssemblePartsbyRCNN['gear'])
            self.detectObjects1()

            while not len(self.curDetectedAssemblePartsbyRCNN['gear']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedAssemblePartsbyRCNN['gear']):
                    print ('gear is detected')
                    break

            if len(self.curDetectedAssemblePartsbyRCNN['gear']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedAssemblePartsbyRCNN['gear'][0])
                self.imgCoord1[0] = self.curDetectedAssemblePartsbyRCNN['gear'][0][1][0]
                self.imgCoord1[1] = self.curDetectedAssemblePartsbyRCNN['gear'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.approachHeight)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.015)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='gear')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickGlue(self):
        self.detectObjects1()
        self.currentAssemObj = 'glue'
        if not len(self.curDetectedBaseFramesbyRCNN['big_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedAssemblePartsbyRCNN['glue']):
            if not self.isBigFramePrepared:
                self.th2.BringBigframe2Saywer()
                self.isBigFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedAssemblePartsbyRCNN['glue'])
            self.detectObjects1()

            while not len(self.curDetectedAssemblePartsbyRCNN['glue']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedAssemblePartsbyRCNN['glue']):
                    print ('piston_shaft is detected')
                    break

            if len(self.curDetectedAssemblePartsbyRCNN['glue']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedAssemblePartsbyRCNN['glue'][0])
                self.imgCoord1[0] = self.curDetectedAssemblePartsbyRCNN['glue'][0][1][0]
                self.imgCoord1[1] = self.curDetectedAssemblePartsbyRCNN['glue'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.010,
                                        z_offset=0.009, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='glue')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickPhone(self):
        self.detectObjects1()
        self.currentAssemObj = 'cellphone'
        if not len(self.curDetectedBaseFramesbyRCNN['big_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedAssemblePartsbyRCNN['cellphone']):
            if not self.isBigFramePrepared:
                self.th2.BringBigframe2Saywer()
                self.isBigFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedAssemblePartsbyRCNN['cellphone'])
            self.detectObjects1()

            while not len(self.curDetectedAssemblePartsbyRCNN['cellphone']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedAssemblePartsbyRCNN['cellphone']):
                    print ('piston_shaft is detected')
                    break

            if len(self.curDetectedAssemblePartsbyRCNN['cellphone']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedAssemblePartsbyRCNN['cellphone'][0])
                self.imgCoord1[0] = self.curDetectedAssemblePartsbyRCNN['cellphone'][0][1][0]
                self.imgCoord1[1] = self.curDetectedAssemblePartsbyRCNN['cellphone'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025, y_offset=-0.007,
                                        z_offset=-0.003, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # resp= self.gripper_on(object='cellphone')
                rospy.sleep(self.wait_after_grasping)
                if(resp):
                    self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                            z_offset=self.retrieveHeight)
                    self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                    self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                    self.th.sendTrajectory()
                    QApplication.processEvents()
                    rospy.sleep(1.0)
                    os.chdir('/home/irobot/Downloads/Cheolhui/')
                    if os.path.isfile(self.currentAssemObj + ".csv"):
                        self.read_learned_values_from_csv(self.currentAssemObj)
                    else:
                        print('no assembly trajectory exist. please make one')
                    self.countBFbaseframe = 0
                else:
                    self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                            y_offset=-0.014, z_offset=self.approachHeight, yaw=10.0)
                    self.th.sendTrajectory()
                    QApplication.processEvents()
                    self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                            y_offset=-0.014,
                                            z_offset=-0.003, yaw=10.0)
                    self.th.sendTrajectory()
                    rospy.sleep(self.wait_before_grasping)
                    # resp = self.gripper_on(object='cellphone')
                    if (resp):
                        self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                                z_offset=self.retrieveHeight)
                        self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                        self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        rospy.sleep(1.0)
                        os.chdir('/home/irobot/Downloads/Cheolhui/')
                        if os.path.isfile(self.currentAssemObj + ".csv"):
                            self.read_learned_values_from_csv(self.currentAssemObj)
                        else:
                            print('no assembly trajectory exist. please make one')
                        self.countBFbaseframe = 0
                    else:
                        self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                                y_offset=-0.004, z_offset=self.approachHeight, yaw=10.0)
                        self.th.sendTrajectory()
                        QApplication.processEvents()
                        self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                                y_offset=-0.004,
                                                z_offset=-0.003, yaw=10.0)
                        self.th.sendTrajectory()
                        rospy.sleep(self.wait_before_grasping)
                        # resp = self.gripper_on(object='cellphone')
                        if(resp):
                            self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                                    z_offset=self.retrieveHeight)
                            self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                            self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                            self.th.sendTrajectory()
                            QApplication.processEvents()
                            rospy.sleep(1.0)
                            os.chdir('/home/irobot/Downloads/Cheolhui/')
                            if os.path.isfile(self.currentAssemObj + ".csv"):
                                self.read_learned_values_from_csv(self.currentAssemObj)
                            else:
                                print('no assembly trajectory exist. please make one')
                            self.countBFbaseframe = 0
    def pickShaft(self):

        self.detectObjects1()
        self.currentAssemObj = 'pistonshaft'
        if not len(self.curDetectedBaseFramesbyRCNN['big_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedAssemblePartsbyRCNN['pistonshaft']):
            if not self.isBigFramePrepared:
                self.th2.BringBigframe2Saywer()
                self.isBigFramePrepared = True

            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedAssemblePartsbyRCNN['pistonshaft'])
            self.detectObjects1()

            while not len(self.curDetectedAssemblePartsbyRCNN['pistonshaft']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedAssemblePartsbyRCNN['pistonshaft']):
                    print ('piston_shaft is detected')
                    break

            if len(self.curDetectedAssemblePartsbyRCNN['pistonshaft']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedAssemblePartsbyRCNN['pistonshaft'][0])
                self.imgCoord1[0] = self.curDetectedAssemblePartsbyRCNN['pistonshaft'][0][1][0]
                self.imgCoord1[1] = self.curDetectedAssemblePartsbyRCNN['pistonshaft'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.approachHeight, yaw = 10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset= 0.025, z_offset=0.005,yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='pistonshaft')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0


    def pickCircle(self):
        self.detectObjects1()
        self.currentAssemObj = 'circle'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepPartsbyRCNN['circle']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepPartsbyRCNN['circle'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepPartsbyRCNN['circle']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepPartsbyRCNN['circle']):
                    print ('circle is detected')
                    break

            if len(self.curDetectedTwoStepPartsbyRCNN['circle']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepPartsbyRCNN['circle'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepPartsbyRCNN['circle'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepPartsbyRCNN['circle'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.021, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='circle')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickCircle_frame(self):
        self.detectObjects1()
        self.currentAssemObj = 'circle_frame'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepFramebyRCNN['circle_frame']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepFramebyRCNN['circle_frame'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepFramebyRCNN['circle_frame']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepFramebyRCNN['circle_frame']):
                    print ('circle_frame is detected')
                    break

            if len(self.curDetectedTwoStepFramebyRCNN['circle_frame']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepFramebyRCNN['circle_frame'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepFramebyRCNN['circle_frame'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepFramebyRCNN['circle_frame'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.015, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='circle_frame')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickHexagon(self):
        self.detectObjects1()
        self.currentAssemObj = 'hexagon'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepPartsbyRCNN['hexagon']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepPartsbyRCNN['hexagon'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepPartsbyRCNN['hexagon']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepPartsbyRCNN['hexagon']):
                    print ('hexagon is detected')
                    break

            if len(self.curDetectedTwoStepPartsbyRCNN['hexagon']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepPartsbyRCNN['hexagon'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepPartsbyRCNN['hexagon'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepPartsbyRCNN['hexagon'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.021, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='hexagon')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickHexagon_frame(self):
        self.detectObjects1()
        self.currentAssemObj = 'hexagon_frame'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepFramebyRCNN['hexagon_frame']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepFramebyRCNN['hexagon_frame'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepFramebyRCNN['hexagon_frame']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepFramebyRCNN['hexagon_frame']):
                    print ('hexagon_frame is detected')
                    break

            if len(self.curDetectedTwoStepFramebyRCNN['hexagon_frame']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepFramebyRCNN['hexagon_frame'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepFramebyRCNN['hexagon_frame'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepFramebyRCNN['hexagon_frame'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.015, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='hexagon_frame')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickSquare(self):
        self.detectObjects1()
        self.currentAssemObj = 'square'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepPartsbyRCNN['square']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepPartsbyRCNN['square'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepPartsbyRCNN['square']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepPartsbyRCNN['square']):
                    print ('square is detected')
                    break

            if len(self.curDetectedTwoStepPartsbyRCNN['square']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepPartsbyRCNN['square'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepPartsbyRCNN['square'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepPartsbyRCNN['square'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.021, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='square')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0



    def pickSquare_frame(self):
        self.detectObjects1()
        self.currentAssemObj = 'square_frame'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepFramebyRCNN['square_frame']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepFramebyRCNN['square_frame'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepFramebyRCNN['square_frame']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepFramebyRCNN['square_frame']):
                    print ('square_frame is detected')
                    break

            if len(self.curDetectedTwoStepFramebyRCNN['square_frame']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepFramebyRCNN['square_frame'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepFramebyRCNN['square_frame'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepFramebyRCNN['square_frame'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset= -0.007, z_offset= self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.015, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='square_frame')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0

    def pickClover(self):
        self.detectObjects1()
        self.currentAssemObj = 'clover'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepPartsbyRCNN['clover']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepPartsbyRCNN['clover'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepPartsbyRCNN['clover']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepPartsbyRCNN['clover']):
                    print ('clover is detected')
                    break

            if len(self.curDetectedTwoStepPartsbyRCNN['clover']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepPartsbyRCNN['clover'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepPartsbyRCNN['clover'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepPartsbyRCNN['clover'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset=-0.007, z_offset=self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.021, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='clover')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0



    def pickClover_frame(self):
        self.detectObjects1()
        self.currentAssemObj = 'clover_frame'
        if not len(self.curDetectedBaseFramesbyRCNN['small_frames']):
            self.Logs.append(_fromUtf8('No baseFrame is detected'))
            self.countBFbaseframe += 1
        if self.countBFbaseframe == 2 or len(self.curDetectedTwoStepFramebyRCNN['clover_frame']):
            if not self.isSmallFramePrepared:
                self.th2.BringSmallframe2Saywer()
                self.isSmallFramePrepared = True
            # self.th2.TakeAwayBigframefromSaywer()
            # self.th2.BringSmallframe2Saywer()
            # self.th2.TakeAwaySmallframefromSaywer()
            self.countBFbaseframe == 0
            # after putting base frame, picks up bottle
            count = len(self.curDetectedTwoStepFramebyRCNN['clover_frame'])
            self.detectObjects1()

            while not len(self.curDetectedTwoStepFramebyRCNN['clover_frame']):
                QApplication.processEvents()
                self.detectObjects1()
                rospy.sleep(1.0)
                if len(self.curDetectedTwoStepFramebyRCNN['clover_frame']):
                    print ('clover_frame is detected')
                    break

            if len(self.curDetectedTwoStepFramebyRCNN['clover_frame']):
                self.init_gripper()
                # self.gripper_off()
                # call AlexNet for coordinate transform
                print (self.curDetectedTwoStepFramebyRCNN['clover_frame'][0])
                self.imgCoord1[0] = self.curDetectedTwoStepFramebyRCNN['clover_frame'][0][1][0]
                self.imgCoord1[1] = self.curDetectedTwoStepFramebyRCNN['clover_frame'][0][1][1]
                print (self.imgCoord1)
                self.camCoord1[0], self.camCoord1[1] = self.frameTFimageToCam1(self.imgCoord1)
                print (self.camCoord1)
                self.robPose1 = self.coordTransform1(self.camCoord1)
                print (self.robPose1)
                self.th.clearTrajectory()
                self.addInitPoseWayPoint()
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        y_offset= -0.007, z_offset= self.approachHeight, yaw=10.0)
                self.th.sendTrajectory()
                QApplication.processEvents()
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2], x_offset=0.025,
                                        y_offset=-0.007, z_offset=0.015, yaw=10.0)
                self.th.sendTrajectory()
                rospy.sleep(self.wait_before_grasping)
                # self.gripper_on(object='clover_frame')
                rospy.sleep(self.wait_after_grasping)
                self.addMotionWayPoints(self.robPose1[0], self.robPose1[1], self.robPose1[2],
                                        z_offset=self.retrieveHeight)
                self.addMotionWayPoints(self.viaPos2['x'], self.viaPos2['y'], self.viaPos2['z'])
                self.addMotionWayPoints(self.viaPos1['x'], self.viaPos1['y'], self.viaPos1['z'])
                self.th.sendTrajectory()
                QApplication.processEvents()
                rospy.sleep(1.0)
                os.chdir('/home/irobot/Downloads/Cheolhui/')
                if os.path.isfile(self.currentAssemObj + ".csv"):
                    self.read_learned_values_from_csv(self.currentAssemObj)
                else:
                    print('no assembly trajectory exist. please make one')
                self.countBFbaseframe = 0




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

            ### read_force_torque_info.     ### wrench = {'force':(x,y,z), 'torque':(x,y,z)}

            self.endpoint_ft_dict = self.th.limb.endpoint_effort()
            self.Fx_read.setText(QtCore.QString.number(self.endpoint_ft_dict['force'][0], 'f', 3))
            self.Fy_read.setText(QtCore.QString.number(self.endpoint_ft_dict['force'][1], 'f', 3))
            self.Fz_read.setText(QtCore.QString.number(self.endpoint_ft_dict['force'][2], 'f', 3))
            self.Mx_read.setText(QtCore.QString.number(self.endpoint_ft_dict['torque'][0], 'f', 3))
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
            self.Y_read.setText(QtCore.QString.number(self.endpoint_position[1] * 1000.0, 'f', 3))
            self.Z_read.setText(QtCore.QString.number(self.endpoint_position[2] * 1000.0, 'f', 3))
            ### Use tf3d to convert from quaternion to RPY
            # list (Rx,Ry,Rz)
            ### give the values to GUI
            self.Rx_read.setText(QtCore.QString.number(deg(self.endpoint_rotation[2]), 'f', 3))
            self.Ry_read.setText(QtCore.QString.number(deg(self.endpoint_rotation[1]), 'f', 3))
            self.Rz_read.setText(QtCore.QString.number(deg(self.endpoint_rotation[0]), 'f', 3))


    def read_status_from_UR_monitor(self):
            '''
            self.ur_monitor
             def q_actual(self, wait=False, timestamp=False):
                    """ Get the actual joint position vector."""
                    if wait:
                        self.wait()
                    with self._dataAccess:
                        if timestamp:
                            return self._timestamp, self._qActual
                        else:
                            return self._qActual
                getActual = q_actual
             def tcf_pose(self, wait=False, timestamp=False, ctrlTimestamp=False):
                    """ Return the tool pose values."""
                    if wait:
                        self.wait()
                    with self._dataAccess:
                        tcf = self._tcp
                        if ctrlTimestamp or timestamp:
                            ret = [tcf]
                            if timestamp:
                                ret.insert(-1, self._timestamp)
                            if ctrlTimestamp:
                                ret.insert(-1, self._ctrlTimestamp)
                            return ret
                        else:
                            return tcf
                getTCF = tcf_pose

                def tcf_force(self, wait=False, timestamp=False):
                    """ Get the tool force. The returned tool force is a
                    six-vector of three forces and three moments."""
                    if wait:
                        self.wait()
                    with self._dataAccess:
                        # tcf = self._fwkin(self._qActual)
                        tcf_force = self._tcp_force
                        if timestamp:
                            return self._timestamp, tcf_force
                        else:
                            return tcf_force
                getTCFForce = tcf_force
                        q_actual
                        array([ 0.64530599, -1.21616489, -1.91338569, -1.58366424,  1.57153821,
                                0.29135299])
                        tcf_pose
                        array([ 0.33829209,  0.11561653,  0.16878469,  2.57603431, -1.79439473,
                                0.00676484])
                        tcf_force
                        array([ 2.08080521e+01,  7.33893027e+00,  5.77083169e+01, -4.12075738e+00,
                                9.02650453e+00,  3.95421018e-02])

            '''
            global isEnabled
            if isEnabled:

                ur_tcf_info = self.th2.ur_monitor.tcf_pose()
                ur_joint_info = self.th2.ur_monitor.q_actual()

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


                self.X_UR5.setText(QtCore.QString.number(ur_tcf_info[0] * 1000.0, 'f', 3))
                self.Y_UR5.setText(QtCore.QString.number(ur_tcf_info[1] * 1000.0, 'f', 3))
                self.Z_UR5.setText(QtCore.QString.number(ur_tcf_info[2] * 1000.0, 'f', 3))
                ### Use tf3d to convert from quaternion to RPY
                # list (Rx,Ry,Rz)
                ### give the values to GUI
                self.Rx_UR5.setText(QtCore.QString.number(ur_tcf_info[3], 'f', 3))
                self.Ry_UR5.setText(QtCore.QString.number(ur_tcf_info[4], 'f', 3))
                self.Rz_UR5.setText(QtCore.QString.number(ur_tcf_info[5], 'f', 3))

                self.J1_UR5.setText(QtCore.QString.number(deg(ur_joint_info[0]), 'f', 3))
                self.J2_UR5.setText(QtCore.QString.number(deg(ur_joint_info[1]), 'f', 3))
                self.J3_UR5.setText(QtCore.QString.number(deg(ur_joint_info[2]), 'f', 3))
                self.J4_UR5.setText(QtCore.QString.number(deg(ur_joint_info[3]), 'f', 3))
                self.J5_UR5.setText(QtCore.QString.number(deg(ur_joint_info[4]), 'f', 3))
                self.J6_UR5.setText(QtCore.QString.number(deg(ur_joint_info[5]), 'f', 3))




    def mask_rcnnDetction(self, data):
        pass
        # self.detectionfull1 = data

    def rcnnDetection1(self, data):  #  @ assembly area

        # process the bounding box to pass over to AlexNet
        self.detectionfull1 = data

    def detectObjects1(self):

        self.totalObjectListInPartArea = []
        self.curDetectedAssemblePartsbyRCNN['bottle'] = []
        self.curDetectedAssemblePartsbyRCNN['glue'] = []
        self.curDetectedAssemblePartsbyRCNN['cell phone'] = []
        self.curDetectedAssemblePartsbyRCNN['piston shaft'] = []
        self.curDetectedAssemblePartsbyRCNN['gear'] = []
        self.curDetectedAssemblePartsbyRCNN['pen'] = []
        self.curDetectedAssemblePartsbyRCNN['else'] = []

        self.curDetectedTwoStepPartsbyRCNN['square'] = []
        self.curDetectedTwoStepPartsbyRCNN['circle'] = []
        self.curDetectedTwoStepPartsbyRCNN['hexagon'] = []
        self.curDetectedTwoStepPartsbyRCNN['clover'] = []
        self.curDetectedTwoStepFramebyRCNN['square_frame'] = []
        self.curDetectedTwoStepFramebyRCNN['circle_frame'] = []
        self.curDetectedTwoStepFramebyRCNN['hexagon_frame'] = []
        self.curDetectedTwoStepFramebyRCNN['clover_frame'] = []


        self.targetObjAreaDictList = {'p': [], 'class': [], 'roi': [], 'item': [], 'image': [], 'pcl_xyz': [], 'theta': [], 'depth': []}

        '''
            Header header
            
            # The image containing the detetions
            sensor_msgs/Image image
            
            # binary images containing masks
            sensor_msgs/Image[] masks
            
            # The array containing all the detections
            DetectionArray detections
        '''

        self.detectionmsg1 = self.detectionfull1.detections.data  # there could be multiple of detected objects, 'data' is list type
        self.mask_imgs = self.detectionfull1.masks  # there could be multiple of detected objects, 'data' is list type

        self.detectionimg1 = self.detectionfull1.image

        self.cv_dctimage1 = self.bridge.imgmsg_to_cv2(self.detectionimg1, "bgr8")  # 1280 * 1024
        # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/aaa.png', self.cv_dctimage1)

        self.cv_dspimage1 = cv2.cvtColor(self.cv_dctimage1, cv2.COLOR_BGR2RGB)  # 1280 * 1024




        self.np_image1 = self.cv_dctimage1
        self.RoICoordList = []
        self.RoICoord = [0.0, 0.0]
        i = 0

        for item, mask in zip(self.detectionmsg1, self.mask_imgs):  # loop over all the detected images

            if item.cam_x <= self.bin_right_margin and item.cam_x >= self.bin_left_margin\
                    and item.cam_y <= self.bin_bottom_margin and item.cam_y >= self.bin_top_margin:

                self.RoICoord[0] = item.x + item.width / 2
                self.RoICoord[1] = item.y + item.height / 2
                if self.RoICoord[0] - 112< 0: # cropped box out of boundary
                    self.RoICoord[0] = 113
                if self.RoICoord[1] - 112< 0:
                    self.RoICoord[1] = 113


                # apply the scaling factor on the PCL coordinates
                scaling_factor = 1.35
                cam_x = item.cam_x/scaling_factor
                cam_y = item.cam_y/scaling_factor
                cam_z = item.cam_z

                cam_coord_tuple = (cam_x, cam_y, cam_z)


                num_rows, num_cols, channel = self.np_image1.shape

                delta_width = self.RoICoord[0] + 112 - num_cols
                delta_height = self.RoICoord[1] + 112 - num_rows

                if delta_width > 0: # If RoI is biased, to the right
                    self.RoICoord[0] = self.RoICoord[0] - delta_width-1 # set the width

                if delta_height > 0:
                    self.RoICoord[1] = self.RoICoord[1] - delta_height-1

                x_trunc = float("{0:.2f}".format(self.RoICoord[0]))

                x_str = str(x_trunc)



                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'y.png', self.cv_dspimage1)
                #
                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'z.png', self.cv_dctimage1)
                #
                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'x.png', self.np_image1)



                ### Setup RoI on the image input
                ### Refer by search on Google with the following keywords; numpy array, opencv, region of interest


                self.np_RoI_img1 = np.ones((720,1280))

                self.np_RoI_img1 = np.copy(self.np_image1[self.RoICoord[1]-self.roiHeight/2:self.RoICoord[1] + self.roiHeight/2,
                                                self.RoICoord[0]-self.roiWidth/2:self.RoICoord[0]+self.roiWidth/2,:])


                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'a.png', self.np_RoI_img1)





                self.mask_img = self.bridge.imgmsg_to_cv2(mask, "mono8")  # 1280 * 1024

                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'b.png', self.mask_img)

                self.cropped_mask = self.mask_img[self.RoICoord[1]-self.roiHeight/2:self.RoICoord[1] + self.roiHeight/2,
                                                self.RoICoord[0]-self.roiWidth/2:self.RoICoord[0]+self.roiWidth/2]

                ## Let's apply PCA!

                y, x =np.nonzero(self.cropped_mask)

                x = x - np.mean(x)
                y = y - np.mean(y)

                coords = np.vstack([x, y])

                self.cov_mat = np.cov(coords)

                evals, evecs =np.linalg.eig(self.cov_mat)

                sort_indices = np.argsort(evals)[::-1]

                x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
                x_v2, y_v2 = evecs[:, sort_indices[1]]

                tan_minor_axis = y_v2/x_v2

                rad_ang = atan(tan_minor_axis)

                deg_ang = deg(rad_ang)



                scale = 50
                plt.plot(x, y, 'k.')
                plt.plot([x_v1 * -scale * 2, x_v1 * scale * 2],
                         [y_v1 * -scale * 2, y_v1 * scale * 2], color='red')
                plt.plot([x_v2 * -scale, x_v2 * scale],
                         [y_v2 * -scale, y_v2 * scale], color='blue')
                plt.axis('equal')
                plt.gca().invert_yaxis()  # Match the image system with origin at top left
                # plt.show()

                # tan_lower = y_v1/x_v1
                #
                # theta =


                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'c.png', self.cropped_mask)


                self.cropped_mask_scaled = 255 * self.mask_img[self.RoICoord[1]-self.roiHeight/2:self.RoICoord[1] + self.roiHeight/2,
                                                self.RoICoord[0]-self.roiWidth/2:self.RoICoord[0]+self.roiWidth/2]

                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'d.png', self.cropped_mask_scaled)


                self.inversed_mask = 255*np.ones((224,224)) - self.cropped_mask_scaled

                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'e.png', self.inversed_mask)


                for j in range(3):
                    self.np_RoI_img1[:,:,j] = np.multiply(self.np_RoI_img1[:,:,j], self.cropped_mask)

                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'f.png', self.np_RoI_img1)
                #





                for k in range(3):
                    self.np_RoI_img1[:, :, k] = np.add(self.inversed_mask, self.np_RoI_img1[:,:,k])
                # the backgrounds are 0 (black)
                # now make it white

                # cv2.imwrite('/home/irobot2/Downloads/Cheolhui/'+x_str+'g.png', self.np_RoI_img1)


                # ratio = 112/75
                #
                # zoom1 = cv2.resize(img, (width * 2, height * 2), interpolation=cv2.INTER_CUBIC)
                #
                # print self.np_RoI_img1.shape
                # self.np_RoI_img1 = cv2.resize(self.np_RoI_img1,( 224, 224),interpolation=cv2.INTER_CUBIC)


                self.image_grasp1 = self.np_RoI_img1


                # cv2.imshow('1stCam', self.image_grasp1)


                # print (self.RoICoord)
                # print (self.image_grasp1.shape)

                # Send the image_grasp to grasping estimator and receive the resultant X, Y and theta

                self.img_msg1 = self.bridge.cv2_to_imgmsg(self.image_grasp1, "bgr8")

                self.croppedRoIPub.publish(self.img_msg1)

                # print self.RoICoord

                sleep(0.15)
                self.topleftCoord = [self.RoICoord[0]-self.roiWidth/2, self.RoICoord[1]-self.roiHeight/2]
                # Once published, acquire the subscribed info
                graspingPose = self.getGrapsingPose(topleftCoord=self.topleftCoord) # calibrate the X, Y, and theta to the camera coord
                #
                h = item.height
                h = float(h)
                w = item.width
                w = float(w)
                tangent = float(h/w)
                rotation_rad =  atan(tangent)
                rotation_deg = deg(rotation_rad)
                # graspingPose[2] = rotation_deg + 90.0

                graspingPose[2] = deg_ang


                if graspingPose[2] >= 120:
                    graspingPose[2] -= 180
                elif graspingPose[2] <= -120:
                    graspingPose[2] += 180

                ## Temporal calculation of grasping pose
                ## calculate from width and height of the RoI bounding boxes
                ##

                constant = 1.0
                # print graspingPose
                # self.targetObjAreaDictList['roi'].append([self.RoICoord[0] - 50,
                #                                          self.RoICoord[1] - 50])  # list containing all the info
                # self.targetObjAreaDictList['roi'].append([graspingPose[0],
                #                                          graspingPose[1]])  # list containing all the info
                self.targetObjAreaDictList['roi'].append([constant*self.RoICoord[0],
                                                          constant *self.RoICoord[1]])  # list containing all the info
                self.targetObjAreaDictList['class'].append(item.object_class)  # list containing all the info
                self.targetObjAreaDictList['p'].append(item.p)  # list containing all the info
                self.targetObjAreaDictList['image'].append(self.image_grasp1)  # list containing all the info
                self.targetObjAreaDictList['pcl_xyz'].append(cam_coord_tuple)  # list containing all the info
                self.targetObjAreaDictList['theta'].append(graspingPose[2])  # list containing all the info
                self.targetObjAreaDictList['depth'].append(cam_z)  # list containing all the info

                self.tempItemTuple = (self.targetObjAreaDictList['p'][i], self.targetObjAreaDictList['roi'][i],
                                      self.targetObjAreaDictList['class'][i],self.targetObjAreaDictList['image'][i],
                                      self.targetObjAreaDictList['pcl_xyz'][i], self.targetObjAreaDictList['theta'][i],
                                      self.targetObjAreaDictList['depth'][i])

                self.targetObjAreaDictList['item'].append(self.tempItemTuple)  # list containing all the info

                # print self.tempItemTuple[0]
                # print self.tempItemTuple[1]
                # print self.tempItemTuple[2]
                # print self.tempItemTuple[4]
                # print self.tempItemTuple[5]


                self.totalObjectListInPartArea.append(self.tempItemTuple)
                self.totalObjectListInPartArea = sorted(self.totalObjectListInPartArea, key=itemgetter(6), reverse=False)
                if item.object_class == 'bottle':
                    self.curDetectedAssemblePartsbyRCNN['bottle'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['bottle'] = sorted(self.curDetectedAssemblePartsbyRCNN['bottle'],
                                                                           key=itemgetter(6), reverse=False)
                elif item.object_class == 'glue':
                    self.curDetectedAssemblePartsbyRCNN['glue'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['glue'] = sorted(self.curDetectedAssemblePartsbyRCNN['glue'],
                                                                         key=itemgetter(6), reverse=False)
                elif item.object_class == 'pen':
                    self.curDetectedAssemblePartsbyRCNN['pen'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['pen'] = sorted(self.curDetectedAssemblePartsbyRCNN['pen'],
                                                                         key=itemgetter(6), reverse=False)

                elif item.object_class == 'cell phone':
                    self.curDetectedAssemblePartsbyRCNN['cell phone'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['cell phone'] = sorted(self.curDetectedAssemblePartsbyRCNN['cell phone'],
                                                                          key=itemgetter(6), reverse=False)
                elif item.object_class == 'piston shaft':
                    self.curDetectedAssemblePartsbyRCNN['piston shaft'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['piston shaft'] = sorted(
                        self.curDetectedAssemblePartsbyRCNN['piston shaft'], key=itemgetter(6), reverse=False)
                elif item.object_class == 'gear':
                    self.curDetectedAssemblePartsbyRCNN['gear'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['gear'] = sorted(self.curDetectedAssemblePartsbyRCNN['gear'],
                                                                     key=itemgetter(6), reverse=False)
                else:
                    self.curDetectedAssemblePartsbyRCNN['else'].append(self.tempItemTuple)
                    self.curDetectedAssemblePartsbyRCNN['else'] = sorted(self.curDetectedAssemblePartsbyRCNN['else'],
                                                                         key=itemgetter(6), reverse=False)

                # if item.object_class == 'bottle':
                #     self.curDetectedAssemblePartsbyRCNN['bottle'].append(self.tempItemTuple)
                #     self.curDetectedAssemblePartsbyRCNN['bottle'] = sorted(
                #         self.curDetectedAssemblePartsbyRCNN['bottle'],
                #         key=itemgetter(0), reverse=True)
                # elif item.object_class == 'glue':
                #     self.curDetectedAssemblePartsbyRCNN['glue'].append(self.tempItemTuple)
                #     self.curDetectedAssemblePartsbyRCNN['glue'] = sorted(
                #         self.curDetectedAssemblePartsbyRCNN['glue'],
                #         key=itemgetter(0), reverse=True)
                # elif item.object_class == 'pen':
                #     self.curDetectedAssemblePartsbyRCNN['pen'].append(self.tempItemTuple)
                #     self.curDetectedAssemblePartsbyRCNN['pen'] = sorted(self.curDetectedAssemblePartsbyRCNN['pen'],
                #                                                         key=itemgetter(0), reverse=True)
                #
                # elif item.object_class == 'cellphone':
                #     self.curDetectedAssemblePartsbyRCNN['cellphone'].append(self.tempItemTuple)
                #     self.curDetectedAssemblePartsbyRCNN['cellphone'] = sorted(
                #         self.curDetectedAssemblePartsbyRCNN['cellphone'],
                #         key=itemgetter(0), reverse=True)
                # elif item.object_class == 'pistonshaft':
                #     self.curDetectedAssemblePartsbyRCNN['pistonshaft'].append(self.tempItemTuple)
                #     self.curDetectedAssemblePartsbyRCNN['pistonshaft'] = sorted(
                #         self.curDetectedAssemblePartsbyRCNN['pistonshaft'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'gear':
                #     self.curDetectedAssemblePartsbyRCNN['gear'].append(self.tempItemTuple)
                #     self.curDetectedAssemblePartsbyRCNN['gear'] = sorted(
                #         self.curDetectedAssemblePartsbyRCNN['gear'],
                #         key=itemgetter(0), reverse=True)

                # elif item.object_class == 'square':
                #     self.curDetectedTwoStepPartsbyRCNN['square'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepPartsbyRCNN['square'] = sorted(self.curDetectedTwoStepPartsbyRCNN['square'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'circle':
                #     self.curDetectedTwoStepPartsbyRCNN['circle'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepPartsbyRCNN['circle'] = sorted(self.curDetectedTwoStepPartsbyRCNN['circle'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'hexagon':
                #     self.curDetectedTwoStepPartsbyRCNN['hexagon'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepPartsbyRCNN['hexagon'] = sorted(self.curDetectedTwoStepPartsbyRCNN['hexagon'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'clover':
                #     self.curDetectedTwoStepPartsbyRCNN['clover'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepPartsbyRCNN['clover'] = sorted(self.curDetectedTwoStepPartsbyRCNN['clover'],key=itemgetter(0), reverse=True)
                # elif item.object_class == 'square_frame':
                #     self.curDetectedTwoStepFramebyRCNN['square_frame'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepFramebyRCNN['square_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['square_frame'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'circle_frame':
                #     self.curDetectedTwoStepFramebyRCNN['circle_frame'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepFramebyRCNN['circle_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['circle_frame'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'hexagon_frame':
                #     self.curDetectedTwoStepFramebyRCNN['hexagon_frame'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepFramebyRCNN['hexagon_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['hexagon_frame'], key=itemgetter(0), reverse=True)
                # elif item.object_class == 'clover_frame':
                #     self.curDetectedTwoStepFramebyRCNN['clover_frame'].append(self.tempItemTuple)
                #     self.curDetectedTwoStepFramebyRCNN['clover_frame'] = sorted(self.curDetectedTwoStepFramebyRCNN['clover_frame'],key=itemgetter(0), reverse=True)
                i += 1
                # self.image_grasp1 = cv2.resize(self.image_grasp1, None, fx=2.0, fy=2.0)
                # self.image_grasp1 = cv2.resize(self.image_grasp1, (200, 200))
            # print self.targetObjAreaDictList['item']

                # print (self.curDetectedTwoStepPartsbyRCNN['circle_frame'])
                # print (len(self.curDetectedTwoStepPartsbyRCNN['circle_frame']))
                # now it should be sent to AlexNet!, selectively
        # print self.targetObjAreaDictList['pcl_xyz']


    def rcnnDetection2(self, data):  # data is necessary for to exploit callback data

        # process the bounding box to pass over to AlexNet
        self.detectionfull2 = data
        self.detectionmsg2 = self.detectionfull2.detections.data  # there could be multiple of detected objects, 'data' is list type
        self.detectionimg2 = self.detectionfull2.image
        self.cv_dctimage2 = self.bridge.imgmsg_to_cv2(self.detectionimg2, "bgr8")  # 1280 * 1024
        self.cv_dspimage2 = cv2.cvtColor(self.cv_dctimage2, cv2.COLOR_BGR2RGB)  # 1280 * 1024



    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1529, 753)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.RobotStatus_Group = QtGui.QGroupBox(self.centralwidget)
        self.RobotStatus_Group.setGeometry(QtCore.QRect(10, 20, 741, 371))
        self.RobotStatus_Group.setObjectName(_fromUtf8("RobotStatus_Group"))
        self.groupBox = QtGui.QGroupBox(self.RobotStatus_Group)
        self.groupBox.setGeometry(QtCore.QRect(470, 10, 151, 171))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.J2_Read_Label_3 = QtGui.QLabel(self.groupBox)
        self.J2_Read_Label_3.setGeometry(QtCore.QRect(0, 80, 51, 17))
        self.J2_Read_Label_3.setObjectName(_fromUtf8("J2_Read_Label_3"))
        self.SawyerGrip_Close = QtGui.QPushButton(self.groupBox)
        self.SawyerGrip_Close.setGeometry(QtCore.QRect(60, 140, 61, 27))
        self.SawyerGrip_Close.setObjectName(_fromUtf8("SawyerGrip_Close"))
        self.SawyerGripPos = QtGui.QTextEdit(self.groupBox)
        self.SawyerGripPos.setGeometry(QtCore.QRect(50, 30, 71, 31))
        self.SawyerGripPos.setObjectName(_fromUtf8("SawyerGripPos"))
        self.J1_Read_Label_3 = QtGui.QLabel(self.groupBox)
        self.J1_Read_Label_3.setGeometry(QtCore.QRect(10, 40, 41, 20))
        self.J1_Read_Label_3.setObjectName(_fromUtf8("J1_Read_Label_3"))
        self.SawyerGrip_Open = QtGui.QPushButton(self.groupBox)
        self.SawyerGrip_Open.setGeometry(QtCore.QRect(60, 110, 61, 27))
        self.SawyerGrip_Open.setObjectName(_fromUtf8("SawyerGrip_Open"))
        self.SawyerGripForce = QtGui.QTextEdit(self.groupBox)
        self.SawyerGripForce.setGeometry(QtCore.QRect(50, 70, 71, 31))
        self.SawyerGripForce.setObjectName(_fromUtf8("SawyerGripForce"))
        self.groupBox_2 = QtGui.QGroupBox(self.RobotStatus_Group)
        self.groupBox_2.setGeometry(QtCore.QRect(260, 40, 181, 271))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.X_Label_read = QtGui.QLabel(self.groupBox_2)
        self.X_Label_read.setGeometry(QtCore.QRect(10, 40, 21, 17))
        self.X_Label_read.setObjectName(_fromUtf8("X_Label_read"))
        self.X_Unit_read = QtGui.QLabel(self.groupBox_2)
        self.X_Unit_read.setGeometry(QtCore.QRect(140, 40, 31, 16))
        self.X_Unit_read.setObjectName(_fromUtf8("X_Unit_read"))
        self.X_read = QtGui.QTextEdit(self.groupBox_2)
        self.X_read.setGeometry(QtCore.QRect(30, 30, 104, 31))
        self.X_read.setReadOnly(True)
        self.X_read.setObjectName(_fromUtf8("X_read"))
        self.R_Unit_Read_2 = QtGui.QLabel(self.groupBox_2)
        self.R_Unit_Read_2.setGeometry(QtCore.QRect(140, 200, 31, 16))
        self.R_Unit_Read_2.setObjectName(_fromUtf8("R_Unit_Read_2"))
        self.Y_Label_read = QtGui.QLabel(self.groupBox_2)
        self.Y_Label_read.setGeometry(QtCore.QRect(10, 80, 21, 17))
        self.Y_Label_read.setObjectName(_fromUtf8("Y_Label_read"))
        self.Ry_Label_read = QtGui.QLabel(self.groupBox_2)
        self.Ry_Label_read.setGeometry(QtCore.QRect(0, 200, 21, 17))
        self.Ry_Label_read.setObjectName(_fromUtf8("Ry_Label_read"))
        self.R_Unit_Read = QtGui.QLabel(self.groupBox_2)
        self.R_Unit_Read.setGeometry(QtCore.QRect(140, 160, 31, 16))
        self.R_Unit_Read.setObjectName(_fromUtf8("R_Unit_Read"))
        self.Z_Label_read = QtGui.QLabel(self.groupBox_2)
        self.Z_Label_read.setGeometry(QtCore.QRect(10, 120, 21, 17))
        self.Z_Label_read.setObjectName(_fromUtf8("Z_Label_read"))
        self.Rz_Label_read = QtGui.QLabel(self.groupBox_2)
        self.Rz_Label_read.setGeometry(QtCore.QRect(0, 240, 21, 17))
        self.Rz_Label_read.setObjectName(_fromUtf8("Rz_Label_read"))
        self.Y_read = QtGui.QTextEdit(self.groupBox_2)
        self.Y_read.setGeometry(QtCore.QRect(30, 70, 104, 31))
        self.Y_read.setReadOnly(True)
        self.Y_read.setObjectName(_fromUtf8("Y_read"))
        self.X_Unit_read_2 = QtGui.QLabel(self.groupBox_2)
        self.X_Unit_read_2.setGeometry(QtCore.QRect(140, 80, 31, 16))
        self.X_Unit_read_2.setObjectName(_fromUtf8("X_Unit_read_2"))
        self.Rx_Label_read = QtGui.QLabel(self.groupBox_2)
        self.Rx_Label_read.setGeometry(QtCore.QRect(0, 160, 21, 17))
        self.Rx_Label_read.setObjectName(_fromUtf8("Rx_Label_read"))
        self.Z_read = QtGui.QTextEdit(self.groupBox_2)
        self.Z_read.setGeometry(QtCore.QRect(30, 110, 104, 31))
        self.Z_read.setReadOnly(True)
        self.Z_read.setObjectName(_fromUtf8("Z_read"))
        self.Ry_read = QtGui.QTextEdit(self.groupBox_2)
        self.Ry_read.setGeometry(QtCore.QRect(30, 190, 104, 31))
        self.Ry_read.setReadOnly(True)
        self.Ry_read.setObjectName(_fromUtf8("Ry_read"))
        self.R_Unit_Read_3 = QtGui.QLabel(self.groupBox_2)
        self.R_Unit_Read_3.setGeometry(QtCore.QRect(140, 240, 31, 16))
        self.R_Unit_Read_3.setObjectName(_fromUtf8("R_Unit_Read_3"))
        self.Rz_read = QtGui.QTextEdit(self.groupBox_2)
        self.Rz_read.setGeometry(QtCore.QRect(30, 230, 104, 31))
        self.Rz_read.setReadOnly(True)
        self.Rz_read.setObjectName(_fromUtf8("Rz_read"))
        self.X_Unit_read_3 = QtGui.QLabel(self.groupBox_2)
        self.X_Unit_read_3.setGeometry(QtCore.QRect(140, 120, 31, 16))
        self.X_Unit_read_3.setObjectName(_fromUtf8("X_Unit_read_3"))
        self.Rx_read = QtGui.QTextEdit(self.groupBox_2)
        self.Rx_read.setGeometry(QtCore.QRect(30, 160, 104, 31))
        self.Rx_read.setReadOnly(True)
        self.Rx_read.setObjectName(_fromUtf8("Rx_read"))
        self.pushButton_8 = QtGui.QPushButton(self.RobotStatus_Group)
        self.pushButton_8.setGeometry(QtCore.QRect(580, 560, 81, 27))
        self.pushButton_8.setObjectName(_fromUtf8("pushButton_8"))
        self.groupBox_3 = QtGui.QGroupBox(self.RobotStatus_Group)
        self.groupBox_3.setGeometry(QtCore.QRect(30, 40, 191, 271))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.Fy_Label_read = QtGui.QLabel(self.groupBox_3)
        self.Fy_Label_read.setGeometry(QtCore.QRect(10, 70, 21, 17))
        self.Fy_Label_read.setObjectName(_fromUtf8("Fy_Label_read"))
        self.F_Unit_Read_3 = QtGui.QLabel(self.groupBox_3)
        self.F_Unit_Read_3.setGeometry(QtCore.QRect(150, 110, 21, 16))
        self.F_Unit_Read_3.setObjectName(_fromUtf8("F_Unit_Read_3"))
        self.M_Unit_Read = QtGui.QLabel(self.groupBox_3)
        self.M_Unit_Read.setGeometry(QtCore.QRect(150, 150, 31, 16))
        self.M_Unit_Read.setObjectName(_fromUtf8("M_Unit_Read"))
        self.Mz_read = QtGui.QTextEdit(self.groupBox_3)
        self.Mz_read.setGeometry(QtCore.QRect(40, 220, 104, 31))
        self.Mz_read.setReadOnly(True)
        self.Mz_read.setObjectName(_fromUtf8("Mz_read"))
        self.My_read = QtGui.QTextEdit(self.groupBox_3)
        self.My_read.setGeometry(QtCore.QRect(40, 180, 104, 31))
        self.My_read.setReadOnly(True)
        self.My_read.setObjectName(_fromUtf8("My_read"))
        self.My_Label_read = QtGui.QLabel(self.groupBox_3)
        self.My_Label_read.setGeometry(QtCore.QRect(10, 190, 21, 17))
        self.My_Label_read.setObjectName(_fromUtf8("My_Label_read"))
        self.Fx_read = QtGui.QTextEdit(self.groupBox_3)
        self.Fx_read.setGeometry(QtCore.QRect(40, 20, 104, 31))
        self.Fx_read.setReadOnly(True)
        self.Fx_read.setObjectName(_fromUtf8("Fx_read"))
        self.Fx_Label_read = QtGui.QLabel(self.groupBox_3)
        self.Fx_Label_read.setGeometry(QtCore.QRect(10, 30, 21, 16))
        self.Fx_Label_read.setObjectName(_fromUtf8("Fx_Label_read"))
        self.Fz_Label_read = QtGui.QLabel(self.groupBox_3)
        self.Fz_Label_read.setGeometry(QtCore.QRect(10, 110, 21, 17))
        self.Fz_Label_read.setObjectName(_fromUtf8("Fz_Label_read"))
        self.Mx_Label_read = QtGui.QLabel(self.groupBox_3)
        self.Mx_Label_read.setGeometry(QtCore.QRect(10, 150, 21, 17))
        self.Mx_Label_read.setObjectName(_fromUtf8("Mx_Label_read"))
        self.M_Unit_Read_3 = QtGui.QLabel(self.groupBox_3)
        self.M_Unit_Read_3.setGeometry(QtCore.QRect(150, 230, 31, 16))
        self.M_Unit_Read_3.setObjectName(_fromUtf8("M_Unit_Read_3"))
        self.Mz_Label_read = QtGui.QLabel(self.groupBox_3)
        self.Mz_Label_read.setGeometry(QtCore.QRect(10, 230, 21, 16))
        self.Mz_Label_read.setObjectName(_fromUtf8("Mz_Label_read"))
        self.Fy_read = QtGui.QTextEdit(self.groupBox_3)
        self.Fy_read.setGeometry(QtCore.QRect(40, 60, 104, 31))
        self.Fy_read.setReadOnly(True)
        self.Fy_read.setObjectName(_fromUtf8("Fy_read"))
        self.M_Unit_Read_2 = QtGui.QLabel(self.groupBox_3)
        self.M_Unit_Read_2.setGeometry(QtCore.QRect(150, 190, 31, 16))
        self.M_Unit_Read_2.setObjectName(_fromUtf8("M_Unit_Read_2"))
        self.Fz_read = QtGui.QTextEdit(self.groupBox_3)
        self.Fz_read.setGeometry(QtCore.QRect(40, 100, 104, 31))
        self.Fz_read.setReadOnly(True)
        self.Fz_read.setObjectName(_fromUtf8("Fz_read"))
        self.F_Unit_Read = QtGui.QLabel(self.groupBox_3)
        self.F_Unit_Read.setGeometry(QtCore.QRect(150, 30, 21, 16))
        self.F_Unit_Read.setObjectName(_fromUtf8("F_Unit_Read"))
        self.Mx_read = QtGui.QTextEdit(self.groupBox_3)
        self.Mx_read.setGeometry(QtCore.QRect(40, 140, 104, 31))
        self.Mx_read.setReadOnly(True)
        self.Mx_read.setObjectName(_fromUtf8("Mx_read"))
        self.F_Unit_Read_2 = QtGui.QLabel(self.groupBox_3)
        self.F_Unit_Read_2.setGeometry(QtCore.QRect(150, 70, 21, 16))
        self.F_Unit_Read_2.setObjectName(_fromUtf8("F_Unit_Read_2"))
        self.SawyerMoveL = QtGui.QGroupBox(self.RobotStatus_Group)
        self.SawyerMoveL.setGeometry(QtCore.QRect(20, 310, 701, 51))
        self.SawyerMoveL.setObjectName(_fromUtf8("SawyerMoveL"))
        self.MoveL_RX = QtGui.QTextEdit(self.SawyerMoveL)
        self.MoveL_RX.setGeometry(QtCore.QRect(230, 20, 51, 31))
        self.MoveL_RX.setObjectName(_fromUtf8("MoveL_RX"))
        self.MoveL_TZ = QtGui.QTextEdit(self.SawyerMoveL)
        self.MoveL_TZ.setGeometry(QtCore.QRect(140, 20, 51, 31))
        self.MoveL_TZ.setObjectName(_fromUtf8("MoveL_TZ"))
        self.MoveL_RZ = QtGui.QTextEdit(self.SawyerMoveL)
        self.MoveL_RZ.setGeometry(QtCore.QRect(370, 20, 51, 31))
        self.MoveL_RZ.setObjectName(_fromUtf8("MoveL_RZ"))
        self.MoveL_TY = QtGui.QTextEdit(self.SawyerMoveL)
        self.MoveL_TY.setGeometry(QtCore.QRect(70, 20, 51, 31))
        self.MoveL_TY.setObjectName(_fromUtf8("MoveL_TY"))
        self.MoveL_TX = QtGui.QTextEdit(self.SawyerMoveL)
        self.MoveL_TX.setGeometry(QtCore.QRect(0, 20, 51, 31))
        self.MoveL_TX.setObjectName(_fromUtf8("MoveL_TX"))
        self.MoveL_RY = QtGui.QTextEdit(self.SawyerMoveL)
        self.MoveL_RY.setGeometry(QtCore.QRect(300, 20, 51, 31))
        self.MoveL_RY.setObjectName(_fromUtf8("MoveL_RY"))
        self.Sawyer_MoveL_acc = QtGui.QTextEdit(self.SawyerMoveL)
        self.Sawyer_MoveL_acc.setGeometry(QtCore.QRect(450, 20, 51, 31))
        self.Sawyer_MoveL_acc.setObjectName(_fromUtf8("Sawyer_MoveL_acc"))
        self.Sawyer_MoveL_vel = QtGui.QTextEdit(self.SawyerMoveL)
        self.Sawyer_MoveL_vel.setGeometry(QtCore.QRect(540, 20, 51, 31))
        self.Sawyer_MoveL_vel.setObjectName(_fromUtf8("Sawyer_MoveL_vel"))
        self.command_MoveL = QtGui.QPushButton(self.SawyerMoveL)
        self.command_MoveL.setGeometry(QtCore.QRect(600, 20, 81, 27))
        self.command_MoveL.setObjectName(_fromUtf8("command_MoveL"))
        self.groupBox_8 = QtGui.QGroupBox(self.SawyerMoveL)
        self.groupBox_8.setGeometry(QtCore.QRect(50, 70, 701, 51))
        self.groupBox_8.setObjectName(_fromUtf8("groupBox_8"))
        self.textEdit_28 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_28.setGeometry(QtCore.QRect(370, 20, 51, 31))
        self.textEdit_28.setObjectName(_fromUtf8("textEdit_28"))
        self.textEdit_29 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_29.setGeometry(QtCore.QRect(190, 20, 51, 31))
        self.textEdit_29.setObjectName(_fromUtf8("textEdit_29"))
        self.textEdit_30 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_30.setGeometry(QtCore.QRect(130, 20, 51, 31))
        self.textEdit_30.setObjectName(_fromUtf8("textEdit_30"))
        self.textEdit_31 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_31.setGeometry(QtCore.QRect(310, 20, 51, 31))
        self.textEdit_31.setObjectName(_fromUtf8("textEdit_31"))
        self.textEdit_32 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_32.setGeometry(QtCore.QRect(70, 20, 51, 31))
        self.textEdit_32.setObjectName(_fromUtf8("textEdit_32"))
        self.textEdit_33 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_33.setGeometry(QtCore.QRect(10, 20, 51, 31))
        self.textEdit_33.setObjectName(_fromUtf8("textEdit_33"))
        self.textEdit_34 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_34.setGeometry(QtCore.QRect(250, 20, 51, 31))
        self.textEdit_34.setObjectName(_fromUtf8("textEdit_34"))
        self.textEdit_35 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_35.setGeometry(QtCore.QRect(450, 20, 51, 31))
        self.textEdit_35.setObjectName(_fromUtf8("textEdit_35"))
        self.textEdit_36 = QtGui.QTextEdit(self.groupBox_8)
        self.textEdit_36.setGeometry(QtCore.QRect(540, 20, 51, 31))
        self.textEdit_36.setObjectName(_fromUtf8("textEdit_36"))
        self.pushButton_11 = QtGui.QPushButton(self.groupBox_8)
        self.pushButton_11.setGeometry(QtCore.QRect(600, 20, 81, 27))
        self.pushButton_11.setObjectName(_fromUtf8("pushButton_11"))
        self.line_3 = QtGui.QFrame(self.SawyerMoveL)
        self.line_3.setGeometry(QtCore.QRect(430, 20, 20, 31))
        self.line_3.setFrameShape(QtGui.QFrame.VLine)
        self.line_3.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_3.setObjectName(_fromUtf8("line_3"))
        self.line_8 = QtGui.QFrame(self.SawyerMoveL)
        self.line_8.setGeometry(QtCore.QRect(510, 20, 20, 31))
        self.line_8.setFrameShape(QtGui.QFrame.VLine)
        self.line_8.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_8.setObjectName(_fromUtf8("line_8"))
        self.groupBox_4 = QtGui.QGroupBox(self.RobotStatus_Group)
        self.groupBox_4.setGeometry(QtCore.QRect(470, 190, 141, 61))
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))
        self.Cur_targetobject = QtGui.QTextEdit(self.groupBox_4)
        self.Cur_targetobject.setGeometry(QtCore.QRect(10, 20, 111, 31))
        self.Cur_targetobject.setObjectName(_fromUtf8("Cur_targetobject"))
        self.groupBox_19 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_19.setGeometry(QtCore.QRect(10, 410, 371, 221))
        self.groupBox_19.setObjectName(_fromUtf8("groupBox_19"))
        self.cam1Frame = QtGui.QFrame(self.groupBox_19)
        self.cam1Frame.setGeometry(QtCore.QRect(0, 20, 331, 201))
        self.cam1Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.cam1Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.cam1Frame.setObjectName(_fromUtf8("cam1Frame"))
        self.groupBox_20 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_20.setGeometry(QtCore.QRect(380, 410, 361, 221))
        self.groupBox_20.setObjectName(_fromUtf8("groupBox_20"))
        self.cam2Frame = QtGui.QFrame(self.groupBox_20)
        self.cam2Frame.setGeometry(QtCore.QRect(10, 20, 331, 201))
        self.cam2Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.cam2Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.cam2Frame.setObjectName(_fromUtf8("cam2Frame"))
        self.groupBox_22 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_22.setGeometry(QtCore.QRect(750, 400, 371, 221))
        self.groupBox_22.setObjectName(_fromUtf8("groupBox_22"))
        self.HelloSawyer = QtGui.QPushButton(self.groupBox_22)
        self.HelloSawyer.setGeometry(QtCore.QRect(0, 20, 121, 41))
        self.HelloSawyer.setObjectName(_fromUtf8("HelloSawyer"))
        self.HelloUR5 = QtGui.QPushButton(self.groupBox_22)
        self.HelloUR5.setGeometry(QtCore.QRect(130, 20, 121, 41))
        self.HelloUR5.setObjectName(_fromUtf8("HelloUR5"))
        self.UR5Conveys = QtGui.QPushButton(self.groupBox_22)
        self.UR5Conveys.setGeometry(QtCore.QRect(130, 70, 121, 51))
        self.UR5Conveys.setObjectName(_fromUtf8("UR5Conveys"))
        self.SawyerAssembles = QtGui.QPushButton(self.groupBox_22)
        self.SawyerAssembles.setGeometry(QtCore.QRect(0, 70, 121, 51))
        self.SawyerAssembles.setObjectName(_fromUtf8("SawyerAssembles"))
        self.SawyerAssembles2 = QtGui.QPushButton(self.groupBox_22)
        self.SawyerAssembles2.setGeometry(QtCore.QRect(0, 130, 121, 51))
        self.SawyerAssembles2.setObjectName(_fromUtf8("SawyerAssembles2"))
        self.StartDEMO = QtGui.QPushButton(self.groupBox_22)
        self.StartDEMO.setGeometry(QtCore.QRect(260, 20, 101, 91))
        self.StartDEMO.setObjectName(_fromUtf8("StartDEMO"))
        self.line = QtGui.QFrame(self.groupBox_22)
        self.line.setGeometry(QtCore.QRect(120, 20, 20, 161))
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.line_9 = QtGui.QFrame(self.groupBox_22)
        self.line_9.setGeometry(QtCore.QRect(250, 20, 20, 161))
        self.line_9.setFrameShape(QtGui.QFrame.VLine)
        self.line_9.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_9.setObjectName(_fromUtf8("line_9"))
        self.E_Stop = QtGui.QPushButton(self.groupBox_22)
        self.E_Stop.setGeometry(QtCore.QRect(260, 120, 101, 51))
        self.E_Stop.setObjectName(_fromUtf8("E_Stop"))
        self.line_4 = QtGui.QFrame(self.groupBox_22)
        self.line_4.setGeometry(QtCore.QRect(140, 120, 118, 3))
        self.line_4.setFrameShape(QtGui.QFrame.HLine)
        self.line_4.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_4.setObjectName(_fromUtf8("line_4"))
        self.humanDEMO_btn = QtGui.QPushButton(self.groupBox_22)
        self.humanDEMO_btn.setGeometry(QtCore.QRect(140, 130, 111, 41))
        self.humanDEMO_btn.setObjectName(_fromUtf8("humanDEMO_btn"))
        self.selfDEMO_btin = QtGui.QPushButton(self.groupBox_22)
        self.selfDEMO_btin.setGeometry(QtCore.QRect(140, 170, 111, 41))
        self.selfDEMO_btin.setObjectName(_fromUtf8("selfDEMO_btin"))
        self.selfDEMO_btin.setCheckable(True)
        self.groupBox_23 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_23.setGeometry(QtCore.QRect(0, 630, 1261, 80))
        self.groupBox_23.setObjectName(_fromUtf8("groupBox_23"))
        self.label = QtGui.QLabel(self.groupBox_23)
        self.label.setGeometry(QtCore.QRect(830, 10, 41, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.splitter = QtGui.QSplitter(self.groupBox_23)
        self.splitter.setGeometry(QtCore.QRect(10, 30, 801, 44))
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName(_fromUtf8("splitter"))
        self.InitAllNodes = QtGui.QPushButton(self.splitter)
        self.InitAllNodes.setObjectName(_fromUtf8("InitAllNodes"))
        self.InitAllNodes.setCheckable(True)
        self.EnDisSawyer = QtGui.QPushButton(self.splitter)
        self.EnDisSawyer.setObjectName(_fromUtf8("EnDisSawyer"))
        self.EnDisUR5 = QtGui.QPushButton(self.splitter)
        self.EnDisUR5.setObjectName(_fromUtf8("EnDisUR5"))
        self.EnDisRobotiq = QtGui.QPushButton(self.splitter)
        self.EnDisRobotiq.setObjectName(_fromUtf8("EnDisRobotiq"))
        self.EnDisPtGrey1 = QtGui.QPushButton(self.splitter)
        self.EnDisPtGrey1.setObjectName(_fromUtf8("EnDisPtGrey1"))
        self.EnDisPtGrey2 = QtGui.QPushButton(self.splitter)
        self.EnDisPtGrey2.setObjectName(_fromUtf8("EnDisPtGrey2"))
        self.Logs = QtGui.QTextEdit(self.groupBox_23)
        self.Logs.setGeometry(QtCore.QRect(870, 0, 381, 78))
        self.Logs.setObjectName(_fromUtf8("Logs"))
        self.ColseWindow = QtGui.QPushButton(self.centralwidget)
        self.ColseWindow.setGeometry(QtCore.QRect(1270, 680, 121, 31))
        self.ColseWindow.setObjectName(_fromUtf8("ColseWindow"))
        self.RobotStatus_Group_2 = QtGui.QGroupBox(self.centralwidget)
        self.RobotStatus_Group_2.setGeometry(QtCore.QRect(760, 30, 741, 361))
        self.RobotStatus_Group_2.setObjectName(_fromUtf8("RobotStatus_Group_2"))
        self.groupBox_10 = QtGui.QGroupBox(self.RobotStatus_Group_2)
        self.groupBox_10.setGeometry(QtCore.QRect(590, 20, 151, 241))
        self.groupBox_10.setObjectName(_fromUtf8("groupBox_10"))
        self.J2_Read_Label_4 = QtGui.QLabel(self.groupBox_10)
        self.J2_Read_Label_4.setGeometry(QtCore.QRect(0, 80, 51, 17))
        self.J2_Read_Label_4.setObjectName(_fromUtf8("J2_Read_Label_4"))
        self.Robotiq_close = QtGui.QPushButton(self.groupBox_10)
        self.Robotiq_close.setGeometry(QtCore.QRect(60, 140, 61, 27))
        self.Robotiq_close.setObjectName(_fromUtf8("Robotiq_close"))
        self.Robotiq_pos = QtGui.QTextEdit(self.groupBox_10)
        self.Robotiq_pos.setGeometry(QtCore.QRect(50, 30, 71, 31))
        self.Robotiq_pos.setObjectName(_fromUtf8("Robotiq_pos"))
        self.J1_Read_Label_4 = QtGui.QLabel(self.groupBox_10)
        self.J1_Read_Label_4.setGeometry(QtCore.QRect(10, 40, 41, 20))
        self.J1_Read_Label_4.setObjectName(_fromUtf8("J1_Read_Label_4"))
        self.Robotiq_open = QtGui.QPushButton(self.groupBox_10)
        self.Robotiq_open.setGeometry(QtCore.QRect(60, 110, 61, 27))
        self.Robotiq_open.setObjectName(_fromUtf8("Robotiq_open"))
        self.Robotiq_force = QtGui.QTextEdit(self.groupBox_10)
        self.Robotiq_force.setGeometry(QtCore.QRect(50, 70, 71, 31))
        self.Robotiq_force.setObjectName(_fromUtf8("Robotiq_force"))
        self.groupBox_11 = QtGui.QGroupBox(self.RobotStatus_Group_2)
        self.groupBox_11.setGeometry(QtCore.QRect(390, 10, 181, 271))
        self.groupBox_11.setObjectName(_fromUtf8("groupBox_11"))
        self.X_Label_read_2 = QtGui.QLabel(self.groupBox_11)
        self.X_Label_read_2.setGeometry(QtCore.QRect(10, 40, 21, 17))
        self.X_Label_read_2.setObjectName(_fromUtf8("X_Label_read_2"))
        self.X_Unit_read_4 = QtGui.QLabel(self.groupBox_11)
        self.X_Unit_read_4.setGeometry(QtCore.QRect(140, 40, 31, 16))
        self.X_Unit_read_4.setObjectName(_fromUtf8("X_Unit_read_4"))
        self.X_UR5 = QtGui.QTextEdit(self.groupBox_11)
        self.X_UR5.setGeometry(QtCore.QRect(30, 30, 104, 31))
        self.X_UR5.setReadOnly(True)
        self.X_UR5.setObjectName(_fromUtf8("X_UR5"))
        self.R_Unit_Read_4 = QtGui.QLabel(self.groupBox_11)
        self.R_Unit_Read_4.setGeometry(QtCore.QRect(140, 200, 31, 16))
        self.R_Unit_Read_4.setObjectName(_fromUtf8("R_Unit_Read_4"))
        self.Y_Label_read_2 = QtGui.QLabel(self.groupBox_11)
        self.Y_Label_read_2.setGeometry(QtCore.QRect(10, 80, 21, 17))
        self.Y_Label_read_2.setObjectName(_fromUtf8("Y_Label_read_2"))
        self.Ry_Label_read_2 = QtGui.QLabel(self.groupBox_11)
        self.Ry_Label_read_2.setGeometry(QtCore.QRect(0, 200, 21, 17))
        self.Ry_Label_read_2.setObjectName(_fromUtf8("Ry_Label_read_2"))
        self.R_Unit_Read_5 = QtGui.QLabel(self.groupBox_11)
        self.R_Unit_Read_5.setGeometry(QtCore.QRect(140, 160, 31, 16))
        self.R_Unit_Read_5.setObjectName(_fromUtf8("R_Unit_Read_5"))
        self.Z_Label_read_2 = QtGui.QLabel(self.groupBox_11)
        self.Z_Label_read_2.setGeometry(QtCore.QRect(10, 120, 21, 17))
        self.Z_Label_read_2.setObjectName(_fromUtf8("Z_Label_read_2"))
        self.Rz_Label_read_2 = QtGui.QLabel(self.groupBox_11)
        self.Rz_Label_read_2.setGeometry(QtCore.QRect(0, 240, 21, 17))
        self.Rz_Label_read_2.setObjectName(_fromUtf8("Rz_Label_read_2"))
        self.Y_UR5 = QtGui.QTextEdit(self.groupBox_11)
        self.Y_UR5.setGeometry(QtCore.QRect(30, 70, 104, 31))
        self.Y_UR5.setReadOnly(True)
        self.Y_UR5.setObjectName(_fromUtf8("Y_UR5"))
        self.X_Unit_read_5 = QtGui.QLabel(self.groupBox_11)
        self.X_Unit_read_5.setGeometry(QtCore.QRect(140, 80, 31, 16))
        self.X_Unit_read_5.setObjectName(_fromUtf8("X_Unit_read_5"))
        self.Rx_Label_read_2 = QtGui.QLabel(self.groupBox_11)
        self.Rx_Label_read_2.setGeometry(QtCore.QRect(0, 160, 21, 17))
        self.Rx_Label_read_2.setObjectName(_fromUtf8("Rx_Label_read_2"))
        self.Z_UR5 = QtGui.QTextEdit(self.groupBox_11)
        self.Z_UR5.setGeometry(QtCore.QRect(30, 110, 104, 31))
        self.Z_UR5.setReadOnly(True)
        self.Z_UR5.setObjectName(_fromUtf8("Z_UR5"))
        self.Ry_UR5 = QtGui.QTextEdit(self.groupBox_11)
        self.Ry_UR5.setGeometry(QtCore.QRect(30, 190, 104, 31))
        self.Ry_UR5.setReadOnly(True)
        self.Ry_UR5.setObjectName(_fromUtf8("Ry_UR5"))
        self.R_Unit_Read_6 = QtGui.QLabel(self.groupBox_11)
        self.R_Unit_Read_6.setGeometry(QtCore.QRect(140, 240, 31, 16))
        self.R_Unit_Read_6.setObjectName(_fromUtf8("R_Unit_Read_6"))
        self.Rz_UR5 = QtGui.QTextEdit(self.groupBox_11)
        self.Rz_UR5.setGeometry(QtCore.QRect(30, 230, 104, 31))
        self.Rz_UR5.setReadOnly(True)
        self.Rz_UR5.setObjectName(_fromUtf8("Rz_UR5"))
        self.X_Unit_read_6 = QtGui.QLabel(self.groupBox_11)
        self.X_Unit_read_6.setGeometry(QtCore.QRect(140, 120, 31, 16))
        self.X_Unit_read_6.setObjectName(_fromUtf8("X_Unit_read_6"))
        self.Rx_UR5 = QtGui.QTextEdit(self.groupBox_11)
        self.Rx_UR5.setGeometry(QtCore.QRect(30, 150, 104, 31))
        self.Rx_UR5.setReadOnly(True)
        self.Rx_UR5.setObjectName(_fromUtf8("Rx_UR5"))
        self.pushButton_10 = QtGui.QPushButton(self.RobotStatus_Group_2)
        self.pushButton_10.setGeometry(QtCore.QRect(580, 560, 81, 27))
        self.pushButton_10.setObjectName(_fromUtf8("pushButton_10"))
        self.groupBox_13 = QtGui.QGroupBox(self.RobotStatus_Group_2)
        self.groupBox_13.setGeometry(QtCore.QRect(0, 30, 361, 101))
        self.groupBox_13.setObjectName(_fromUtf8("groupBox_13"))
        self.J4_UR5 = QtGui.QTextEdit(self.groupBox_13)
        self.J4_UR5.setGeometry(QtCore.QRect(300, 20, 61, 31))
        self.J4_UR5.setObjectName(_fromUtf8("J4_UR5"))
        self.J5_Read_Label_5 = QtGui.QLabel(self.groupBox_13)
        self.J5_Read_Label_5.setGeometry(QtCore.QRect(10, 70, 21, 21))
        self.J5_Read_Label_5.setObjectName(_fromUtf8("J5_Read_Label_5"))
        self.J2_Read_Label_5 = QtGui.QLabel(self.groupBox_13)
        self.J2_Read_Label_5.setGeometry(QtCore.QRect(100, 30, 21, 17))
        self.J2_Read_Label_5.setObjectName(_fromUtf8("J2_Read_Label_5"))
        self.J5_UR5 = QtGui.QTextEdit(self.groupBox_13)
        self.J5_UR5.setGeometry(QtCore.QRect(30, 60, 61, 31))
        self.J5_UR5.setObjectName(_fromUtf8("J5_UR5"))
        self.J6_Read_Label_5 = QtGui.QLabel(self.groupBox_13)
        self.J6_Read_Label_5.setGeometry(QtCore.QRect(100, 70, 21, 17))
        self.J6_Read_Label_5.setObjectName(_fromUtf8("J6_Read_Label_5"))
        self.J6_UR5 = QtGui.QTextEdit(self.groupBox_13)
        self.J6_UR5.setGeometry(QtCore.QRect(120, 60, 61, 31))
        self.J6_UR5.setObjectName(_fromUtf8("J6_UR5"))
        self.J2_UR5 = QtGui.QTextEdit(self.groupBox_13)
        self.J2_UR5.setGeometry(QtCore.QRect(120, 20, 61, 31))
        self.J2_UR5.setObjectName(_fromUtf8("J2_UR5"))
        self.J3_UR5 = QtGui.QTextEdit(self.groupBox_13)
        self.J3_UR5.setGeometry(QtCore.QRect(210, 20, 61, 31))
        self.J3_UR5.setObjectName(_fromUtf8("J3_UR5"))
        self.J3_Read_Label_5 = QtGui.QLabel(self.groupBox_13)
        self.J3_Read_Label_5.setGeometry(QtCore.QRect(190, 30, 21, 17))
        self.J3_Read_Label_5.setObjectName(_fromUtf8("J3_Read_Label_5"))
        self.J1_Read_Label_5 = QtGui.QLabel(self.groupBox_13)
        self.J1_Read_Label_5.setGeometry(QtCore.QRect(10, 30, 21, 21))
        self.J1_Read_Label_5.setObjectName(_fromUtf8("J1_Read_Label_5"))
        self.J1_UR5 = QtGui.QTextEdit(self.groupBox_13)
        self.J1_UR5.setGeometry(QtCore.QRect(30, 20, 61, 31))
        self.J1_UR5.setObjectName(_fromUtf8("J1_UR5"))
        self.J4_Read_Label_5 = QtGui.QLabel(self.groupBox_13)
        self.J4_Read_Label_5.setGeometry(QtCore.QRect(280, 30, 21, 17))
        self.J4_Read_Label_5.setObjectName(_fromUtf8("J4_Read_Label_5"))
        self.groupBox_16 = QtGui.QGroupBox(self.RobotStatus_Group_2)
        self.groupBox_16.setGeometry(QtCore.QRect(10, 300, 701, 51))
        self.groupBox_16.setObjectName(_fromUtf8("groupBox_16"))
        self.MoveL_RX_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_RX_2.setGeometry(QtCore.QRect(230, 20, 51, 31))
        self.MoveL_RX_2.setObjectName(_fromUtf8("MoveL_RX_2"))
        self.MoveL_TZ_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_TZ_2.setGeometry(QtCore.QRect(140, 20, 51, 31))
        self.MoveL_TZ_2.setObjectName(_fromUtf8("MoveL_TZ_2"))
        self.MoveL_RZ_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_RZ_2.setGeometry(QtCore.QRect(370, 20, 51, 31))
        self.MoveL_RZ_2.setObjectName(_fromUtf8("MoveL_RZ_2"))
        self.MoveL_TY_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_TY_2.setGeometry(QtCore.QRect(70, 20, 51, 31))
        self.MoveL_TY_2.setObjectName(_fromUtf8("MoveL_TY_2"))
        self.MoveL_TX_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_TX_2.setGeometry(QtCore.QRect(0, 20, 51, 31))
        self.MoveL_TX_2.setObjectName(_fromUtf8("MoveL_TX_2"))
        self.MoveL_RY_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_RY_2.setGeometry(QtCore.QRect(300, 20, 51, 31))
        self.MoveL_RY_2.setObjectName(_fromUtf8("MoveL_RY_2"))
        self.MoveL_acc_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_acc_2.setGeometry(QtCore.QRect(450, 20, 51, 31))
        self.MoveL_acc_2.setObjectName(_fromUtf8("MoveL_acc_2"))
        self.MoveL_vel_2 = QtGui.QTextEdit(self.groupBox_16)
        self.MoveL_vel_2.setGeometry(QtCore.QRect(540, 20, 51, 31))
        self.MoveL_vel_2.setObjectName(_fromUtf8("MoveL_vel_2"))
        self.command_MoveL_UR = QtGui.QPushButton(self.groupBox_16)
        self.command_MoveL_UR.setGeometry(QtCore.QRect(600, 20, 81, 27))
        self.command_MoveL_UR.setObjectName(_fromUtf8("command_MoveL_UR"))
        self.groupBox_17 = QtGui.QGroupBox(self.groupBox_16)
        self.groupBox_17.setGeometry(QtCore.QRect(50, 70, 701, 51))
        self.groupBox_17.setObjectName(_fromUtf8("groupBox_17"))
        self.textEdit_37 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_37.setGeometry(QtCore.QRect(370, 20, 51, 31))
        self.textEdit_37.setObjectName(_fromUtf8("textEdit_37"))
        self.textEdit_38 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_38.setGeometry(QtCore.QRect(190, 20, 51, 31))
        self.textEdit_38.setObjectName(_fromUtf8("textEdit_38"))
        self.textEdit_39 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_39.setGeometry(QtCore.QRect(130, 20, 51, 31))
        self.textEdit_39.setObjectName(_fromUtf8("textEdit_39"))
        self.textEdit_40 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_40.setGeometry(QtCore.QRect(310, 20, 51, 31))
        self.textEdit_40.setObjectName(_fromUtf8("textEdit_40"))
        self.textEdit_41 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_41.setGeometry(QtCore.QRect(70, 20, 51, 31))
        self.textEdit_41.setObjectName(_fromUtf8("textEdit_41"))
        self.textEdit_42 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_42.setGeometry(QtCore.QRect(10, 20, 51, 31))
        self.textEdit_42.setObjectName(_fromUtf8("textEdit_42"))
        self.textEdit_43 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_43.setGeometry(QtCore.QRect(250, 20, 51, 31))
        self.textEdit_43.setObjectName(_fromUtf8("textEdit_43"))
        self.textEdit_44 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_44.setGeometry(QtCore.QRect(450, 20, 51, 31))
        self.textEdit_44.setObjectName(_fromUtf8("textEdit_44"))
        self.textEdit_45 = QtGui.QTextEdit(self.groupBox_17)
        self.textEdit_45.setGeometry(QtCore.QRect(540, 20, 51, 31))
        self.textEdit_45.setObjectName(_fromUtf8("textEdit_45"))
        self.pushButton_13 = QtGui.QPushButton(self.groupBox_17)
        self.pushButton_13.setGeometry(QtCore.QRect(600, 20, 81, 27))
        self.pushButton_13.setObjectName(_fromUtf8("pushButton_13"))
        self.line_13 = QtGui.QFrame(self.groupBox_16)
        self.line_13.setGeometry(QtCore.QRect(430, 20, 20, 31))
        self.line_13.setFrameShape(QtGui.QFrame.VLine)
        self.line_13.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_13.setObjectName(_fromUtf8("line_13"))
        self.line_14 = QtGui.QFrame(self.groupBox_16)
        self.line_14.setGeometry(QtCore.QRect(510, 20, 20, 31))
        self.line_14.setFrameShape(QtGui.QFrame.VLine)
        self.line_14.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_14.setObjectName(_fromUtf8("line_14"))
        self.groupBox_5 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_5.setGeometry(QtCore.QRect(1120, 390, 401, 131))
        self.groupBox_5.setObjectName(_fromUtf8("groupBox_5"))
        self.pickBottleBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickBottleBtn.setGeometry(QtCore.QRect(0, 30, 71, 27))
        self.pickBottleBtn.setObjectName(_fromUtf8("pickBottleBtn"))
        self.pickGlueBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickGlueBtn.setGeometry(QtCore.QRect(80, 30, 71, 27))
        self.pickGlueBtn.setObjectName(_fromUtf8("pickGlueBtn"))
        self.pickPhoneBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickPhoneBtn.setGeometry(QtCore.QRect(160, 30, 71, 27))
        self.pickPhoneBtn.setObjectName(_fromUtf8("pickPhoneBtn"))
        self.pickShaftBtn4 = QtGui.QPushButton(self.groupBox_5)
        self.pickShaftBtn4.setGeometry(QtCore.QRect(240, 30, 71, 27))
        self.pickShaftBtn4.setObjectName(_fromUtf8("pickShaftBtn4"))
        self.pickGearBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickGearBtn.setGeometry(QtCore.QRect(320, 30, 71, 27))
        self.pickGearBtn.setObjectName(_fromUtf8("pickGearBtn"))
        self.pickCircleBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickCircleBtn.setGeometry(QtCore.QRect(0, 90, 71, 27))
        self.pickCircleBtn.setObjectName(_fromUtf8("pickCircleBtn"))
        self.label_2 = QtGui.QLabel(self.groupBox_5)
        self.label_2.setGeometry(QtCore.QRect(0, 70, 111, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.pickSquareBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickSquareBtn.setGeometry(QtCore.QRect(80, 90, 71, 27))
        self.pickSquareBtn.setObjectName(_fromUtf8("pickSquareBtn"))
        self.pickHexagonBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickHexagonBtn.setGeometry(QtCore.QRect(160, 90, 71, 27))
        self.pickHexagonBtn.setObjectName(_fromUtf8("pickHexagonBtn"))
        self.pickCloverBtn = QtGui.QPushButton(self.groupBox_5)
        self.pickCloverBtn.setGeometry(QtCore.QRect(240, 90, 71, 27))
        self.pickCloverBtn.setObjectName(_fromUtf8("pickCloverBtn"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1529, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.InitAllNodes.clicked.connect(self.robot_Button_Clicked)
        ######################################################################
        # self.randomBtoA.clicked.connect(self.pickplace_BtoA)
        # self.randomAtoB.clicked.connect(self.pickplace_AtoB)
        ######################################################################
        self.pickBottleBtn.clicked.connect(self.pickBottle)
        self.pickGearBtn.clicked.connect(self.pickGear)
        self.pickGlueBtn.clicked.connect(self.pickGlue)
        self.pickPhoneBtn.clicked.connect(self.pickPhone)
        self.pickShaftBtn4.clicked.connect(self.pickShaft)
        self.E_Stop.clicked.connect(self.pickPen)
        ######################################################################
        self.pickCircleBtn.clicked.connect(self.pickCircle_frame)
        self.pickHexagonBtn.clicked.connect(self.pickHexagon_frame)
        self.pickSquareBtn.clicked.connect(self.pickSquare_frame)
        self.pickCloverBtn.clicked.connect(self.pickClover_frame)
        ######################################################################
        self.humanDEMO_btn.clicked.connect(self.save_demo_values_to_csv)
        self.selfDEMO_btin.clicked.connect(self.overwrite_measured_Values)
        self.StartDEMO.clicked.connect(self.overwrite_measured_Values)




    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "DeepRL_Sawyer", None))
        self.RobotStatus_Group.setTitle(_translate("MainWindow", "Robot Status (read from Sawyer)", None))
        self.groupBox.setTitle(_translate("MainWindow", "Schunk Gripper", None))
        self.J2_Read_Label_3.setText(_translate("MainWindow", "Force", None))
        self.SawyerGrip_Close.setText(_translate("MainWindow", "Close", None))
        self.J1_Read_Label_3.setText(_translate("MainWindow", "Pos.", None))
        self.SawyerGrip_Open.setText(_translate("MainWindow", "Open", None))
        self.groupBox_2.setTitle(_translate("MainWindow", "Position in C.S.", None))
        self.X_Label_read.setText(_translate("MainWindow", "X", None))
        self.X_Unit_read.setText(_translate("MainWindow", "mm", None))
        self.R_Unit_Read_2.setText(_translate("MainWindow", "deg", None))
        self.Y_Label_read.setText(_translate("MainWindow", "Y", None))
        self.Ry_Label_read.setText(_translate("MainWindow", "Ry", None))
        self.R_Unit_Read.setText(_translate("MainWindow", "deg", None))
        self.Z_Label_read.setText(_translate("MainWindow", "Z", None))
        self.Rz_Label_read.setText(_translate("MainWindow", "Rz", None))
        self.X_Unit_read_2.setText(_translate("MainWindow", "mm", None))
        self.Rx_Label_read.setText(_translate("MainWindow", "Rx", None))
        self.R_Unit_Read_3.setText(_translate("MainWindow", "deg", None))
        self.X_Unit_read_3.setText(_translate("MainWindow", "mm", None))
        self.pushButton_8.setText(_translate("MainWindow", "Command", None))
        self.groupBox_3.setTitle(_translate("MainWindow", "Force & Moment in C.S.", None))
        self.Fy_Label_read.setText(_translate("MainWindow", "Fy", None))
        self.F_Unit_Read_3.setText(_translate("MainWindow", "N", None))
        self.M_Unit_Read.setText(_translate("MainWindow", "Nm", None))
        self.My_Label_read.setText(_translate("MainWindow", "My", None))
        self.Fx_read.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.Fx_Label_read.setText(_translate("MainWindow", "Fx", None))
        self.Fz_Label_read.setText(_translate("MainWindow", "Fz", None))
        self.Mx_Label_read.setText(_translate("MainWindow", "Mx", None))
        self.M_Unit_Read_3.setText(_translate("MainWindow", "Nm", None))
        self.Mz_Label_read.setText(_translate("MainWindow", "Mz", None))
        self.M_Unit_Read_2.setText(_translate("MainWindow", "Nm", None))
        self.F_Unit_Read.setText(_translate("MainWindow", "N", None))
        self.F_Unit_Read_2.setText(_translate("MainWindow", "N", None))
        self.SawyerMoveL.setTitle(_translate("MainWindow", "Move L:                  Trans.                                     Rot.                              |       Accel   |          Vel", None))
        self.command_MoveL.setText(_translate("MainWindow", "Command", None))
        self.groupBox_8.setTitle(_translate("MainWindow", "Move J:                                               J1~ J7                                              |       Accel   |          Vel", None))
        self.pushButton_11.setText(_translate("MainWindow", "Command", None))
        self.groupBox_4.setTitle(_translate("MainWindow", "Cur target object", None))
        self.groupBox_19.setTitle(_translate("MainWindow", "Cam1 View", None))
        self.groupBox_20.setTitle(_translate("MainWindow", "Cam2 View", None))
        self.groupBox_22.setTitle(_translate("MainWindow", "DEMO", None))
        self.HelloSawyer.setText(_translate("MainWindow", "Hello, Sawyer!", None))
        self.HelloUR5.setText(_translate("MainWindow", "Hello, UR5!", None))
        self.UR5Conveys.setText(_translate("MainWindow", "UR5 conveys \n"
" assemby plates", None))
        self.SawyerAssembles.setText(_translate("MainWindow", "Sawyer \n"
" Assmebles!", None))
        self.SawyerAssembles2.setText(_translate("MainWindow", "Sawyer \n"
" Assmebles2!", None))
        self.StartDEMO.setText(_translate("MainWindow", "************\n"
" Start DEMO! \n"
"************", None))
        self.E_Stop.setText(_translate("MainWindow", "Pick\npen", None))
        self.humanDEMO_btn.setText(_translate("MainWindow", "Record 1st \n"
" human demo", None))
        self.selfDEMO_btin.setText(_translate("MainWindow", "Record \n"
" self demos", None))
        self.groupBox_23.setTitle(_translate("MainWindow", "Wake up / Sleep Robots & Devices", None))
        self.label.setText(_translate("MainWindow", "Logs", None))
        self.InitAllNodes.setText(_translate("MainWindow", "Initialize all nodes", None))
        self.EnDisSawyer.setText(_translate("MainWindow", "Enable/Disable \n"
" Sawyer", None))
        self.EnDisUR5.setText(_translate("MainWindow", "Enable/Disable \n"
" UR5", None))
        self.EnDisRobotiq.setText(_translate("MainWindow", "Enable/Disable \n"
" Robotiq", None))
        self.EnDisPtGrey1.setText(_translate("MainWindow", "Enable/Disable \n"
" PtGrey1", None))
        self.EnDisPtGrey2.setText(_translate("MainWindow", "Enable/Disable \n"
" PtGrey2", None))
        self.ColseWindow.setText(_translate("MainWindow", "Close", None))
        self.RobotStatus_Group_2.setTitle(_translate("MainWindow", "Robot Status (read from UR5)", None))
        self.groupBox_10.setTitle(_translate("MainWindow", "Robotiq Gripper", None))
        self.J2_Read_Label_4.setText(_translate("MainWindow", "Force", None))
        self.Robotiq_close.setText(_translate("MainWindow", "Close", None))
        self.J1_Read_Label_4.setText(_translate("MainWindow", "Pos.", None))
        self.Robotiq_open.setText(_translate("MainWindow", "Open", None))
        self.groupBox_11.setTitle(_translate("MainWindow", "Position in C.S.", None))
        self.X_Label_read_2.setText(_translate("MainWindow", "X", None))
        self.X_Unit_read_4.setText(_translate("MainWindow", "mm", None))
        self.R_Unit_Read_4.setText(_translate("MainWindow", "deg", None))
        self.Y_Label_read_2.setText(_translate("MainWindow", "Y", None))
        self.Ry_Label_read_2.setText(_translate("MainWindow", "Ry", None))
        self.R_Unit_Read_5.setText(_translate("MainWindow", "deg", None))
        self.Z_Label_read_2.setText(_translate("MainWindow", "Z", None))
        self.Rz_Label_read_2.setText(_translate("MainWindow", "Rz", None))
        self.X_Unit_read_5.setText(_translate("MainWindow", "mm", None))
        self.Rx_Label_read_2.setText(_translate("MainWindow", "Rx", None))
        self.R_Unit_Read_6.setText(_translate("MainWindow", "deg", None))
        self.X_Unit_read_6.setText(_translate("MainWindow", "mm", None))
        self.pushButton_10.setText(_translate("MainWindow", "Command", None))
        self.groupBox_13.setTitle(_translate("MainWindow", "Joint Positions", None))
        self.J5_Read_Label_5.setText(_translate("MainWindow", "J5", None))
        self.J2_Read_Label_5.setText(_translate("MainWindow", "J2", None))
        self.J6_Read_Label_5.setText(_translate("MainWindow", "J6", None))
        self.J3_Read_Label_5.setText(_translate("MainWindow", "J3", None))
        self.J1_Read_Label_5.setText(_translate("MainWindow", "J1", None))
        self.J4_Read_Label_5.setText(_translate("MainWindow", "J4", None))
        self.groupBox_16.setTitle(_translate("MainWindow", "Move L:                  Trans.                                     Rot.                              |       Accel   |          Vel", None))
        self.command_MoveL_UR.setText(_translate("MainWindow", "Command", None))
        self.groupBox_17.setTitle(_translate("MainWindow", "Move J:                                               J1~ J7                                              |       Accel   |          Vel", None))
        self.pushButton_13.setText(_translate("MainWindow", "Command", None))
        self.groupBox_5.setTitle(_translate("MainWindow", "Assemble specific object", None))
        self.pickBottleBtn.setText(_translate("MainWindow", "Bottle", None))
        self.pickGlueBtn.setText(_translate("MainWindow", "Glue", None))
        self.pickPhoneBtn.setText(_translate("MainWindow", "Phone", None))
        self.pickShaftBtn4.setText(_translate("MainWindow", "Shaft", None))
        self.pickGearBtn.setText(_translate("MainWindow", "Gear", None))
        self.pickCircleBtn.setText(_translate("MainWindow", "Circle", None))
        self.label_2.setText(_translate("MainWindow", "2 step assembly", None))
        self.pickSquareBtn.setText(_translate("MainWindow", "Square", None))
        self.pickHexagonBtn.setText(_translate("MainWindow", "Hexagon", None))
        self.pickCloverBtn.setText(_translate("MainWindow", "Clover", None))

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

    timer2 = QtCore.QTimer()
    timer2.timeout.connect(ui.read_status_from_UR_monitor)
    timer2.start(100)



    sys.exit(app.exec_())
