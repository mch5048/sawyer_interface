#!/usr/bin/python
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class urMotion(object):

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

    def deployDemo(self):
        print('deploying Demo motion')
        self.ur5.movel(self.init_wpr,self.lin_accel,self.lin_vel)
        self.ur5.movel(self.wp1)
        self.ur5.movel(self.wp2)
        self.ur5.movel(self.wp3)
        self.ur5.movel(self.wp4)
