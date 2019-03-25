#!/usr/bin/python



import rospy
from object_detection_YOLOv2.msg import *
from std_msgs.msg import *
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
from intera_interface import Limb


Class trajectorySender(object):

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