#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String
import tf
import numpy as np
from numpy import linalg
from utils.utils import wrapToPi
from utils.grids import StochOccupancyGrid2D
from planners import AStar, compute_smoothed_traj
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum

from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig


nav_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


def cmd_mid_callback_nav(data):
        """
        loads in goal if different from current goal, and replans
        """
        control_type = rospy.get_param("control")
        if control_type == "Nav":
                nav_vel_pub.publish(data)
                print("Nav")
        

def cmd_mid_callback_key(data):
        """
        loads in goal if different from current goal, and replans
        """
        control_type = rospy.get_param("control")
        if control_type == "Key":
                nav_vel_pub.publish(data)
                print("Key")
        


def subscriber():
        rospy.init_node('mid', anonymous=True)
        rospy.Subscriber("/control_switch", String, cmd_callback)
        rospy.Subscriber("/cmd_mid_nav",  Twist,  cmd_mid_callback_nav)
        rospy.Subscriber("/cmd_mid_key",  Twist,  cmd_mid_callback_key)
        rospy.spin()


def cmd_callback(data):

        rospy.set_param("control", data)    

if __name__ == '__main__':
    subscriber()
