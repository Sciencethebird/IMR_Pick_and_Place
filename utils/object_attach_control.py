import rospy
from panda_robot import PandaArm
import tf_conversions as tf
import franka_interface
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32
from moveit_commander.conversions import pose_to_list
import quaternion # for processing ee_pose() output
from pyquaternion import Quaternion
import argparse


class object_server():
    def __init__(self):

        self.pub = rospy.Publisher('/robot_arm/attached_model/name', String, queue_size=2)
        self.pubx = rospy.Publisher('/robot_arm/attached_model/xoffset', Float32, queue_size=2)
        self.puby = rospy.Publisher('/robot_arm/attached_model/yoffset', Float32, queue_size=2)
        self.pubz = rospy.Publisher('/robot_arm/attached_model/zoffset', Float32, queue_size=2)
        self.pubyr = rospy.Publisher('/robot_arm/attached_model/y_rotation', Float32, queue_size=2)
        # you can implement x roll, z roll by your self
        # clear file name index
        try: 
            rospy.init_node("gazebo_object_attach_server", anonymous=True)
        except:
            print "gazebo_object_controller is already initialized!"

    def attach_object(self, object_name, x_offset = 0.0, y_offset = 0.0, z_offset = -0.18, yr = 0.0):
    	rate = rospy.Rate(100)
        #rospy.loginfo(self.object_msg)
        for i in range(10):
            self.pub.publish(object_name)
            self.pubx.publish(x_offset)
            self.puby.publish(y_offset)
            self.pubz.publish(z_offset)
            self.pubyr.publish(yr)

    def detach_object(self):
    	rate = rospy.Rate(100)
        #rospy.loginfo(self.object_msg)
        for i in range(10):
            self.pub.publish("none")


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='attach a gazebo object to robot arm')
    parser.add_argument('--attach', action='store_true',help='attach_object')
    args = parser.parse_args()

    test  = object_server()
    if args.attach:
        test.attach_object("kinect_ros", 0.048, -0.05, 1.57)
        #test.attach_object("densefusion_waterer", 0.1, -0.18)
    else:
        test.detach_object()
