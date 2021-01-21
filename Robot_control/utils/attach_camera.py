from move_gazebo_object import gazebo_object
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


offset_transition = np.array(  [[1.0, 0.0, 0.0, 0.048],
 					   			[0.0, 1.0, 0.0, 0.000],
 					   			[0.0, 0.0, 1.0,-0.050],
 					   			[0.0, 0.0, 0.0, 1.000]])
offset_rotation = np.array([[ np.cos(1.5708), 0, np.sin(1.5708)],
						    [              0, 1,             0 ],
						    [-np.sin(1.5708), 0, np.cos(1.5708)]])


target = gazebo_object("kinect_ros")


def combined_RT(rvec,tvec, inv = 0):
    
    trivial_row = np.array([0, 0, 0, 1]).reshape((1, 4))
    extrinsic = np.append(rvec, tvec.reshape((3, 1)), axis=1)
    extrinsic = np.append(extrinsic, trivial_row, axis=0)

    return extrinsic


if __name__ == '__main__':

 	rr = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
 	robot = PandaArm()
	while True:

			extrinsic = combined_RT(offset_rotation, np.array([0.0, 0.0, 0.0]))
			extrinsic = np.matmul(offset_transition ,extrinsic)
			extrinsic = np.matmul(combined_RT(rr, np.array([0.0, 0.0, 0.0])), extrinsic)

			#time.sleep(0.0000001)
			#print(robot.ee_pose()[1][1])
			q = quaternion.as_float_array(robot.ee_pose()[1])
			q = Quaternion(q)
			
			tvec = robot.ee_pose()[0]
			rvec = q.rotation_matrix

			extrinsic = np.matmul(combined_RT(rvec, tvec), extrinsic)
			# init quaternion from rotation matrix
			qq = Quaternion(matrix = extrinsic[0:3, 0:3])
			xyz = extrinsic[0:3, 3].transpose()
			
			target.move_to_quaternion(xyz, qq)

