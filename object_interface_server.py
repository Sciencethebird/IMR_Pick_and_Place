from utils.gazebo_object_control import gazebo_object
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


attach = False
target = None
offset_transition = np.array( [[1.0, 0.0, 0.0, 0.0],
 					   [0.0, 1.0, 0.0, 0.0],
 					   [0.0, 0.0, 1.0, 0.1],
 					   [0.0, 0.0, 0.0, 1.0]])
offset_rotation = np.array([[1, 0, 0],
						    [0, 1, 0],
						 	[0, 0, 1]])
def callback(data):
	global attach, target 
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	if data.data != "none":
		target = gazebo_object(data.data)
		attach = True
	if data.data == "none":
		target = gazebo_object("none")
		attach = False

def callback_zoffset(data):
	global offset_transition	
	offset_transition[2, 3] = float(data.data)
	#print("model offset: ", offset_RT)

def callback_yoffset(data):
	global offset_transition	
	offset_transition[1, 3] = float(data.data)
	#print("model offset: ", offset_RT)

def callback_xoffset(data):
	global offset_transition	
	offset_transition[0, 3] = float(data.data)
	#print("model offset: ", offset_RT)

def callback_yr(data):
	global offset_rotation	
	offset_rotation = np.array([[np.cos(data.data), 0, np.sin(data.data)],
						 [0                , 1, 0                ],
						 [-np.sin(data.data),0, np.cos(data.data)]])
	#print(offset_RT[0:3, 0:3])
	#print(rotation)
	#offset_RT[0:3, 0:3] = np.matmul(rotation, offset_RT[0:3, 0:3])
	#print(offset_RT)

# rospy init
rospy.init_node('robot_arm_object_interface',anonymous=True)
rospy.Subscriber('/robot_arm/attached_model/name', String, callback)
rospy.Subscriber('/robot_arm/attached_model/xoffset', Float32, callback_xoffset)
rospy.Subscriber('/robot_arm/attached_model/yoffset', Float32, callback_yoffset)
rospy.Subscriber('/robot_arm/attached_model/zoffset', Float32, callback_zoffset)
rospy.Subscriber('/robot_arm/attached_model/y_rotation', Float32, callback_yr)
# gloabl variable

target = gazebo_object("none")


def combined_RT(rvec,tvec, inv = 0):
    
    trivial_row = np.array([0, 0, 0, 1]).reshape((1, 4))
    extrinsic = np.append(rvec, tvec.reshape((3, 1)), axis=1)
    extrinsic = np.append(extrinsic, trivial_row, axis=0)

    return extrinsic


if __name__ == '__main__':

 	rr = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
 	robot = PandaArm()
 	try:
		while True:
			if attach:	
	
				extrinsic = combined_RT(offset_rotation, np.array([0.0, 0.0, 0.0]))
				extrinsic = np.matmul(offset_transition ,extrinsic)
				extrinsic = np.matmul(combined_RT(rr, np.array([0.0, 0.0, 0.0])), extrinsic)
	
				q = quaternion.as_float_array(robot.ee_pose()[1])
				q = Quaternion(q)
					
				tvec = robot.ee_pose()[0]
				rvec = q.rotation_matrix
	
				extrinsic = np.matmul(combined_RT(rvec, tvec), extrinsic)
	
				qq = Quaternion(matrix = extrinsic[0:3, 0:3])
				xyz = extrinsic[0:3, 3].transpose()
					
				target.move_to_quaternion(xyz, qq)
				
	except KeyboardInterrupt:
		print('stopping object interface server.......')