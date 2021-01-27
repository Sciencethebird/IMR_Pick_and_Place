import rospy
from panda_robot import PandaArm
import tf_conversions as tf
import franka_interface
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from utils.object_attach_control import object_server
from df.df_inference import densefusion_docker
import quaternion # for processing ee_pose() output
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2

def combined_RT(rvec,tvec, inv = 0):
    
    trivial_row = np.array([0, 0, 0, 1]).reshape((1, 4))
    extrinsic = np.append(rvec, tvec.reshape((3, 1)), axis=1)
    extrinsic = np.append(extrinsic, trivial_row, axis=0)

    return extrinsic

# camera offset relative to gripper
offset_transition = np.array(  [[1.0, 0.0, 0.0, 0.048],
 					   			[0.0, 1.0, 0.0, 0.000],
 					   			[0.0, 0.0, 1.0,-0.052],
 					   			[0.0, 0.0, 0.0, 1.000]])


offset_rotation = R.from_rotvec(np.pi/180 * np.array([0, 90, 0])).as_dcm()
offset_rotation = combined_RT(offset_rotation, np.array([0.0, 0.0, 0.0]))

camera_rot_to_gripper = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
image_to_camera = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])

# camera to gripper extrinsic
extrinsic = combined_RT(image_to_camera, np.array([0.0, 0.0, 0.0]))
extrinsic = np.matmul(offset_rotation, extrinsic)
extrinsic = np.matmul(offset_transition, extrinsic)
camera_to_gripper = np.matmul(combined_RT(camera_rot_to_gripper, np.array([0.0, 0.0, 0.0])), extrinsic)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
scene = moveit_commander.PlanningSceneInterface()

robot_arm = PandaArm()
robot_gripper = franka_interface.GripperInterface()
obj_server = object_server()
pose_estimator = densefusion_docker()

if __name__ == '__main__':

	for i in range(10):

		obj_server.attach_object("kinect_ros_0", 0.05, -0.05, 0.0)

	# init robot arm state
	robot_arm.move_to_neutral()
	robot_arm.get_gripper().home_joints()
	robot_arm.get_gripper().open()


	print( robot_arm.ee_pose() )

	xr = R.from_rotvec(np.pi/180 * np.array([180, 0, 0])).as_dcm()
	yr = R.from_rotvec(np.pi/180 * np.array([0, -30, 0])).as_dcm()

	observe_pose_rvec = np.matmul(yr, xr)
	q = Quaternion(matrix = observe_pose_rvec)
	observe_pose_q = [ q[1], q[2], q[3], q[0] ]
	print(observe_pose_q)

	neutral = [ 1, 0, 0, 0 ] # neutral oriantation (x, y, z, w)

	# move to observation pos
	valid, pos = robot_arm.inverse_kinematics([0.2, 0.13528, 0.6], observe_pose_q)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)
	time.sleep(2)

	# pose estimation
	pose_estimator.predict()

	# load predicted RT
	pred_t = np.load('/home/birb/DenseFusion/ros_image_buffer/tvec.npy')
	print(pred_t)
	pred_r = np.load('/home/birb/DenseFusion/ros_image_buffer/rvec.npy')
	print(pred_r)

	# show predict image
	pred_img = cv2.imread('/home/birb/DenseFusion/ros_image_buffer/0000rgb-visual.png')
	time_count = 0
	cv2.imshow('pred_result', pred_img)
	cv2.waitKey()
	cv2.destroyAllWindows()

	# read current gripper position
	print(robot_arm.ee_pose()[0])
	q = quaternion.as_float_array(robot_arm.ee_pose()[1])
	q = Quaternion(q)
		
	tvec = robot_arm.ee_pose()[0]
	rvec = q.rotation_matrix

	# calculate object to base extrinsic
	print("ctg")
	print(camera_to_gripper)
	extrinsic = np.matmul(camera_to_gripper, combined_RT(pred_r, pred_t))
	print(extrinsic)
	extrinsic = np.matmul(combined_RT(rvec, tvec), extrinsic)

	print("object_to_base")
	print(extrinsic)

	# extract object x, y, z from extrinsic
	chimp_xyz = extrinsic[0:3, 3]
	chimp_xyz[2] += 0.1 # for clearence purpose
	print(chimp_xyz)
	
	# move to object pos
	valid, pos = robot_arm.inverse_kinematics(chimp_xyz ,neutral)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	# grasp object
	robot_arm.get_gripper().open()
	print(robot_gripper.grasp(0.04, 10))
	obj_server.attach_object("densefusion_chimp", z_offset = -0.12, y_offset = -0.01)
	time.sleep(2)

	# move to location 2
	valid, pos = robot_arm.inverse_kinematics([0.489668, -0.0, 0.60424],neutral)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	

	# move to target location
	valid, pos = robot_arm.inverse_kinematics([0.489668, -0.221428, 0.38],neutral)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	# release object
	robot_arm.get_gripper().open()
	obj_server.detach_object()
	time.sleep(2)

	# move to leaving pos
	valid, pos = robot_arm.inverse_kinematics([0.3, -0.221428, 0.490424],neutral)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)


	robot_arm.get_gripper().home_joints()
	robot_arm.move_to_neutral()
	