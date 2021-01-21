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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
scene = moveit_commander.PlanningSceneInterface()

robot_arm = PandaArm()
robot_gripper = franka_interface.GripperInterface()
obj_server = object_server()

if __name__ == '__main__':

	for i in range(10):

		obj_server.attach_object("kinect_ros_0", 0.05, -0.05, 0.0)

	# init robot arm state
	robot_arm.move_to_neutral()
	robot_arm.get_gripper().home_joints()
	robot_arm.get_gripper().open()


	print( robot_arm.ee_pose() )
	q = tf.transformations.quaternion_from_euler(0, 0, 0)
	print(q)
	ori = [0.00841868210081883, 0.99965734551983, -0.0142474501592607, 0.0202812106120972]
	ori = [ 1, 0, 0, 0 ]

	# move entry pos
	valid, pos = robot_arm.inverse_kinematics([0.3, 0.13528, 0.488424],ori)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	# move to object pos
	valid, pos = robot_arm.inverse_kinematics([0.489668, 0.13528, 0.488424],ori)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	# grasp object
	robot_arm.get_gripper().open()
	print(robot_gripper.grasp(0.001, 10))
	obj_server.attach_object("densefusion_waterer", z_offset = -0.17, y_offset = -0.01)
	time.sleep(2)

	# move to location 2
	valid, pos = robot_arm.inverse_kinematics([0.489668, -0.0, 0.60424],ori)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	

	# move to target location
	valid, pos = robot_arm.inverse_kinematics([0.489668, -0.221428, 0.480424],ori)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)

	# release object
	robot_arm.get_gripper().open()
	obj_server.detach_object()
	time.sleep(2)

	# move to leaving pos
	valid, pos = robot_arm.inverse_kinematics([0.3, -0.221428, 0.480424],ori)
	if valid:
		print("find inverse kinematic")
	robot_arm.move_to_joint_position(pos)


	robot_arm.get_gripper().home_joints()
	robot_arm.move_to_neutral()
