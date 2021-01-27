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
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
scene = moveit_commander.PlanningSceneInterface()
#rospy.init_node("panda_demo")
r = PandaArm()


while True:	
	time.sleep(0.1)
	print(r.ee_pose())
	
