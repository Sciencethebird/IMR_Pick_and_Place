import cv2
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import subprocess
from std_msgs.msg import Int32
import rospy
from std_msgs.msg import String

class densefusion_docker():
	def __init__(self):
		self.pub = rospy.Publisher("image_save_trigger", String, queue_size=2)
		try: 
			rospy.init_node("gazebo_object_controller", anonymous=True)
		except:
			print "gazebo_object_controller is already initialized!"
	def predict(self):
		self.pub_message()
		print("calling densefusion docker")
		os.system("pwd")
		os.system("bash df/df_docker_predict.sh")
		print("prediction done")
		return 

	def pub_message(self):
		rate = rospy.Rate(1000)
    	#rospy.loginfo(self.object_msg)
		for i in range(1):
			self.pub.publish('1')


if __name__ == '__main__':
	test = densefusion_docker()
	test.predict()