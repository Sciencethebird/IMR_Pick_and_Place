import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
import numpy as np
import math
import random
import time
import argparse


class gazebo_object():
    def __init__(self, object_name):

        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=2)
        # clear file name index
        try: 
            rospy.init_node("gazebo_object_controller", anonymous=True)
        except:
            print "gazebo_object_controller is already initialized!"
        self.object_msg = ModelState()
        self.object_msg.model_name = object_name
        self.object_msg.reference_frame = 'world'
        #static position
        self.object_msg.pose.position.x = 0.0
        self.object_msg.pose.position.y = 0.0
        self.object_msg.pose.position.z = 0.5
        self.object_msg.pose.orientation.x = 0.0
        self.object_msg.pose.orientation.y = 0.0
        self.object_msg.pose.orientation.z = 0.0
        self.object_msg.pose.orientation.w = 0.0
        # motion
        self.object_msg.twist.linear.x = 0.0
        self.object_msg.twist.linear.y = 0.0
        self.object_msg.twist.linear.z = 0.0
        self.object_msg.twist.angular.x = 0.0
        self.object_msg.twist.angular.y = 0.0
        self.object_msg.twist.angular.z = 0.0

    def move_to(self, x, y, z, i = 0.0, j= 0.0, k= 0.0, w=0.0):
        # x, y, z
        self.object_msg.pose.position.x = x
        self.object_msg.pose.position.y = y
        self.object_msg.pose.position.z = z
        # quaternions
        self.object_msg.pose.orientation.x = i
        self.object_msg.pose.orientation.y = j
        self.object_msg.pose.orientation.z = k
        self.object_msg.pose.orientation.w = w

        self.pub_message()

    def move_to_quaternion(self, xyz, q):
        # x, y, z
        self.object_msg.pose.position.x = xyz[0]
        self.object_msg.pose.position.y = xyz[1]
        self.object_msg.pose.position.z = xyz[2]
        # quaternions
        self.object_msg.pose.orientation.w = q[0]
        self.object_msg.pose.orientation.x = q[1]
        self.object_msg.pose.orientation.y = q[2]
        self.object_msg.pose.orientation.z = q[3]

        #self.object_msg.twist.linear.x = linear[0]
        #self.object_msg.twist.linear.y = linear[1]
        #self.object_msg.twist.linear.z = linear[2]

        #self.object_msg.twist.angular.x = angular[0]
        #self.object_msg.twist.angular.y = angular[1]
        #self.object_msg.twist.angular.z = angular[2]

        self.pub_message()

    def set_eular_angle(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        #return [qx, qy, qz, qw]

        self.object_msg.pose.orientation.x = qx
        self.object_msg.pose.orientation.y = qy
        self.object_msg.pose.orientation.z = qz
        self.object_msg.pose.orientation.w = qw
        self.pub_message()

    def point_object_at(self, px, py, pz, theta=0.0):
        
        dx = px - self.object_msg.pose.position.x
        dy = py - self.object_msg.pose.position.y
        dz = pz - self.object_msg.pose.position.z

        base = np.sqrt(np.sum(np.square([dx, dy])))

        yaw = np.arctan2(dy,dx)
        pitch = -1.0*np.arctan2(dz,base)

        print('pitch', pitch)
        self.set_eular_angle(0.0, pitch, yaw)
        

    def send_save_signal(self):
        if args.mask:
            self.save_trigger.publish("10")
        else:
            self.save_trigger.publish("11")

    def pub_message(self):
        rate = rospy.Rate(1000)
        #rospy.loginfo(self.object_msg)
        for i in range(3):
            self.pub.publish(self.object_msg)
            #rate.sleep()



if __name__ == '__main__':

    waterer = gazebo_object("densefusion_waterer_3")
    chimp = gazebo_object("densefusion_chimp")
    #x = 0.1
    waterer.move_to(0, 0, 0.1, -1.57569726e+00, -4.82374882e-004,  2.15975946e+00,  4.36766917e-001)
    for x in range(10):
        print(x/10.0)
        waterer.move_to(x/10.0, 0, 0.1, -1.57569726e+00, -4.82374882e-004,  2.15975946e+00,  4.36766917e-001)