#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as IMG
import numpy as np

#os.makedirs(rgb_path)
#os.makedirs(depth_path)

img_idx = 0

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback2)
    self.trigger_sub = rospy.Subscriber("image_save_trigger",String,self.callback3)
    self.rgb_img = None
    self.depth_img = None
    self.saving_rgb = False
    self.saving_depth = False

  def callback(self,data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.rgb_img = cv_image
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)

    #key = cv2.waitKey(3)

  def callback2(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
      self.depth_img = cv_image
    except CvBridgeError as e:
      print(e)

  def callback3(self,data):
    global img_idx
    print("read save trigger", data.data)
    if(data.data == '1'):
      self.save_imgs()

  # mask mode : save only rgb under mask folder
  def save_imgs(self, mask_mode = False):
      self.depth_img = self.depth_img*1000
      save_path = ""
      cv2.imwrite("test.png", self.rgb_img)
      #print(os.path.join(save_path , '%04ddepth.png' % (img_idx,)))
      cv2.imwrite(os.path.join(save_path , '%04ddepth.png' % (img_idx,)), self.depth_img.astype(np.uint16))
      cv2.imwrite(os.path.join(save_path , '%04drgb.png' % (img_idx,)), self.rgb_img)
      print('image saved: ',  '%04d.png' % (img_idx,) )
      #img_idx+=1
 


def main(args):
  ic = image_converter()
  #dic = depth_image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)