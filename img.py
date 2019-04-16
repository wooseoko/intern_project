#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import model
import tensorflow as tf
import os
import numpy as np



class get_image:
  def __init__(self):
    self.bridge = CvBridge()
    self.steer = 0
    self.X = tf.placeholder(tf.float32, [None, 200, 200, 3])
    self.steering = model.model(self.X)
    self.predictions = tf.nn.softmax(self.steering)
    self.SAVER_DIR = "/home/wooseoko/catkin_ws/src/project2/src/model"
    self.saver = tf.train.Saver()
    self.checkpoint_path = os.path.join(self.SAVER_DIR,"model")
    self.ckpt = tf.train.get_checkpoint_state(self.SAVER_DIR)
    self.sess = tf.Session()
    self.sess.run(tf.global_variables_initializer())
    self.saver.restore(self.sess, self.ckpt.model_checkpoint_path)      
    self.steering_pub = rospy.Publisher("steering_by_network",Float32,queue_size = 1)
    self.image_sub = rospy.Subscriber("/camera/zed/rgb/image_rect_color", Image,self.callback)
    self.cnt = 0

  def callback(self,data):
    try:
      src = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    src = cv2.resize(src, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    if(200 > src.shape[0]):
      src = cv2.resize(src, (src.shape[1],200), interpolation = cv2.INTER_CUBIC)
    if(200 > src.shape[1]):
      src = cv2.resize(src, (200, src.shape[0]), interpolation = cv2.INTER_CUBIC)
    height,width,_ = src.shape
    middle = int(width/2)
    src = src[height-200 : height, middle - 100 : middle + 100]
    cv_image = np.asarray(src, dtype = np.float32)/255.0
      
    batch_image = np.zeros((1, 200, 200 ,3))
    batch_image[0,:,:,:] = cv_image
#    steer = self.sess.run(tf.argmax(self.predictions,1)[0],feed_dict = {self.X:batch_image})
    steer_action_value = self.sess.run(self.predictions,feed_dict = {self.X:batch_image})
    steer = steer_action_value[0][0] * -0.8 + steer_action_value[0][2] * 0.8
#    print("%d  "%self.cnt)
#    print(steer)
    self.steering_pub.publish(steer)
#    self.cnt+=1
def main(args):
  ic = get_image()
  print("get_image")
  rospy.init_node('get_image', anonymous=True)
  try:
    rospy. spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

