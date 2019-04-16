#!/usr/bin/env python
from keras.models import model_from_json
from keras import backend as K
import tensorflow as tf
import numpy as np
import rospy
import sys
import roslib
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#Dronet class :
#get image by callback, get model , image to model -> output (steer, coll)
class Dronet(object):
#initial. set publisher, subsctriber
    def __init__(self):
	self.bridge = CvBridge()

	self.steer=0
	self.coll = 0
	self.target_size=(320,240)
	self.crop_size = (200,200)
	K.clear_session()
	K.set_learning_phase(0)
	model = self.jsonToModel("/home/wooseoko/catkin_ws/src/project2/models/model_struct.json")
	model.load_weights("/home/wooseoko/catkin_ws/src/project2/models/best_weights.h5")
	model.compile(loss='mse', optimizer='sgd')
	self.graph = tf.get_default_graph()
	self.model = model
        self.steering_pub = rospy.Publisher("steering_by_network",Float32,queue_size = 1)
	self.coll_prob_pub = rospy.Publisher("collision_prob",Float32,queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/zed/rgb/image_rect_color", Image,self.callback_image)

#get image

    def callback_image(self,data):
	image_type = data.encoding
	self.src = self.bridge.imgmsg_to_cv2(data,image_type)
	self.src = cv2.resize(self.src, self.target_size)
	self.src = cv2.cvtColor(self.src, cv2.COLOR_BGR2GRAY)
	half_the_width = self.src.shape[1] / 2
	self.src = self.src[(self.src.shape[0] - self.crop_size[0]) : self.src.shape[0], (half_the_width - (self.crop_size[1] / 2)): (half_the_width + (self.crop_size[1]  / 2))]

	self.src = self.src.reshape(self.src.shape[0],self.src.shape[1],1)
	self.src = np.asarray(self.src, dtype=np.float32) * np.float32(1.0/255.0)
	with self.graph.as_default():
	    outs = self.model.predict_on_batch(self.src[None])
	    self.steer , self.coll = outs[0][0] , outs[1][0]
	self.steering_pub.publish(self.steer)
	self.coll_prob_pub.publish(self.coll)

#get model in json file

    def jsonToModel(self,json_model_path):
	with open(json_model_path, 'r') as json_file:
	    loaded_model_json = json_file.read()
	model = model_from_json(loaded_model_json)
	return model

#mains

def main(args):
    print("start")
    dronet = Dronet()
    rospy.init_node('dronet', anonymous=True)
    rospy.spin()
    cv2.destroyAllWindows()
if(__name__ == '__main__'):
    print("main")
    main(sys.argv)
