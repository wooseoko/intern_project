import tensorflow as tf
import random

def model (X):
    
    L1 = tf.layers.conv2d(X,32,5,strides=[2,2],padding = 'same')
    L1 = tf.layers.max_pooling2d(L1,3,2)
#    L2 = tf.layers.batch_normalization(L1, training = True)
    L2 = tf.nn.relu(L1)
    L2 = tf.layers.conv2d(L2,32,3,strides=[2,2],padding='same')
#    L2 = tf.layers.batch_normalization(L2, training = True)
    L2 = tf.nn.relu(L2)
    L2 = tf.layers.conv2d(L2,32,3,strides=[1,1],padding='same')
    L1 = tf.layers.conv2d(L1,32,1,strides=[2,2],padding='same')
    L3 = tf.add(L1,L2)
    
#    L4 = tf.layers.batch_normalization(L3, training = True)
    L4 = tf.nn.relu(L3)
    L4 = tf.layers.conv2d(L4,64,3,strides=[2,2],padding='same')
#    L4 = tf.layers.batch_normalization(L4, training = True)
    L4 = tf.nn.relu(L4)
    L4 = tf.layers.conv2d(L4,64,3,strides=[1,1],padding='same')
    L3 = tf.layers.conv2d(L3,64,1,strides=[2,2],padding='same')
    L5 = tf.add(L3,L4)
    
#    L6 = tf.layers.batch_normalization(L5, training = True)
    L6 = tf.nn.relu(L5)
    L6 = tf.layers.conv2d(L4,128,3,strides=[2,2],padding='same')
#    L6 = tf.layers.batch_normalization(L6, training = True)
    L6 = tf.nn.relu(L6)
    L6 = tf.layers.conv2d(L6,128,3,strides=[1,1],padding='same')
    L5 = tf.layers.conv2d(L5,128,1,strides=[2,2],padding='same')
    L7 = tf.add(L5,L6)
    
    X_ = tf.layers.flatten(L7)
    X_ = tf.nn.relu(X_)
    X_ = tf.layers.dropout(X_)
    steer = tf.layers.dense(X_,3)
    return steer
