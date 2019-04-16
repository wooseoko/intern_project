import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
from glob import glob

def img_process(fname, cwidth, cheight):
    src = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)
    if(cheight > src.shape[0]):
        src = cv2.resize(src, (src.shape[1],cheight), interpolation = cv2.INTER_CUBIC)
    if(cwidth > src.shape[1]):
        src = cv2.resize(src, (cwidth, src.shape[0]), interpolation = cv2.INTER_CUBIC)
    height,width = src.shape
    middle = int(width/2)
    src = src[height-cheight : height, middle - int(cwidth/2) : middle + int(cwidth/2)]
    return np.asarray(src, dtype = np.float32)

def batch_data(data_list, data_height, data_width, start_point ,batch_size = 32):
    batch_image = np.zeros((batch_size, data_height, data_width ,1))
    batch_label = np.zeros((batch_size,3))
    for n, path in enumerate(data_list[start_point:start_point+batch_size]):
        image = img_process(path, data_width,data_height)
        label = get_label_from_path(path)
        batch_image[n,:,:,0] = image
        batch_label[n] = label
    return batch_image, batch_label

def get_label_from_path(path):
    label = np.zeros(3)
    label[int(path.split('/')[1].split('.jpeg')[0])+1] = 1
    return label
