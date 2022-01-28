#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

import scipy
from scipy.interpolate import griddata
from scipy import ndimage
import cv2

class MaskFilter():
    def __init__(self):
        rospy.init_node('maskfilter', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/camera/depth/image_raw_noisy",Image, queue_size=2)
        self.sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)
        self.phase = 0

    def callback(self, depth_msg):
        rospy.loginfo_once("Depth is received!")

        # Prepare depth_image
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_image = depth_image.reshape((depth_msg.height, depth_msg.width))
        depth_image = np.nan_to_num(depth_image)
        depth_image[depth_image==0] = 1000.0

        # Add 2D sinwaves
        ii, jj = np.mgrid[0:depth_image.shape[0], 0:depth_image.shape[1]]
        ii_ = ii.reshape(-1, 1)
        jj_ = jj.reshape(-1, 1)
        depth_image_ = depth_image.reshape(-1, 1)

        ii_add = np.array([0.3, 0.07, 0.1]).reshape(1,-1)
        jj_add = np.array([0.2, 0.1, 0.5]).reshape(1,-1)
        phase_add = np.array([1.1, 2.3, 2.9]).reshape(1,-1)
        mag_add = np.array([0.0027, 0.0039, 0.003]).reshape(1,-1)

        ii_mul = np.array([0.2, 0.22]).reshape(1,-1)
        jj_mul = np.array([0.3, 0.1]).reshape(1,-1)
        phase_mul = np.array([1.9, 1.7]).reshape(1,-1)
        mag_mul = np.array([0.004, 0.015]).reshape(1,-1)

        noise_mul = depth_image_ * mag_mul * np.sin(ii_*ii_mul + jj_*jj_mul + self.phase*phase_mul)
        noise_add = mag_add * np.sin(ii_*ii_add + jj_*jj_add + self.phase*phase_add)
        noise = noise_mul.mean(axis=1).reshape(depth_image.shape) + noise_add.mean(axis=1).reshape(depth_image.shape)

        self.phase += 1
        if self.phase > 999999:
            self.phase = 0
        noisy_depth_image = depth_image + noise

        if False:
            plt.clf()
            plt.imshow(noise)
            plt.colorbar()
            plt.pause(0.0000001)


        # Add shadowing effect
        noisy_depth_image = scipy.ndimage.gaussian_filter(noisy_depth_image, sigma=0.5)

        # Add gaussian noise
        noisy_depth_image += noisy_depth_image * np.random.normal(0,0.003,noisy_depth_image.shape) + np.random.normal(0,0.001,noisy_depth_image.shape)

        # Remove non-positive and far away points
        depth_uint = noisy_depth_image*1000
        mask1 = np.bitwise_or(depth_uint<0, depth_uint>10000)
        depth_uint[mask1] = 0

        # Add salt and pepper noise for zero pixels
        saltpepper_prob = 0.003
        mask2 = np.random.choice(2, mask1.shape, p=[1-saltpepper_prob, saltpepper_prob])
        mask3 = np.bitwise_and(mask1, mask2).astype(np.bool)
        randdist = np.random.randint(10000, size=mask3.sum())
        depth_uint[mask3] = randdist

        # publish message
        depth_uint = depth_uint.astype(np.uint16)
        roi_msg = self.bridge.cv2_to_imgmsg(depth_uint, '16UC1')
        roi_msg.header = depth_msg.header
        #roi_msg.header.stamp = rospy.get_rostime()
        self.pub.publish(roi_msg)



if __name__ == '__main__':
    MaskFilter()
    rospy.spin()