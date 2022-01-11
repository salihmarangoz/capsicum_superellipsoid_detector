#!/usr/bin/env python3

import rospy

from yolact_ros_msgs.msg import Detections
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np


class MaskFilter():
    def __init__(self):
        rospy.init_node('maskfilter', anonymous=True)
        self.bridge = CvBridge()
        self.roi_pub = rospy.Publisher("roi_depth",Image, queue_size=2)
        self.nonroi_pub = rospy.Publisher("nonroi_depth",Image, queue_size=2) # for debugging purposes

        self.depth_sub = Subscriber("input_depth", Image)
        self.detection_sub = Subscriber("input_detection", Detections)

        self.ats = ApproximateTimeSynchronizer([self.depth_sub, self.detection_sub], queue_size=300, slop=0.1)
        self.ats.registerCallback(self.callback)

    def callback(self, depth_msg, detection_msg):
        if self.roi_pub.get_num_connections()==0 and self.nonroi_pub.get_num_connections()==0:
            return

        rospy.loginfo_once("Depth and detection received!")

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_image = depth_image.reshape((depth_msg.height, depth_msg.width))

        roi_mask = np.zeros(depth_image.shape, dtype=np.float32)

        for detection in detection_msg.detections:
            class_name, score, box, mask_msg = detection.class_name, detection.score, detection.box, detection.mask
            width, height, mask_raw = mask_msg.width, mask_msg.height, mask_msg.mask 
            x1, y1, x2, y2 = box.x1, box.y1, box.x2, box.y2

            mask = np.unpackbits(np.frombuffer(mask_raw, dtype=np.uint8), count=width*height).reshape(height, width)
            roi_mask[y1:y2, x1:x2] = mask

        roi_mask = np.array(roi_mask, dtype=np.bool)

        if self.roi_pub.get_num_connections()>0:
            roi_depth_image = np.zeros(depth_image.shape, dtype=np.uint16)
            #roi_depth_image[:,:] = 1 # np.nan
            roi_depth_image[roi_mask] = depth_image[roi_mask]
            roi_msg = self.bridge.cv2_to_imgmsg(roi_depth_image, "passthrough")
            roi_msg.header = depth_msg.header
            self.roi_pub.publish(roi_msg)

        if self.nonroi_pub.get_num_connections()>0:
            nonroi_mask = np.invert(roi_mask.copy())
            nonroi_depth_image = np.zeros(depth_image.shape, dtype=np.uint16)
            #nonroi_depth_image[:,:] = 1 # np.nan
            nonroi_depth_image[nonroi_mask] = depth_image[nonroi_mask]
            nonroi_msg = self.bridge.cv2_to_imgmsg(nonroi_depth_image, "passthrough")
            nonroi_msg.header = depth_msg.header
            self.nonroi_pub.publish(nonroi_msg)


if __name__ == '__main__':
    MaskFilter()
    rospy.spin()