#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Empty

pm_service_path = '/voxblox_node/publish_pointclouds'
pm_rate = 1

def call_empty_service():
    rospy.wait_for_service(pm_service_path)
    try:
        service_func = rospy.ServiceProxy(pm_service_path, Empty)
        service_func()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('trigger_voxblox', anonymous=True)

    pm_service_path = rospy.get_param('service_path', pm_service_path)
    pm_rate = rospy.get_param('rate', pm_rate)

    rate = rospy.Rate(pm_rate)
    while not rospy.is_shutdown():
        call_empty_service()
        rate.sleep()


