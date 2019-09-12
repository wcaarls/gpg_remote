#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import io
import socket
import struct
import numpy as np

def fake_depth_publisher():
    # Setup ROS    
    pub = rospy.Publisher('depth', Image, queue_size=1, latch=True)
    rospy.init_node('fake_depth_publisher', anonymous=True)
    bridge = CvBridge()
    
    image = np.zeros((240, 320), np.float32)
    image[:] = 1
    
    msg = bridge.cv2_to_imgmsg(image, "passthrough")
    msg.header.frame_id = "camera_link"
    
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
      msg.header.stamp = rospy.Time.now()
      pub.publish(msg)
      r.sleep()

if __name__ == '__main__':
    try:
        fake_depth_publisher()
    except rospy.ROSInterruptException:
        pass
