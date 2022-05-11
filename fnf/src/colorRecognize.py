#!/usr/bin/python
import cv_bridge
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs
import rospy

def main():
    rospy.init_node("rnode")

    rospy.loginfo("hello")
    img = rospy.wait_for_message("/csi_cam_0/image_raw", sensor_msgs.msg.Image, timeout=None)

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    color = cv_image[360,640]

    rospy.loginfo("hello2")
    f = open("flag_color.txt", "w")
    f.write(color)
    f.close()

    #detect the color of the image
    #save to file the three part array/coordinates
    #write that part into the im_cb function in captureFlag


if __name__ == '__main__':
    main()

