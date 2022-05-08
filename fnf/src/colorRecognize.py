from cv_bridge import CvBridge
import cv2
import numpy as np

def main():
    __init__node()

    
    img = rospy.wait_for_message("/csi_cam_0/imageraw", sensor_msgs.msg.Image, timeout=None)

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

    #detect the color of the image
    #save to file the three part array/coordinates
    #write that part into the im_cb function in captureFlag




