#!/usr/bin/python
import cv2 
import matplotlib.pyplot as plt
import numpy as np
#import imutils
import rospy
import sensor_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped

class car:
    def __init__ (self):
        #receive
        self.sub = rospy.Subscriber("/csi_cam_0/image_raw", sensor_msgs.msg.Image, self.im_cb)
        self.x = 0
         
        #send
        #see mushr_control for details
        self.pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.pub_cb)
        self.c_locat = 0
        self.ackermann_msg_id = 0
          
    def im_cb(self, img):
        rospy.loginfo("hello")

        # 75, 135,80   105,255,135  blue
        #25,125,0   90,255,255  yellow
        #have to write seperate code that determines color and can set that as the flag color that we are trying to recognize
        f = open("flag_color.txt", "r")
        rgb_color = f.read()
        color = np.uint8(rgb_color)
        hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        hue = hsv_color[0][0][0]
        lower = np.array([str(hue-10)][100][100])
        upper = np.array([str(hue+10)][255][255])

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        #lower = np.array([78,158,124])
        #upper = np.array([138,255,255])
        mask = cv2.inRange(hsv,lower,upper)
        segment = cv2.bitwise_and(img, img, mask=mask)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(contours[0])
        x_coord = rect[0][0]
        #Image processing ex: self.x = detect(img)
        #colormask & compound control
        # add way to determine if the color is on the left, right or in the middle
        # define this as -1, 0, 1 for left, straight, right respectively
        if x_coord < 400:
            self.c_locat = -1
        elif x_coord > 880:
            self.c_locat = 1
        # self.pub.publish(ctrl)
       
    def pub_cb(self, event):
        rospy.loginfo("hello2")

        #calc control from self.x & self.if.Flag
        ctrl = [0, 0]
        if self.c_locat < 0:
            ctrl[0] = 2
            ctrl[1] = 1
            rospy.loginfo("hello3")
        elif self.c_locat > 0:
            ctrl[0] = 2
            ctrl[1] = -1
        else:
            ctrl[0] = 2
            rospy.loginfo("hello4")

        rospy.loginfo("hello4")
        assert len(ctrl) == 2
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.header.seq = self.ackermann_msg_id
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        self.pub.publish(ctrlmsg)
        self.ackermann_msg_id += 1

        self.pub.publish(ctrlmsg)



def main():
    rospy.init_node("cptfnode")
    rospy.loginfo("main()")
    node = car()
    rospy.spin()

if __name__ == '__main__':
    main()

