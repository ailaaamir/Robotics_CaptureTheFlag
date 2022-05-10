import cv2 
import matplotlib.pyplot as plt
import numpy as np
import imutils

class car:
    def __init__ (self):
        #receive
        self.sub = rospy.Subscriber("/csi_cam_0/imageraw", sensor_msgs.msg.Image, self.im_cb)
        self.x = 0
         
        #send
        #see mushr_control for details
        self.pub = rospy.Publisher(/vesc/mux/.../navigation, AckermanDriveStamped)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_cb)
          
          
    def im_cb(self, img):

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
        segment = cv2.bitwise_and(imge, imge, mask=mask)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(contours[0])
        x_coord = rect[0][0]
        #Image processing ex: self.x = detect(img)
        #colormask & compound control
        # add way to determine if the color is on the left, right or in the middle
        # define this as -1, 0, 1 for left, straight, right respectively
        self.c_locat = 0
        if x_coord < 400:
            self.c_locat = -1
        elif x_coord > 880:
            self.c_locat = 1
        prp.publish(ctrl)
       
    def pub_cb(self, event):
        #calc control from self.x & self.if.Flag
        self.pub.publish(ctrl)

        if c_locat < 0:
            ctrl[0] = 2
            ctrl[1] = 0.2
        elif c_locat > 0:
            ctrl[0] = 2
            ctrl[1] = -0.2
        else:
            ctrl[0] = 2

        assert len(ctrl) == 2
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.header.seq = self.ackermann_msg_id
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        self.pub.publish(ctrlmsg)
        self.ackermann_msg_id += 1




def main():
    __init__node()
  #  node = fastnfurious()
  rospy.spin()
