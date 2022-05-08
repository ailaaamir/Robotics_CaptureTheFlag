class car:
    def __init__ (self):
        #receive
        self.sub = rospy.Subscriber(/csi_cam_0/imageraw, sensor_msgs.msg.Image, self.im_cb)
        self.x = 0
         
        #send
        #see mushr_control for details
        self.pub = rospy.Publisher(/vesc/mux/.../navigation, AckermanDriveStamped)
        self.timer = rospy.Timer(rospy.Duration(), self.pub_cb)
          
          
    def im_cb(img):
        #Image processing ex: self.x = detect(img)
        #colormask & compound control
        prp.publish(ctrl)
       
    def pub.cb(event):
        #calc control from self.x & self.if.Flag
        self.pub.publish(ctrl)




def main():
    init_node()
  #  node = fastnfurious()
  rospy.spin()
