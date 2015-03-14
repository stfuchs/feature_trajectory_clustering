#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
#from std_msgs.msg Import Header
from cv_bridge import CvBridge

class Video:
    def __init__(self, source, rate):
        self.cap = cv2.VideoCapture(source)
        self.pub = rospy.Publisher("/camera/rgb/image_color",Image,queue_size=1)
        self.bridge = CvBridge()
        self.r = rospy.Rate(int(rate))
        
    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret == False:
                break
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            msg = self.bridge.cv2_to_imgmsg(cv2.resize(frame, (640,480)),'bgr8')
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "/videofile"
            self.pub.publish(msg)
            self.r.sleep()

        self.cap.release()

if __name__ == '__main__':
    import sys
    try:
        src = sys.argv[1]
    except:
        src = 0
    try:
        rate = sys.argv[2]
    except:
        rate = 30
    print """ videofile publisher
    usage: pub_video.py video_source [rate in hz]
    currently, crops video to 640x480
    """
    rospy.init_node('videofile_publisher')
    Video(src,rate).run()
    #cv2.destroyAllWindows()
