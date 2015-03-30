#!/usr/bin/env python

import rospy
from mlr_msgs.msg import KernelState
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

def fig2img(fig):
    fig.canvas.draw()
    w,h = fig.canvas.get_width_height()
    #buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    #buf = numpy.roll(buf,3,axis=2)
    buf = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
    buf.shape = (h,w,3)
    buf.reshape(h,w,3)
    #buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    return buf

class PlotNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber("tracking/kernel", KernelState, self.kernel_cb,
                                    queue_size=1, buff_size=2**24)
        self.pub_image = rospy.Publisher("tracking/variance",Image,queue_size=1)
        self.bridge = CvBridge()

    def kernel_cb(self, msg):
        data = np.sort(msg.data)
        fig, ax = plt.subplots(ncols=1,nrows=2,figsize=(12,8))
        ax[0].grid()
        ax[0].set_ylim([0,.05])
        #ax.plot(data[data>.0001],'x-')
        ax[0].plot(data, 'x-')
        ax[1].grid()
        ax[1].set_ylim([0,1.])
        ax[1].plot(np.exp(-10000.*data))
        img = fig2img(fig)
        plt.close('all')
        msg = self.bridge.cv2_to_imgmsg(img,'rgb8')
        self.pub_image.publish(msg)
        


if __name__ == '__main__':
    import sys
    rospy.init_node('plot_node')
    node = PlotNode()
    rospy.spin()
