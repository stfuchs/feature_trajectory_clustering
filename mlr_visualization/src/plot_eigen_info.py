#!/usr/bin/env python

import rospy
from mlr_msgs.msg import KernelState
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy import mat,array
from sklearn.cluster import KMeans
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

def symmetric(arr, n):
    M = np.zeros([n,n],np.float32)
    M[np.triu_indices(n,1)] = arr
    return M+M.T+np.diag(np.ones(n,dtype=np.float32))


class PlotNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber("tracking/kernel", KernelState, self.kernel_cb,
                                    queue_size=1, buff_size=2**24)
        self.pub_image = rospy.Publisher("tracking/eigen_info",Image,queue_size=1)
        self.bridge = CvBridge()

    def kernel_cb(self, msg):
        n = len(msg.ids)
        X = mat(symmetric(msg.data,n))
        l = KMeans(2).fit_predict(X)
        idx0 = np.where(l==0)[0]
        idx1 = np.where(l==1)[0]

        try:
            D = mat(np.diag(np.sum(array(X),axis=0)))
            Xnorm = D.I*X
            w,V = np.linalg.eig(Xnorm)
            widx = np.argsort(np.abs(w))[-2:]
            Vp1 = mat(V[:,widx]).T
            Xp = array(Vp1*Xnorm)
            
            w,V = np.linalg.eig(D-X)
            Vp2 = V[:,np.argsort(abs(w))[:2]].T

            w,V = np.linalg.eig(np.identity(n) - Xnorm)
            Vp3 = V[:,np.argsort(abs(w))[:2]].T
        except:
            return

        plt.clf()
        fig, ax = plt.subplots(ncols=1,nrows=1,figsize=(12,8))
        ax.grid()
        ax.set_xlim([-1.,1.])
        ax.set_ylim([-1.,1.])
        ax.plot(Xp[0,idx0], Xp[1,idx0], 'or')
        ax.plot(Xp[0,idx1], Xp[1,idx1], 'ob')
        #ax.plot(Vp2[0,idx0], Vp2[1,idx0], 'o')
        #ax.plot(Vp2[0,idx1], Vp2[1,idx1], 'o')
        ax.plot(-np.abs(Vp3[0,idx0]), Vp3[1,idx0], 'or')
        ax.plot(-np.abs(Vp3[0,idx1]), Vp3[1,idx1], 'ob')
        img = fig2img(fig)
        plt.close('all')
        msg = self.bridge.cv2_to_imgmsg(img,'rgb8')
        self.pub_image.publish(msg)
        


if __name__ == '__main__':
    import sys
    rospy.init_node('plot_node')
    node = PlotNode()
    rospy.spin()
