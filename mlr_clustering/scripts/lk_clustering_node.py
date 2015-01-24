#!/usr/bin/env python

import rospy
from mlr_msgs.msg import KernelState, ObjectIds
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge
from collections import defaultdict


class ClusteringNode:
    def __init__(self):
        self.sub = rospy.Subscriber("lk_kernel",KernelState,self.lk_callback,queue_size=1)
        self.pub_image = rospy.Publisher("lk_kernel_matrix",Image,queue_size=1)
        self.pub_objs = rospy.Publisher("lk_objects",ObjectIds,queue_size=1)
        self.bridge = CvBridge()
        self.n_cluster = 8

        from dynamic_reconfigure.server import Server as DynReconfServer
        from mlr_clustering.cfg import lk_clustering_paramsConfig
        self.server = DynReconfServer(lk_clustering_paramsConfig, self.reconfigure)

        import matplotlib.pyplot as plt
        from matplotlib import cm
        from matplotlib import colors
        self.cmap = cm.ScalarMappable(norm=colors.Normalize(vmin=.0,vmax=1.),
                                       cmap=plt.get_cmap("RdBu"))
                                       #cmap=plt.get_cmap("Oranges"))

    def lk_callback(self,kernel_state):
        start = rospy.Time.now()
        n = len(kernel_state.ids)
        K = np.ones([n,n],dtype=np.float32)
        print("start shape: {0}".format(K.shape))
        it = 0
        for i in range(n):
            for j in range(i+1,n):
                K[i,j] = kernel_state.data[it]
                K[j,i] = kernel_state.data[it]
                it += 1

        from sklearn.mixture import GMM
        gmm = GMM(n_components=self.n_cluster, covariance_type='full')
        gmm.fit(K)
        y = gmm.predict(K)
        #from sklearn.cluster import SpectralClustering
        #spec = SpectralClustering(n_clusters=8, affinity='precomputed')
        #spec.fit(K)
        #y = spec.labels_

        print("Clustering took {0} sec".format( (rospy.Time.now() - start).to_sec() ))
        self.publish_object_ids(kernel_state,y)
        self.publish_kernel_image(K,y)

    def publish_object_ids(self, kernel_state, y):
        msg = ObjectIds()
        idx = np.argsort(y)
        msg.header = kernel_state.header
        msg.ids = np.array(kernel_state.ids)[idx]
        msg.offsets = [ 0 ]
        for i in range(1,len(y)):
            if (y[idx[i]] != y[idx[i-1]]):
                msg.offsets.append(i)
        msg.offsets.append(len(y))
        self.pub_objs.publish(msg)

    def publish_kernel_image(self, K, y):
        #idx = np.argsort(y)
        #Ksorted = K[idx,:]
        Ksorted = K#Ksorted[:,idx]
        img = np.array(self.cmap.to_rgba(Ksorted)[:,:,:3]*255, dtype=np.uint8)
        img_scaled = cv2.resize(img,(640,640),interpolation=cv2.INTER_NEAREST)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_scaled,'rgb8'))
                                      

    def reconfigure(self, config, level):
        self.n_cluster = config["n_cluster"]
        print("reconfigure n_cluster={}".format(self.n_cluster))
        return config

if __name__ == '__main__':
    import sys
    print ("Python: " + sys.version)
    rospy.init_node('lk_clustering_node')
    node = ClusteringNode()
    rospy.spin()
