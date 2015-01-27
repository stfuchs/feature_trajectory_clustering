#!/usr/bin/env python

import rospy
from mlr_msgs.msg import KernelState, ObjectIds
from sensor_msgs.msg import Image
from std_msgs.msg import Header

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
        self.probs = {}
        self.last_header = Header()

        from dynamic_reconfigure.server import Server as DynReconfServer
        from mlr_clustering.cfg import lk_clustering_paramsConfig
        self.server = DynReconfServer(lk_clustering_paramsConfig, self.reconfigure)

        import matplotlib.pyplot as plt
        from matplotlib import cm
        from matplotlib import colors
        self.cmap = cm.ScalarMappable(norm=colors.Normalize(vmin=.0,vmax=1.),
                                       cmap=plt.get_cmap("RdBu"))
                                       #cmap=plt.get_cmap("Oranges"))

    def predict(self, ids):
        X = np.zeros([len(ids),self.k])
        for i,v in enumerate(ids):
            if v not in self.probs:
                p = np.ones(self.k)*1./self.k
            else:
                p = self.probs[v]

            X[i,:len(p)] = self.transition[:len(p),:len(p)].dot(p)
        return X

    def update(self, Z, X, ids):
        X = Z*X
        for i,v in enumerate(ids):
            self.probs[v] = X[i,:]/np.sum(X[i,:])

    def maxmax(self, M):
        idx = [0]*self.k
        for fi in self.f:
            i,j = np.unravel_index(fi(M),M.shape)
            M[i,:] = 0
            M[:,j] = 0
            idx[i]=j
        return idx

    def getLabels(self, ids):
        l = []
        for i in ids:
            l.append(np.argmax(self.probs[i]))
        return l


    def indices(self, X, Z):
        cov = np.zeros([self.k,self.k])
        for x,z in zip(X,Z):
            cov += np.mat(x).T*np.mat(z)
        idx = self.maxmax(cov)
        return idx

    def lk_callback(self,kernel_state):
        if kernel_state.header.stamp.to_sec() < self.last_header.stamp.to_sec():
            self.probs = {}
        self.last_header = kernel_state.header

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
        #y = gmm.predict(K)
        Z = gmm.predict_proba(K)
        X = self.predict(kernel_state.ids)
        idx = self.indices(X,Z)
        self.update(Z[:,idx],X,kernel_state.ids)
        ids = list(self.probs.keys())
        y = self.getLabels(ids)
        #from sklearn.cluster import SpectralClustering
        #spec = SpectralClustering(n_clusters=8, affinity='precomputed')
        #spec.fit(K)
        #y = spec.labels_

        print("Clustering took {0} sec".format( (rospy.Time.now() - start).to_sec() ))
        self.publish_object_ids(ids,y)
        self.publish_kernel_image(K,y)

    def publish_object_ids(self, ids, y):
        msg = ObjectIds()
        msg.ids = ids
        idx = np.argsort(y)
        msg.header = self.last_header
        msg.ids = np.array(ids)[idx]
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
        self.k = config["n_cluster"]
        print("reconfigure n_cluster={}".format(self.k))

        prob = .4
        self.transition = np.ones([self.k,self.k])*(1.-prob)/(self.k-1.)
        self.transition[np.diag_indices_from(self.transition)] = prob
        self.f = [np.argmax]*self.k

        return config

if __name__ == '__main__':
    import sys
    print ("Python: " + sys.version)
    rospy.init_node('lk_clustering_node')
    node = ClusteringNode()
    rospy.spin()
