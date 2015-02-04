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
        self.sub = rospy.Subscriber("tracking/kernel", KernelState, self.lk_callback,
                                    queue_size=1, buff_size=2**24)
        self.pub_image = rospy.Publisher("tracking/kernel_matrix",Image,queue_size=1)
        self.pub_objs = rospy.Publisher("tracking/objects",ObjectIds,queue_size=1)
        self.pub_probs = rospy.Publisher("tracking/probabilities",Image,queue_size=1)
        self.bridge = CvBridge()
        self.probs = {}
        self.k = 0
        self.gamma = .4
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
                xi = np.ones(self.k)*1./self.k
            else:
                xi = np.zeros(self.k)
                p = self.probs[v]
                xi[:len(p)] = p
                
            X[i,:] = self.transition.dot(xi)
        return X

    def update(self, X, Z, ids):
        res = X*Z
        for i,v in enumerate(ids):
            self.probs[v] = res[i,:]/np.sum(res[i,:])

    def maxmax(self, M):
        idx = [0]*self.k
        for ki in range(self.k):
            i,j = np.unravel_index(np.argmax(M),M.shape)
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
        cov = np.zeros([X.shape[1],self.k])
        for x,z in zip(X,Z):
            cov += np.mat(x).T*np.mat(z)
        idx = self.maxmax(cov)
        return idx

    def lk_callback(self,kernel_state):
        np.set_printoptions(precision=5, suppress=True)
        if kernel_state.header.stamp.to_sec() < self.last_header.stamp.to_sec():
            self.probs = {}
            self.k = 0
        self.last_header = kernel_state.header

        start = rospy.Time.now()
        n = len(kernel_state.ids)
        K = np.ones([n,n],dtype=np.float32)
        print("start shape: %s" % str(K.shape))
        it = 0
        for i in range(n):
            for j in range(i+1,n):
                K[i,j] = kernel_state.data[it]
                K[j,i] = kernel_state.data[it]
                it += 1

        from sklearn.decomposition import PCA
        pca = PCA(n_components=.999)
        Kp = pca.fit(K).transform(K)
        if pca.explained_variance_ratio_[0] < 2./n:
            print("Something is wrong with the kernel matrix. PCA:\n%s"
                  % pca.explained_variance_ratio_)
            return

        for k,v in enumerate(pca.explained_variance_ratio_):
            if v < .008: break

        #k=7
        if k > self.k:
            self.k = k
            self.transition = np.ones([k,k])*(1.-self.gamma)/(k-1.)
            self.transition[np.diag_indices_from(self.transition)] = self.gamma

        print("PCA took %s sec" % (rospy.Time.now() - start).to_sec())
        print("cluster: %s, reduction: %s,%s" % (k,Kp.shape[0],Kp.shape[1]))

        start = rospy.Time.now()
        from sklearn.mixture import GMM

        gmm = GMM(n_components=self.k, covariance_type='full')
        gmm.fit(Kp)

        #Z = np.zeros([n,self.k])
        #Z[:,:k] = gmm.predict_proba(Kp)
        Z = gmm.predict_proba(Kp)
        X = self.predict(kernel_state.ids)
        idx = self.indices(X,Z)
        self.update(X, Z[:,idx], kernel_state.ids)
        ids = list(self.probs.keys())
        y = self.getLabels(ids)

        print("Clustering took %s sec" % (rospy.Time.now() - start).to_sec())
        self.publish_object_ids(ids,y)
        self.publish_kernel_image(Kp,self.getLabels(kernel_state.ids))
        self.publish_probabilities(ids)

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
        Ksorted = K#K[idx,:]
        img = np.array(self.cmap.to_rgba(Ksorted)[:,:,:3]*255, dtype=np.uint8)
        img_scaled = cv2.resize(img,(640,640),interpolation=cv2.INTER_NEAREST)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_scaled,'rgb8'))

    def publish_probabilities(self, ids):
        X = np.zeros([len(ids),self.k])
        for i,v in enumerate(np.sort(ids)):
            p = self.probs[v]
            X[i,:len(p)] = p
        #size = (self.k*32, len(ids)*5)
        size = (256,800)
        img = np.array(self.cmap.to_rgba(X)[:,:,:3]*255, dtype=np.uint8)
        img_scaled = cv2.resize(img,size,interpolation=cv2.INTER_NEAREST)
        self.pub_probs.publish(self.bridge.cv2_to_imgmsg(img_scaled,'rgb8'))

    def reconfigure(self, config, level):
        #self.k = config["n_cluster"]
        print("reconfigure n_cluster={}".format(self.k))
        return config

if __name__ == '__main__':
    import sys
    print ("Python: " + sys.version)
    rospy.init_node('clustering_node')
    node = ClusteringNode()
    rospy.spin()
