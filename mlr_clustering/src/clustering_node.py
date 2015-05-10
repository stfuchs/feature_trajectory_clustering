#!/usr/bin/env python

import rospy
from mlr_msgs.msg import KernelState, ObjectIds
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool, Int64MultiArray

import cv2
import numpy as np
from sklearn.mixture import GMM
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib import colors

from cv_bridge import CvBridge
from collections import defaultdict
import itertools

def max_pick(M):
    idx = [0]*M.shape[0]
    for ki in range(M.shape[0]):
        i,j = np.unravel_index(np.argmax(M),M.shape)
        M[i,:] = -1.
        M[:,j] = -1.
        idx[i]=j
    return idx

def remap(X, Z):
    return max_pick( np.sum(map(lambda x,z : np.mat(x).T*np.mat(z), X, Z),axis=0) )

def symmetric(arr, n, diag=1.):
    M = np.zeros([n,n],np.float32)
    M[np.triu_indices(n,1)] = arr
    return M+M.T+np.diag(np.ones(n,dtype=np.float32)*diag)

class DivisiveKMeans:
    def __init__(self, W, v=0.05):
        self.W = W
        self.beta = -np.log(0.5)/(v**2)

    def fit(self, K):
        n = K.shape[0]
        self.l_ = np.zeros(n,int)
        self.lmax_ = itertools.count(1)
        self.recursive(K, range(n))
        self.k_ = self.lmax_.next()
        self.proba_ = np.zeros([n,self.k_])
        self.proba_[range(n),self.l_] = 1.
        return self

    def recursive(self, K, idx):
        A = K[np.ix_(idx,idx)].copy() # get submatrix for all idx

        if A.min() > .4:
            return

        L = np.diag(np.sum(A,axis=0)) - A # graph laplacian
        e,E = np.linalg.eig(L)
        v = (E[:,np.argsort(e)])[:,1] # second eigenvector

        #vidx = np.argsort(v)
        #dv = np.diff(v[vidx])
        #split = v[vidx[np.argmax(dv)]]
        split = 0

        #W = self.W[np.ix_(idx,idx)].copy()
        #D = np.diag(1./np.sum(W,axis=0))
        #V = np.exp(-self.beta*v2*v2)
        #v = V*(D.dot(W).dot(v2)) + (1.-V)*v2

        lnew = -np.ones_like(self.l_)
        lnew[idx] = (v>split).astype(int)
        idx1 = np.where(lnew==0)[0]
        idx2 = np.where(lnew==1)[0]
        self.l_[idx2] = self.lmax_.next()

        if len(idx1) > 1: self.recursive(K, idx1)
        if len(idx2) > 1: self.recursive(K, idx2)

class HierarchicalKMeans:
    def __init__(self, term=1.):
        self.term_ = term

    def fit(self, K):
        n = K.shape[0]
        self.l_ = np.zeros(n,int)
        self.lmax_ = itertools.count(1)
        #self.score_ = []
        self.recursive(K, range(n))
        self.k_ = self.lmax_.next()
        self.proba_ = np.zeros([n,self.k_])
        self.proba_[range(n),self.l_] = 1.
        #self.score_.sort()
        #self.score_ = np.array(self.score_)[:,1]
        return self

    def recursive(self, K, idx):
        aff = K[np.ix_(idx,idx)].flatten()
        low = aff[aff<.4]
        if len(low) == 0: return
        #if len(low) < 0.05*len(aff) and np.mean(low)>0.25: return
        #var = K[np.ix_(idx,idx)].var()
        #aff_min = K[np.ix_(idx,idx)].min()
        #print("Inertia: %s, Variance: %s" % (inertia, var))
        #if aff_min > .3 and var < .1:#self.term_:
        #if var < .1:#self.term_:
            #self.score_.append( (self.l_[idx[0]], var) )
            #return

        lnew = -np.ones_like(self.l_)
        lnew[idx] = KMeans(2).fit_predict(K[np.ix_(idx,idx)])
        idx1 = np.where(lnew==0)[0]
        idx2 = np.where(lnew==1)[0]
        self.l_[idx2] = self.lmax_.next()

        if len(idx1) > 1: self.recursive(K, idx1)
        if len(idx2) > 1: self.recursive(K, idx2)

    def is_good_fit(self, K, idx):
        return KMeans(1).fit(K[np.ix_(idx,idx)]).inertia_/len(idx) < self.term_


class ClusteringNode:
    def __init__(self):
        self.sub_reset = rospy.Subscriber("reset_all", Bool,
                                         self.reset, queue_size=1)
        self.sub = rospy.Subscriber("kernel", KernelState, self.lk_callback,
                                    queue_size=1, buff_size=2**24)
        self.sub_select = rospy.Subscriber("monitor/ids", Int64MultiArray,
                                           self.cb_select, queue_size=1)
        self.pub_image = rospy.Publisher("kernel_matrix",Image,queue_size=1)
        self.pub_objs = rospy.Publisher("objects",ObjectIds,queue_size=1)
        self.pub_probs = rospy.Publisher("probabilities",Image,queue_size=1)
        self.bridge = CvBridge()
        self.reset(True)
        self.gamma = .4

        self.cmap = cm.ScalarMappable(norm=colors.Normalize(vmin=.0,vmax=1.),
                                      cmap=plt.get_cmap("RdBu"))

    def reset(self, msg):
        self.probs = {}
        self.k = 1
        self.k_last = 1
        self.transition = np.ones([1,1])
        self.last_header = Header()
        self.select = []

    def cb_select(self, msg):
        self.select = msg.data

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
        self.probs.update( zip(ids, map(lambda x : x/np.sum(x), X*Z)) )

    def getLabels(self, ids):
        return [ np.argmax(self.probs[i]) for i in ids ]

    def lk_callback(self,kernel_state):
        np.set_printoptions(precision=5, suppress=True)
        if kernel_state.header.stamp.to_sec() < self.last_header.stamp.to_sec():
            self.probs = {}
            self.k = 0
        self.last_header = kernel_state.header

        start = rospy.Time.now()
        n = len(kernel_state.ids)
        K = symmetric(kernel_state.data,n)
        W = symmetric(kernel_state.distances,n,diag=0)
        alpha = -np.log(0.5)/(1.**2)
        W = np.exp(-alpha*W)
        Kp = K

        start = rospy.Time.now()

        #hkm = HierarchicalKMeans(term=1.).fit(K)
        hkm = DivisiveKMeans(W).fit(K)
        self.k_last = hkm.k_

        if self.k_last > self.k:
            self.k = self.k_last
            self.gamma = .99
            self.transition = np.ones([self.k,self.k])*(1.-self.gamma)/(self.k-1.)
            self.transition[np.diag_indices_from(self.transition)] = self.gamma


        Z = np.ones([n,self.k])*.0001
        Z[:,:self.k_last] += hkm.proba_
        Z = np.vstack(map(lambda zi: zi/np.sum(zi), Z))

        X = self.predict(kernel_state.ids)
        self.update(X, Z[:,remap(X,Z)], kernel_state.ids)
        ids = kernel_state.ids #list(self.probs.keys())
        y = self.getLabels(ids)

        print("Clustering took %s sec" % (rospy.Time.now() - start).to_sec())
        self.publish_object_ids(ids,y)
        self.publish_kernel_image(Kp,ids,y,True)
        self.publish_probabilities(ids)

    def publish_object_ids(self, ids, y):
        msg = ObjectIds()
        msg.ids = ids
        idx = np.argsort(y)
        msg.header = self.last_header
        msg.ids = np.array(ids)[idx]
        msg.offsets = [ 0 ]
        msg.labels = [ y[idx[0]] ]
        for i in range(1,len(y)):
            if (y[idx[i]] != y[idx[i-1]]):
                msg.labels.append(y[idx[i]])
                msg.offsets.append(i)
        msg.offsets.append(len(y))
        self.pub_objs.publish(msg)

    def publish_kernel_image(self, K, ids, l, sort=True):
        cmap_kernel = cm.ScalarMappable(norm=colors.Normalize(vmin=0,vmax=1.),
                                        cmap=plt.get_cmap("RdBu"))
        if sort:
            #idx = np.lexsort((ids,l))
            idx = np.argsort(np.array(ids))
            Ksorted = K[idx,:]
            Ksorted = Ksorted[:,idx]
        else:
            Ksorted = K
        img = np.array(self.cmap.to_rgba(Ksorted)[:,:,:3]*255, dtype=np.uint8)
        img_scaled = cv2.resize(img,(640,640),interpolation=cv2.INTER_NEAREST)
        self.highlight_selection(img_scaled, np.array(ids)[idx])
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_scaled,'rgb8'))

    def publish_probabilities(self, ids):
        X = np.zeros([len(ids),self.k])
        for i,v in enumerate(np.sort(ids)):
            p = self.probs[v]
            X[i,:len(p)] = p
        size = (256, 640)
        img = np.array(self.cmap.to_rgba(X)[:,:,:3]*255, dtype=np.uint8)
        img_scaled = cv2.resize(img,size,interpolation=cv2.INTER_NEAREST)
        self.highlight_selection(img_scaled, np.sort(ids))
        self.pub_probs.publish(self.bridge.cv2_to_imgmsg(img_scaled,'rgb8'))

    def highlight_selection(self, img, ids):
        h,w,c = img.shape
        pix_size = float(h)/float(len(ids))
        for i in self.select:
            try:
                idx = np.where(ids==i)[0][0]
                off = idx * pix_size
                p1 = (0,int(off))
                p2 = (w,int(off+pix_size))
                cv2.rectangle(img, p1, p2, ( 46, 204, 113), 2)
            except IndexError:
                print("Selected id %s not in Kernel message" % i)


if __name__ == '__main__':
    import sys
    print ("Python: " + sys.version)
    rospy.init_node('clustering_node')
    node = ClusteringNode()
    rospy.spin()
