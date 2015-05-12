#!/usr/bin/env python

import rospy
from mlr_msgs.msg import KernelState
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from sklearn.cluster import KMeans
import numpy as np
from pylab import *
import matplotlib.gridspec as gridspec
import itertools

nrows = 2
ncols = 2
fig_width = 10

plot_sym_mat_props = {
    "interpolation": 'none',
    "norm": Normalize(vmin=0,vmax=1.),
    "cmap": get_cmap('RdBu') }

cp = [(231, 76, 60),
      (46, 204, 113),
      (52, 152, 219),
      (241, 196, 15),
      (155, 89, 182),
      (52, 73, 94),
      (230, 126, 34),
      (26, 188, 156),
      (224, 246, 53),
      (149, 165, 166),
      (245, 160, 167),
      (218, 202, 143),
      (141, 177, 115),
      (199, 74, 108),
      (101, 70, 65),
      (54, 99, 120),
      (146, 39, 41)]
html = lambda r,g,b: "#%02x%02x%02x"%(r,g,b)


def fig2img(fig):
    fig.tight_layout()
    fig.canvas.draw()
    w,h = fig.canvas.get_width_height()
    print(w,h)
    #buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    #buf = numpy.roll(buf,3,axis=2)
    buf = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
    buf.shape = (h,w,3)
    buf.reshape(h,w,3)
    #buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    return buf

def symmetric(arr, n, diag=1.):
    M = np.zeros([n,n],np.float32)
    M[np.triu_indices(n,1)] = arr
    return M+M.T+np.diag(np.ones(n,dtype=np.float32)*diag)

class HierarchicalKMeans:
    def __init__(self, B, v=0.01):
        self.B = B
        self.beta = -np.log(0.5)/(v**2)

    def fit(self, K):
        n = K.shape[0]
        self.l_ = np.zeros(n,int)
        self.lmax_ = itertools.count(1)
        self.v_ = []
        self.v2_ = []
        self.v3_ = []
        self.recursive(K, range(n))
        self.k_ = self.lmax_.next()
        return self

    def recursive(self, K, idx):
        A = K[np.ix_(idx,idx)].copy() # get submatrix for all idx
        B = self.B[np.ix_(idx,idx)].copy()
        if A.min() > .4:
            return
        D = np.diag(np.sum(A,axis=0))
        L = D - A # graph laplacian
        e,E = np.linalg.eig(L)
        v = (E[:,np.argsort(e)])[:,1] # second eigenvector
        vidx = argsort(v)

        #B = self.B[np.ix_(idx,idx)].copy()
        #Db = np.diag(1./sum(B,axis=0))
        #w = exp(-self.beta*v*v)
        #v2 = B.dot(v)
        #v2 = (Db.dot(B)).dot(v)

        
        #W = diag(sqrt(exp(-self.beta*v*v)))
        #Aw = array(mat(W)*mat(self.B[ix_(idx,idx)])*mat(W)) + identity(len(v))
        #w = exp(-self.beta*v*v).reshape(len(v),1)
        #W = fmax(w,w.T)
        #Aw = (W*self.B[np.ix_(idx,idx)] + ones_like(W) - W)*A

        #w = exp(-self.beta*v*v).reshape(len(v),1)
        #W = fmax(w,w.T)
        #Aw = (ones_like(W)-W)*A + W*self.B[np.ix_(idx,idx)]
        #Dw = mat(np.diag(np.sum(Aw,axis=0)))
        #Lw = sqrt(Dw).I * (Dw - Aw) * sqrt(Dw).I
        #Lw = Dw - Aw
        #e,E = np.linalg.eig(Lw)
        #v2 = (E[:,np.argsort(e)])[:,1]
        
        #Lsym = np.sqrt(mat(D)).I * mat(L) * np.sqrt(mat(D)).I
        #e,E = np.linalg.eig(array(Lsym))
        #v3 = (E[:,np.argsort(e)])[:,1]

        #e,E = np.linalg.eig(A)
        vidx0 = np.where(abs(v)<0.01)[0] # extract uncertain points
        vidx1 = np.where(abs(v)>=0.01)[0] # extract good points
        vidx11 = np.where(v>=0.01)[0]
        if (len(vidx0) > 0):
            print(len(vidx0),len(vidx1))
            C = mat(B[ix_(vidx1,vidx1)].copy()) # kernel from good points
            c = mat(B[ix_(vidx0,vidx1)].copy()) # relation of uncertain points to good ones
            t = ones_like(vidx1)
            t[v[vidx1]<0] = -1. # indicator for good points
            y = array(c*(C+identity(len(t))*0.0001).I * mat(t).T).T[0]
            print(C.shape)
            print(c.shape)
            print(t.shape)
            print(y)
            v2 = sign(v.copy())
            v2[vidx0] = sign(y)
            self.v2_.append(v2[vidx]*0.1)
        else:
            self.v2_.append(ones_like(v))
            
        self.v_.append(v[vidx])
        self.l_[vidx0] = self.lmax_.next()
        self.l_[vidx11] = self.lmax_.next()
        return

        #self.v3_.append(v3[vidx])
        #dv = diff(v[vidx])
        #split = v[vidx[argmax(dv)]]
        split = 0

        lnew = -np.ones_like(self.l_)
        #lnew[idx] = KMeans(2).fit_predict(A)
        lnew[idx] = (v>split).astype(int)
        idx1 = np.where(lnew==0)[0]
        idx2 = np.where(lnew==1)[0]
        self.l_[idx2] = self.lmax_.next()
        return
        if len(idx1) > 1: self.recursive(K, idx1)
        if len(idx2) > 1: self.recursive(K, idx2)


class PlotNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber("tracking/kernel", KernelState, self.kernel_cb,
                                    queue_size=1, buff_size=2**24)
        self.pub_image = rospy.Publisher("tracking/eigen_info",Image,queue_size=1)
        self.bridge = CvBridge()

    def kernel_cb(self, msg):
        alpha = - log(0.5)/(.8**2) # for spatial distances
        beta = - log(0.5)/(0.05**2) # for indicator function

        n = len(msg.ids)
        K = mat(symmetric(msg.data,n))
        #X = exp(-alpha*mat(symmetric(msg.distances,n,diag=0)))
        X = symmetric(msg.distances,n,diag=0)
        #min_idx = argsort(X,axis=1)
        #x_idx = indices(X.shape)
        #Xnn = zeros_like(X)
        #Xnn[x_idx[0,:,:10],min_idx[:,:10]] = 1.
        X = exp(-alpha*mat(X))
        #l = KMeans(2).fit_predict(K)
        hkm = HierarchicalKMeans(array(X)).fit(array(K))
        l = hkm.l_
        klen = histogram(l,hkm.k_)[0]
        D = mat(diag(sum(array(K),axis=0)))
        I = identity(n)
        #L = array(I - D.I*K)
        L = array(D - K)
        try:
            w,V = linalg.eig(L)
            eidx = argsort(w)
            V = V[:,eidx]
        except:
            print("eigendecomposition failed")
            return


        v1 = (V[:,eidx])[:,1]
        v2 = (V[:,eidx])[:,2]
        v3 = (V[:,eidx])[:,3]
        v4 = (V[:,eidx])[:,4]
        v1idx = argsort(v1)
        y1 = []
        y2 = []
        y3 = []
        #x = v1[v1idx]
        x = range(len(v1idx))
        trK = array(K).trace()
        for vi in v1idx:
            Y = zeros([len(v1),2])
            Y[np.where(v1<=v1[vi])[0],0] = 1
            Y[np.where(v1>v1[vi])[0],1] = 1
            Y[:,0] *= 1./linalg.norm(Y[:,0])
            Y[:,1] *= 1./linalg.norm(Y[:,1])
            print(Y.T.dot(I
            y1.append( trK- (Y.T.dot(array(K)).dot(Y)).trace() )
            y2.append( 2. - (Y.T.dot(array(D.I*K)).dot(Y)).trace() )
            y3.append( (Y.T.dot(array(D - K)).dot(Y)).trace() )

        w = exp(-hkm.beta*v1*v1).reshape(len(v1),1)
        W = fmax(w,w.T)
        Aw = W*hkm.B + ones_like(W) - W
        #W = diag(sqrt(exp(-hkm.beta*v1*v1)))
        #Aw = mat(W)*mat(hkm.B)*mat(W)


        
        fig = figure(1,figsize=(fig_width,fig_width*float(nrows)/float(ncols)))
        gs_top = gridspec.GridSpec(nrows,ncols)
        ax00 = fig.add_subplot(gs_top[0,0])
        ax01 = fig.add_subplot(gs_top[0,1])
        ax10 = fig.add_subplot(gs_top[1,0])
        ax11 = fig.add_subplot(gs_top[1,1])

        didx = lexsort((v1,l))
        Ks = K.copy()[:,didx]
        Ks = Ks[didx,:]
        ax10.imshow(Ks,**plot_sym_mat_props)
        ax10.set_yticks(cumsum(klen)-1)
        ax10.set_yticklabels(klen)
        Xs = X.copy()[:,didx]
        Xs = Xs[didx,:]
        ax11.imshow(Xs,**plot_sym_mat_props)
        ax11.set_yticks(cumsum(klen)-1)
        ax11.set_yticklabels(klen)

        
        #for i in range(hkm.k_):
            #lidx = where(l==i)[0]
            #ax00.plot(v1[lidx],v2[lidx], 'o',c=html(*cp[i]))
            #ax01.plot(v1[lidx],v2[lidx], 'o',c=html(*cp[i]))
        #for v1i,v2i,i in zip(hkm.v_,hkm.v2_,range(len(hkm.v_))):
        if len(hkm.v_) > 0:
            idx = range(len(hkm.v_[0]))
            ax00.plot(idx, hkm.v_[0], 'o-',c=html(*cp[1]))
            ax00.plot(idx, hkm.v2_[0], 'o--',c=html(*cp[0]))
            #ax00.plot(idx, hkm.v3_[0], 'o--',c=html(*cp[2]))
        #if len(hkm.v_) > 1:
        #    idx = range(len(hkm.v_[1]))
        #    ax01.plot(idx, hkm.v_[1], 'o-',c=html(*cp[1]))
        #    ax01.plot(idx, hkm.v2_[1], 'o--',c=html(*cp[0]))
            #ax01.plot(idx, hkm.v3_[1], 'o--',c=html(*cp[2]))
        ax01.plot(x,y1,'x-',c=html(*cp[0]))
        ax01.plot(x,y2,'x-',c=html(*cp[1]))
        ax01.plot(x,y3,'x-',c=html(*cp[2]))

        #ax01.set_xlim([-.1,len(hkm.v_)-1.+.1])
        ax00.set_ylim([-.11,.11])
        ax00.set_yticks([-.1,-.05,-.01,0,.01,.05,.1])
        ax00.grid()
        ax01.grid()
        #ax01.plot(v3[l0],v4[l0], '+',c=html(*cp[1]))
        #ax01.plot(v3[l1],v4[l1], 'x',c=html(*cp[0]))
        #ax01.set_xlim([-.6,.6])
        #ax01.set_ylim([-.6,.6])
        #ax01.grid()
        #k = 11
        #ax2.plot(range(k),w[eidx][:k],'-o',c=html(*cp[0]))
        #ax2.set_ylim([-.01,1.01])
        #ax2.set_xlim([-.1,k+.1])
        #ax2.grid()


        img = fig2img(fig)
        close('all')
        msg = self.bridge.cv2_to_imgmsg(img,'rgb8')
        self.pub_image.publish(msg)
        


if __name__ == '__main__':
    import sys
    rospy.init_node('plot_node')
    node = PlotNode()
    rospy.spin()
