#!/usr/bin/env python

from os import listdir
from collections import defaultdict
from numpy import *
from sklearn.cluster import KMeans
from sklearn.mixture import GMM
import matplotlib.pyplot as plt
import matplotlib.cm as pltcm
import matplotlib.colors as pltcolors

class Trajectory:
    def __init__(self, id_):
        self.dist = defaultdict(list)
        self.id_ = id_
        self.t_curr = 0.
        self.t_prev = 0.
        self.td_inv = 0.
        self.data = []

    def update(self, x):
        self.data.append(x[:-1])
        self.t_prev = self.t_curr
        self.t_curr = x[-1]
        if(self.t_prev != 0):
            self.td_inv = 1./(self.t_curr-self.t_prev)

    def update_distance(self, f):
        if (self.t_prev==0):
            return False
        elif (self.t_curr == f.t_curr):
            d = linalg.norm(f.data[-1] - self.data[-1])
        elif (self.t_curr == f.t_prev):
            d = linalg.norm(f.data[-2] - self.data[-1])
        elif (self.t_curr < f.t_prev):
            return False
        else:
            a = (self.t_curr - f.t_curr)*f.td_inv
            d = linalg.norm((a*f.data[-1] + (1.-a)*f.data[-2]) - self.data[-1])
        self.dist[f.id_].append(d)
        return True

def variance(f1, f2):
    d = hstack([f1.dist[f2.id_], f2.dist[f1.id_]])
    return var(d)

def skip(K, i):
    tmp = zeros([K.shape[0]-1, K.shape[1]-1])
    tmp[:i,:i] = K[:i,:i]
    tmp[i:,i:] = K[i+1:,i+1:]
    tmp[:i,i:] = K[:i,i+1:]
    tmp[i:,:i] = K[i+1:,:i]
    return tmp


path = '/home/steffenfuchs/Dropbox/Master/traj/toy_truck/'
fnames = listdir(path)
#fnames.remove('0025.txt') #53
#fnames.remove('0093.txt') #59
#fnames.remove('0081.txt') #38
#fnames.remove('0133.txt') #34

data = []
F = []
timeline = []
for i in range(len(fnames)):
    data.append(loadtxt(path+fnames[i]))
    F.append(Trajectory(i))
    for j in range(len(data[-1])):
        timeline.append( (data[-1][j,-1],i,data[-1][j,:]) )

timeline.sort()
for t in timeline:
    idx = t[1]
    F[idx].update( t[2] )
    if( F[idx].t_prev == 0 ): continue
    for f in F:
        if f.id_ == idx: continue
        f.update_distance(F[idx])

n = len(F)
K = zeros([n,n])
for i in range(n):
    for j in range(i+1,n):
        K[i,j] = exp(-variance(F[i],F[j]))

#K = mat(skip(K+K.T + diag(ones(n)),53))
K = K+K.T + diag(ones(n))
v,e=linalg.eig(K)
idx = argsort(v)
k = 3
A = mat(e[idx][:2])
Kp = (A*K).T

gmm = GMM(n_components=k, covariance_type='full')
gmm.fit(K)
c = gmm.predict(K)
#gmm.weights_
#gmm.means_
#gmm.covars_

#km = KMeans(k)
#km.fit(Kp)
#c = km.predict(Kp)

def plotGMM(samples):
    gmm.fit(samples)
    xmin = samples[:,0].min()
    xmax = samples[:,0].max()
    ymin = samples[:,1].min()
    ymax = samples[:,1].max() 
    x = linspace(xmin-.1*(xmax-xmin), xmax+.1*(xmax-xmin))
    y = linspace(ymin-.1*(ymax-ymin), ymax+.1*(ymax-ymin))
    X, Y = meshgrid(x, y)
    XX = array([X.ravel(), Y.ravel()]).T
    Z = -gmm.score_samples(XX)[0]
    Z = Z.reshape(X.shape)
    #CS = plt.contour(X, Y, Z, norm=pltcolors.LogNorm(vmin=1.0, vmax=1000.0),levels=logspace(0, 3, 10))
    CS = plt.contour(X,Y,Z)
    #CB = plt.colorbar(CS, shrink=0.8, extend='both')
    plt.plot(samples[:,0],samples[:,1], '+')
    #plt.scatter(samples[:, 0], samples[:, 1], .8)
    plt.title('Negative log-likelihood predicted by a GMM')
    plt.axis('tight')
    plt.show()

def myplot():
    fig1 = plt.figure(figsize=(830/80.,800/80.), dpi=80)
    fig2 = plt.figure(figsize=(830/80.,800/80.), dpi=80)
    ax1 = fig1.add_subplot(111)
    ax2 = fig2.add_subplot(111)
    colors = ['r','g','b','c','m','k']
    jet = plt.get_cmap('jet')
    cnorm = pltcolors.Normalize(vmin=0,vmax=1.)
    cmap = pltcm.ScalarMappable(norm=cnorm,cmap=jet)
    for i in range(len(F)):
        tmp = vstack(F[i].data)
        ax1.plot(tmp[:,0],tmp[:,2], '-'+colors[ c[i] ])
        ax2.plot(tmp[:,1],tmp[:,2], '-'+colors[ c[i] ])

    ax1.grid()
    ax2.grid()
    ax1.set_xlabel('x')
    ax1.set_ylabel('z')
    ax2.set_xlabel('y')
    ax2.set_ylabel('z')
    ax1.set_ylim([.6,1.4])
    ax2.set_ylim([.6,1.4])
    plt.show()

#plot()
