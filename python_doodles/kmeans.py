#!/usr/bin/env python

import numpy as np
from sklearn.cluster import KMeans

def rand_network(N):
    x = np.random.rand(N,N)
    x = np.triu(x) + np.triu(x).T
    return x - np.diag(x.diagonal()) + np.identity(N)

class BinMeans(object):
    def __init__(self, data):
        self.X = data
        self.n = data.shape[0]
        self.init()

    def init(self):
        self.l = np.mat(np.round(np.random.rand(self.n)))
        while not ((self.l == 1).any() and (self.l == 0).any()):
            self.l = np.mat(np.round(np.random.rand(self.n)))            
        self.mstep()
        self.estep()

    def estep(self):
        X0 = np.array(self.X - self.mu0)**2
        X1 = np.array(self.X - self.mu1)**2
        self.d0 = np.sum(np.mat(X0),axis=0)
        self.d1 = np.sum(np.mat(X1),axis=0)
        self.l = (self.d0 > self.d1).astype(np.float)

    def mstep(self):
        denom = np.sum(self.l)
        self.mu1 = np.mat((self.l*self.X)/denom).T
        self.mu0 = np.mat((1.-self.l)*self.X/(self.n-denom)).T

    def fit(self, n=100, n_init = 20):
        vlast = 10**10;
        for i in range(n_init):
            self.init()
            l = self.iter(n)
            v = self.l * self.d1.T + (1.-self.l)*self.d0.T
            if v < vlast:
                vlast = v
                res = l.copy()
        return res

    def iter(self, n=100):
        l = self.l.copy()
        for i in range(n):
            self.mstep()
            self.estep()
            if (l == self.l).all():
                #print("BinMeans iterations: %s"%i)
                break
            else:
                l = self.l.copy()
        return np.array(l)[0,:].astype(int)

class BinPartition(object):
    def __init__(self, data):
        self.X = data
        self.n = data.shape[0]
        idx = np.argsort(np.sort(data)[:,0])
        self.mu = [ data[idx[0]], data[idx[1]] ]
        self.n_mu = [1., 1.]
        self.l = np.zeros(self.n)
        self.l[idx[1]] = 1
        for i in range(2,self.n):
            d0 = (data[idx[i]] - self.mu[0]/self.n_mu[0])
            d1 = (data[idx[i]] - self.mu[1]/self.n_mu[1])
            k = np.argmin([d0.dot(d0),d1.dot(d1)])
            self.l[idx[i]] = k
            self.mu[k] += data[idx[i]]
            self.n_mu[k] += 1.
            print(self.mu[k]/self.n_mu[k])


def Test(n,d=5):
    for i in range(n):
        X = rand_network(d)
        #print(X)
        l_bm = BinMeans(X.copy()).fit()
        l_km = KMeans(2).fit_predict(X.copy())
        print("BinMeans res: %s"%l_bm)
        print("BinPartition: %s"%BinPartition(X.copy()).l.astype(int))
        print("KMeans res:   %s\n"%l_km)
        
