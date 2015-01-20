#!/usr/bin/env python

import roslib
roslib.load_manifest('mlr_clustering')
import rospy

from mlr_msgs.msg import KernelState
import numpy as np
from sklearn.mixture import GMM

from collections import defaultdict

id_map = defaultdict(list)
ids = []
timestamps = []
count = 0
kernel = []

def update_id_map(ids,labels):
    global count
    for i in range(len(ids)):
        if id_map.has_key(ids[i]):
            id_map[ids[i]].append(labels[i])
        else:
            id_map[ids[i]] = [-1]*count
    for v in id_map.values():
        if len(v) < count:
            v.append(-1)
    count += 1

def lk_callback(kernel_state):
    start = rospy.Time.now()
    n = len(kernel_state.ids)
    K = np.ones([n,n])
    print("start shape: {0}".format(K.shape))
    it = 0
    for i in range(n):
        for j in range(i+1,n):
            K[i,j] = kernel_state.data[it]
            K[j,i] = kernel_state.data[it]
            it += 1
    gmm = GMM(n_components=7, covariance_type='full')
    gmm.fit(K)
    c_gmm = gmm.predict(K)
    #print(c_gmm)
    timestamps.append(kernel_state.header.stamp.to_sec())
    update_id_map(kernel_state.ids,c_gmm)
    print("end shape: {0}".format(K.shape))
    kernel.append(K)
    ids.append(kernel_state.ids)
    print("Clustering took {0} sec".format( (rospy.Time.now() - start).to_sec() ))

def run():
    rospy.Subscriber("lk_kernel", KernelState, lk_callback, queue_size=1)
    print "Subscribed to lk_kernel"
    rospy.spin()


if __name__ == '__main__':
    import sys
    print ("Python: " + sys.version)
    rospy.init_node('lk_clustering_node')
    run()
    print ("saving labels")
    path = "/home/steffen/git/ipynb/trajectory_clustering/labels/"
    np.save(path+"labels", np.array(id_map.values()))
    np.save(path+"ids", np.array(id_map.keys()))
    np.save(path+"timestamps", np.array(timestamps))
    for i in range(len(kernel)):
        name = path+"kernels/"+str(i).zfill(4)
        print ("saveing kernel "+name)
        np.save(name, np.array(kernel[i]))
    for i in range(len(ids)):
        name = path+"ids/"+str(i).zfill(4)
        print ("saveing ids "+name)
        np.save(name, np.array(ids[i]))
