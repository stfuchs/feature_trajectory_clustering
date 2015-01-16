#!/usr/bin/env python

import roslib
roslib.load_manifest('mlr_clustering')
import rospy

from mlr_msgs.msg import KernelState
import numpy as np
from sklearn.mixture import GMM

def lk_callback(kernel_state):
    start = rospy.Time.now()
    n = len(kernel_state.ids)
    K = np.ones([n,n])
    it = 0
    for i in range(n):
        for j in range(i+1,n):
            K[i,j] = kernel_state.data[it]
            K[j,i] = kernel_state.data[it]
            it += 1
    gmm = GMM(n_components=6, covariance_type='full')
    gmm.fit(K)
    c_gmm = gmm.predict(K)
    print(c_gmm)
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
