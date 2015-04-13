#!/usr/bin/env python

import rospy
import numpy as np
from mlr_msgs.msg import KernelState

def symmetric(arr, n):
    M = np.zeros([n,n],np.float32)
    M[np.triu_indices(n,1)] = arr
    return M+M.T+np.diag(np.ones(n,dtype=np.float32))


class dumper(object):
    def __init__(self,interval,out):
        self.inter = interval
        self.last = rospy.Time.now()
        self.sub = rospy.Subscriber("/tracking/kernel",KernelState,self.listen)
        self.count = 0
        self.out = out

    def listen(self, kernel):
        if (rospy.Time.now() - self.last).to_sec() >= self.inter:
            self.last = rospy.Time.now()
            K = symmetric(kernel.data,len(kernel.ids))
            idx = np.argsort(np.array(kernel.ids))
            K = K[:,idx]
            K = K[idx,:]
            fname = "kernel%05d.npy"%self.count
            self.count += 1
            #fname = "kernel_%s.npy"%rospy.Time.now().to_nsec()
            np.save(self.out+fname, K)
            print("Saved to %s"%fname)

if __name__ == '__main__':
    import sys
    rospy.init_node('dump_kernel')
    inter = 0.5
    if len(sys.argv) > 1:
        inter = float(sys.argv[1])
    if len(sys.argv) > 2:
        oname = sys.argv[2]
    else:
        oname = ""
    node = dumper(inter,oname)
    rospy.spin()
