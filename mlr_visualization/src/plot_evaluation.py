#!/usr/bin/env python

import rospy
from mlr_msgs.msg import ObjectIds
from std_msgs.msg import Float64

import numpy as np
import yaml
from itertools import product

class EvalNode(object):
    def __init__(self, filename=""):
        self.ok = True
        if filename is "":
            self.sub_truth = rospy.Subscriber("tracking/objects_truth",ObjectIds,
                                              self.truth_cb,queue_size=1)
        else:
            try:
                d = yaml.load(open(filename,'r'))
                self.C = list(map(set,d.values()))
            except:
                print("Could not load %s"%filename)
                self.ok = False
                
        self.sub = rospy.Subscriber("tracking/objects",ObjectIds,
                                    self.object_cb,queue_size=1)
        self.pub_pc = rospy.Publisher("tracking/result/pc",Float64,queue_size=1)
        self.pub_po = rospy.Publisher("tracking/result/po",Float64,queue_size=1)
        self.pub_mi = rospy.Publisher("tracking/result/mi",Float64,queue_size=1)


    def truth_cb(self,msg):
        self.C = list()
        for s,e in zip(msg.offsets[:-1], msg.offsets[1:]):
            self.C.append( set(map(lambda i: i>>32, msg.ids[s:e])) )

    def object_cb(self,msg):
        O = list() # predicted sets
        N = list(map(lambda i: i>>32, msg.ids))
        for s,e in zip(msg.offsets[:-1], msg.offsets[1:]):
            O.append( set(N[s:e]) )
            
        N = set(N)  # set of all currently tracked ids
        ninv = 1./len(N)
        C = list(map(lambda c: c & N, self.C)) # expected sets
        #print(C)
        #print(O)
        intersect = lambda s: len(s[0] & s[1])
        pco= np.array(list(map(intersect,product(C,O))),dtype=float).reshape(len(C),len(O))
        pc = np.array(list(map(len,C)),dtype=float)
        po = np.array(list(map(len,O)),dtype=float)
        print("P_co:\n%s"%pco)
        print("P_c: %s"%pc)
        print("P_o: %s"%po)
        print("N: %s"%len(N))
        pc *= ninv
        po *= ninv
        # inter cluster disimilarity / purity over all C
        #purityc = np.sum(pco.max(axis=1))*ninv
        # inner cluster similarity / purity over all O
        purityo = np.sum(pco.max(axis=0))*ninv
        pco *= ninv
        # entropy
        Hc = - np.sum(pc*np.nan_to_num(np.log(pc)))
        Ho = - np.sum(po*np.nan_to_num(np.log(po)))
        # mutal information:
        pcpo = np.array(np.mat(pc).T*np.mat(po))
        mi = np.sum(pco*np.nan_to_num(np.log(pco/pcpo)))
        nmi = 2.*mi / (Hc+Ho)
        print("%s, %s, %s"%(Hc,Ho,mi))
        #self.pub_pc.publish(Float64(purityc))
        self.pub_po.publish(Float64(purityo))
        self.pub_mi.publish(Float64(nmi))

if __name__=='__main__':
    import sys
    if len(sys.argv) > 1:
        fname = sys.argv[1]
        print("ground truth from file %s"%fname)
    else:
        print("ground truth from topic /tracking/objects_truth")
        fname = ""
        
    rospy.init_node('evaluate_node')
    node = EvalNode(fname)
    if node.ok:
        rospy.spin()


