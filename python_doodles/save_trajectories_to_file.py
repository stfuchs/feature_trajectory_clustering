#!/usr/bin/env python

import roslib
roslib.load_manifest('python_doodles')
import rospy

from mlr_msgs.msg import TrajectoryPointUpdateArray
from mlr_msgs.srv import *

from collections import defaultdict
import numpy as np

id_map = defaultdict(list)
count = 0

def save(prefix):
    for k in id_map.keys():
        t = np.vstack(id_map[k])
        filename = prefix+str(k).zfill(4)
        np.save(filename,t)
        print "Saved Trajectory to " + filename

def callback_msg(data):
    global count
    for p in data.points:
        if id_map.has_key(p.id):
            id_map[p.id].append( np.array([p.point.x,p.point.y,p.point.z,p.header.stamp.to_sec()]) )
        else:
            id_map[p.id] = [np.array([np.nan]*4)]*count
    for v in id_map.values():
        if len(v) < count:
            v.append(np.array([np.nan]*4))
    count += 1

def callback_srv(req):
    print "Service called"
    save(req.prefix)
    return SaveTrajectoryResponse()


# This ends up being the main while loop.
def listener():
    # Get the ~private namespace parameters from command line or launch file.
    #topic = rospy.get_param('~topic', 'chatter')
    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber("lk_points", TrajectoryPointUpdateArray, callback_msg)
    print "Subscribed to lk_points"
    s = rospy.Service('save_lk_points', SaveTrajectory, callback_srv)
    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('save_trajectories_to_file')
    # Go to the main loop.
    listener()
    save("/home/steffen/git/ipynb/trajectory_clustering/labels/traj/")
