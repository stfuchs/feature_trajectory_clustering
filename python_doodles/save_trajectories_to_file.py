#!/usr/bin/env python

import roslib
roslib.load_manifest('python_doodles')
import rospy

from mlr_msgs.msg import TrajectoryPointUpdateArray
from mlr_msgs.srv import *

from collections import defaultdict
import numpy as np

id2t = defaultdict(list)

def callback_msg(data):
    for p in data.points:
        id2t[p.id].append( np.array([p.point.x,p.point.y,p.point.z,p.header.stamp.to_sec()]) )

def callback_srv(req):
    print "Service called"
    for k in id2t.keys():
        t = np.vstack(id2t[k])
        filename = req.prefix+str(k).zfill(4)+'.txt'
        np.savetxt(filename,t)
        print "Saved Trajectory to " + filename
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
