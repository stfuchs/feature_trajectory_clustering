#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
    rospy.init_node('resetter')
    pub = rospy.Publisher("tracking/reset_all", Bool, queue_size=1)
    msg = Bool()
    msg.data = True
    print("Wait for it...")
    rospy.Rate(2).sleep() # waiting for reconfigure
    pub.publish(msg)
    print("Sent. Kill me.")
    try:
        while not rospy.is_shutdown():
            rospy.Rate(1).sleep() # waiting for reconfigure
    except rospy.ROSInterruptException:
        pass
