#!/usr/bin/env python
import rospy
import os
from ackermann_msgs.msg import AckermannDriveStamped


if __name__ == '__main__':
    rospy.init_node('angle_publisher')
    pub = rospy.Publisher('/drive', Path, latch=True, queue_size=1