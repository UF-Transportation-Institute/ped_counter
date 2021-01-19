#!/usr/bin/env python

## Pedestrian counter that listens to geometry_msgs::PoseArray published
## to the 'poses' topic

import rospy
from geometry_msgs.msg import PoseArray

def callback(data):
    print(data)

def ped_counter():

    rospy.init_node('ped_counter', anonymous=True)

    rospy.Subscriber('ped_detector/poses', PoseArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    ped_counter()
