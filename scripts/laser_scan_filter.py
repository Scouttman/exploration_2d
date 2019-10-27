#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import tf

rangeLim = 100

class LaserFilter():
    def __init__(self):
        self.laserOut = rospy.Publisher('/scan', LaserScan,queue_size=10)
    def filter(self,scanIn):
        data = np.array(scanIn.ranges)
        data[data>rangeLim] = rangeLim
        scanIn.ranges = data
        #print max(scanIn.ranges)
        #br = tf.TransformBroadcaster()
        #br.sendTransform((0, 0, 0),
        #                 tf.transformations.quaternion_from_euler(0, 0, 0),
        #                 rospy.Time.now(),
        #                 "laser",
        #                 "base_scan")
        #scanIn.header.stamp = rospy.Time.now()
        self.laserOut.publish(scanIn)

if __name__ == '__main__':
    rospy.init_node('scan_values')
    laserFilter = LaserFilter()
    sub = rospy.Subscriber('/scan_raw', LaserScan, laserFilter.filter)
    rospy.spin()
    exit(0)