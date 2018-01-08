#!/usr/bin/env python

"""
Intent: Read a raw laserscan and publish a modified laserscan.  
"""

import rospy
import copy
from sensor_msgs.msg import LaserScan
from math import pi
from math import degrees
from math import radians

class ModifyLaserscan:
    
    def fixScan(self, inputScan):
        inputScan.angle_min = radians(self.startCount) - pi
        inputScan.angle_max = radians(self.endCount + 1) - pi
        inputScan.ranges = inputScan.ranges[self.startCount:self.endCount]
        inputScan.intensities = inputScan.intensities[self.startCount:self.endCount]
        self.scanPublisher.publish(inputScan); 


    def __init__(self, startAngle = 90, endAngle = 270, isRadians = False):
        # Initialize Node
        rospy.init_node('ModifyLaserscanNode', log_level=rospy.DEBUG)  
        if(isRadians): 
            self.startCount = (degrees(startAngle) + 180) % 360; 
            self.endCount = (degrees(endAngle) + 180) % 360 - 1; 
        else:
            self.startCount = startAngle; 
            self.endCount = endAngle -1; 
            

        
        self.endAngle = endAngle

        # Setup publisher and Subscriber
        self.scanPublisher = rospy.Publisher('/scan', LaserScan, queue_size=None)
        self.inputScan = rospy.Subscriber('/scan_raw', LaserScan, self.fixScan, queue_size=1)
        

        
# This is the program's main function
if __name__ == '__main__':
    
    # Create ModifyLaserScan object
    modifyLaserScan = ModifyLaserscan()
    rospy.spin()