#!/usr/bin/env python
# Brief: Node for detecting and publishing invisible humans
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse


LEFT = 901
RIGHT = 180
ROBOT_RADIUS = 0.35
RANGE = 0.7

class DockingCheck(object):
    def __init__(self) -> None:
        rospy.init_node('docking_check_node')
        rospy.Subscriber('/map_scanner/map_scan', LaserScan, self.laser_callback)
        s = rospy.Service('get_docking_spot', Trigger, self.get_docking_spot)
        self.scan_data = None
        rospy.spin()
        

    def laser_callback(self, msg):
        self.scan_data = msg
    
    def get_docking_spot(self, req):
        left = self.scan_data.ranges[LEFT] - ROBOT_RADIUS
        right = self.scan_data.ranges[RIGHT] - ROBOT_RADIUS
        
        if left > RANGE and right > RANGE:
            return TriggerResponse(success=True, message="l" + str(RANGE))
        
        if left > 0 and right > 0:
            if left < right:
                return TriggerResponse(success=True, message="l" + str(left))
            else:
                return TriggerResponse(success=True, message="r" + str(right))
            
        if left < 0 and right > 0:
            return TriggerResponse(success=True, message="r" + str(right))
                    
        if right < 0 and left > 0:
            return TriggerResponse(success=True, message="l" + str(left))        
        
        return TriggerResponse(success=True, message="")        
    

if __name__ == '__main__':
    docking = DockingCheck()
    
