#!/usr/bin/env python
# Brief: Node for detecting and publishing invisible humans
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse


LEFT = 901
RIGHT = 180
ROBOT_RADIUS = 0.34
SAFETY_OBS = 0.05
RANGE = 1.0
RANGE_MAX = 5.0

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
        left = self.scan_data.ranges[LEFT] - ROBOT_RADIUS - SAFETY_OBS
        right = self.scan_data.ranges[RIGHT] - ROBOT_RADIUS - SAFETY_OBS
        
        if left > RANGE and right > RANGE:
            return TriggerResponse(success=True, message="l" + str(RANGE))
        
        if left > 0 and right > 0:
            if left < right:
                if right < RANGE_MAX:
                    return TriggerResponse(success=True, message="l" + str(left))
                else:
                    return TriggerResponse(success=True, message="r" + str(RANGE))
            else:
                if left < RANGE_MAX:
                    return TriggerResponse(success=True, message="r" + str(right))
                else:
                    return TriggerResponse(success=True, message="l" + str(RANGE))
            
        if left < 0 and right > 0:
            dist = min (right, RANGE)
            return TriggerResponse(success=True, message="r" + str(dist))
                    
        if right < 0 and left > 0:
            dist = min (left, RANGE)
            return TriggerResponse(success=True, message="l" + str(dist))        
        
        return TriggerResponse(success=True, message="")        
    

if __name__ == '__main__':
    docking = DockingCheck()
    
