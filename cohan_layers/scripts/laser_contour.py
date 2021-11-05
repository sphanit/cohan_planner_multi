#!/usr/bin/env python
# Brief: Node for converting laser data into contours
# Author: Phani Teja Singamaneni

import rospy
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry


class LaserContour(object):
  def __init__(self):
    rospy.init_node('laser_coutour')
    rospy.Subscriber('base_scan', LaserScan, self.laserCB)
    rospy.Subscriber('base_pose_ground_truth', Odometry, self.odomCB)

    self.x = []
    self.y = []
    self.r_pose = []
    self.r_dx = []
    self.data_counter = 0
    self.saved = False
    self.got_robot_pose = False


    rospy.Timer(rospy.Duration(5.0), self.save_contours)

    # Keep the node alive
    rospy.spin()

  def laserCB(self, scan):
    self.saved = False
    self.x = []
    self.y = []
    if self.got_robot_pose:
      for i in range(0,len(scan.ranges)):
        angle = scan.angle_min + (i * scan.angle_increment)
        self.x[len(self.x):] = [scan.ranges[i]*np.cos(angle) + self.r_pose[0]]
        self.y[len(self.y):] = [scan.ranges[i]*np.sin(angle) + self.r_pose[1]]
      self.saved = True

    # data_x = np.asarray(self.x)
    # data_y = np.asarray(self.y)
    # np.savetxt("x_data"+str(self.data_counter)+".csv", data_x, delimiter=",")
    # np.savetxt("y_data"+str(self.data_counter)+".csv", data_y, delimiter=",")

  def odomCB(self, msg):
    self.got_robot_pose = False
    self.r_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    rot = msg.pose.pose.orientation
    r,p,y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
    self.r_dx = [np.sin(y), np.cos(y)]
    print(self.r_dx)
    self.got_robot_pose = True

  def save_contours(self, event):
    if(self.saved):
      plt.plot(self.r_pose[0],self.r_pose[1],'ro')
      plt.plot(self.r_pose[0]+self.r_dx[0],self.r_pose[1]+self.r_dx[1],'y^')
      plt.plot(self.x, self.y)
      # plt.axis([-5, 14.2, -10, 31.6])
      plt.axis('scaled')
      plt.title("Laser Contour")
      plt.xlabel("x (m)")
      plt.ylabel("y (m)")
      plt.savefig("contour"+str(self.data_counter)+".png")
      self.data_counter += 1
      plt.clf()



if __name__ == '__main__':
  cont = LaserContour()
