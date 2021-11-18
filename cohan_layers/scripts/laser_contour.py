#!/usr/bin/env python
# Brief: Node for converting laser data into contours
# Author: Phani Teja Singamaneni

import rospy
import sys
import numpy as np
import tf2_ros
import math
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, OccupancyGrid
from utils import worldTomap, getIndex


class LaserContour(object):
  def __init__(self):
    rospy.init_node('laser_coutour')

    self.x = []
    self.y = []
    self.x_vis = []
    self.y_vis = []
    self.r_pose = []
    self.r_dx = []
    self.corners = [[],[]]
    self.data_counter = 0
    self.saved = False
    self.got_robot_pose = False
    self.drawing = False
    self.info = []
    self.map = []

    rospy.Subscriber('base_scan_filtered', LaserScan, self.laserCB)
    rospy.Subscriber('base_pose_ground_truth', Odometry, self.odomCB)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.mapCB)

    #Intialize tf2 transform listener
    self.tf = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf)

    rospy.Timer(rospy.Duration(0.01), self.save_contours)

    plt.ion()

    # Keep the node alive
    rospy.spin()

  def laserCB(self, scan):
    self.saved = False
    prev_point = [0.0,0.0]
    if self.got_robot_pose and self.drawing:
      self.x = []
      self.y = []
      self.x_vis = []
      self.y_vis = []
      self.corners = [[],[]]
      for i in range(0,len(scan.ranges)):
        angle = scan.angle_min + (i * scan.angle_increment)
        if abs(angle) < 1.2:
          self.x_vis[len(self.x_vis):] = [scan.ranges[i]*np.cos(angle)]
          self.y_vis[len(self.y_vis):] = [scan.ranges[i]*np.sin(angle)]
          idx = len(self.x_vis) - 1
          if(idx>0):
            dist = np.linalg.norm([prev_point[0]-self.x_vis[idx], prev_point[1]-self.y_vis[idx]])
            check = (abs(prev_point[0]-self.x_vis[idx]) >= 0.5 or abs(prev_point[1]-self.y_vis[idx]) >= 0.5)
            under_rad = min(np.linalg.norm([self.x_vis[idx], self.y_vis[idx]]), np.linalg.norm([prev_point[0], prev_point[1]]))
            if(dist>0.15 and under_rad<=5.0 and check):
              if(np.linalg.norm([prev_point[0], prev_point[1]]) < np.linalg.norm([self.x_vis[idx], self.y_vis[idx]])):
                self.corners[0][len(self.corners[0]):] = [prev_point[0]]
                self.corners[1][len(self.corners[1]):] = [prev_point[1]]
              else:
                self.corners[0][len(self.corners[0]):] = [self.x_vis[idx]]
                self.corners[1][len(self.corners[1]):] = [self.y_vis[idx]]
          prev_point = [self.x_vis[idx], self.y_vis[idx]]

        self.x[len(self.x):] = [scan.ranges[i]*np.cos(angle)]
        self.y[len(self.y):] = [scan.ranges[i]*np.sin(angle)]

        if len(self.map) > 1 and self.got_robot_pose:
          value = scan.ranges[i]
          if math.isnan(value):
            value = 7.0
          mx, my = worldTomap(value*np.cos(angle), value*np.sin(angle), self.info, self.r_pose)
          m_idx = getIndex(mx, my, self.info)
          print(self.map[m_idx])

      self.saved = True
    # data_x = np.asarray(self.x)
    # data_y = np.asarray(self.y)
    # np.savetxt("x_data"+str(self.data_counter)+".csv", data_x, delimiter=",")
    # np.savetxt("y_data"+str(self.data_counter)+".csv", data_y, delimiter=",")


  # The robot position is same irrespective of the map position as the laser is with respect to laser frame
  # Hence you can remove this function and use the fixed cordinates for the robot
  #   position:
  #   x: -0.275
  #   y: -0.055
  #   z: -0.1
  # orientation:
  #   x: 0.0
  #   y: 0.0
  #   z: 0.0
  #   w: 1.0

  def odomCB(self, msg):
    self.got_robot_pose = False
    try:
      laser_transform = self.tf.lookup_transform("base_laser_link", "map", rospy.Time(),rospy.Duration(0.0001))
      p1 = tf2_geometry_msgs.do_transform_pose(msg.pose, laser_transform)
      self.r_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
      print(p1)
      rot = p1.pose.orientation
      r,p,y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
      self.r_dx = [np.cos(y), np.sin(y)]
      self.got_robot_pose = True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass

  def mapCB(self, map):
    self.info = map.info
    self.map = map.data

  def save_contours(self, event):
    self.drawing = True
    # print(self.corners)
    if(self.saved and len(self.x)>0):
      if (len(self.x) == len(self.y)) and (len(self.corners[0])==len(self.corners[1])):
        plt.plot(self.x, self.y)
        plt.plot(self.x_vis, self.y_vis, 'r')
        plt.plot(self.corners[0],self.corners[1], 'yo')
        # plt.arrow(self.r_pose[0],self.r_pose[1],self.r_dx[0],self.r_dx[1], length_includes_head=True,
            # head_width=0.2, head_length=0.2)
        # plt.plot(self.r_pose[0],self.r_pose[1],'ro')
        # plt.axis('scaled')
        plt.title("Laser Contour")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis([-7.5, 7.5, -7.5, 7.5])
        plt.draw()
        plt.pause(0.001)
        # plt.savefig("contour"+str(self.data_counter)+".png")
        # self.data_counter += 1
        plt.clf()
      self.drawing = False



if __name__ == '__main__':
  cont = LaserContour()
