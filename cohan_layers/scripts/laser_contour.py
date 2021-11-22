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
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, OccupancyGrid
from utils import *


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
    self.rays = [[],[]]
    self.centers = [[],[]]
    self.data_counter = 0
    self.saved = False
    self.got_robot_pose = False
    self.drawing = False
    self.locating = False
    self.info = []
    self.map = []

    rospy.Subscriber('base_scan_filtered', LaserScan, self.laserCB)
    rospy.Subscriber('base_pose_ground_truth', Odometry, self.odomCB)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.mapCB)
    self.pub_invis_human = rospy.Publisher('invisible_humans', MarkerArray, queue_size = 10)

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
    if self.got_robot_pose and (not self.drawing) and (not self.locating):
      self.x = []
      self.y = []
      self.x_vis = []
      self.y_vis = []
      self.corners = [[],[]]
      self.rays = [[],[]]
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
                self.rays[0][len(self.rays[0]):] = [self.x_vis[idx]]
                self.rays[1][len(self.rays[1]):] = [self.y_vis[idx]]
              else:
                self.corners[0][len(self.corners[0]):] = [self.x_vis[idx]]
                self.corners[1][len(self.corners[1]):] = [self.y_vis[idx]]
                self.rays[0][len(self.rays[0]):] = [prev_point[0]]
                self.rays[1][len(self.rays[1]):] = [prev_point[1]]

          prev_point = [self.x_vis[idx], self.y_vis[idx]]

        self.x[len(self.x):] = [scan.ranges[i]*np.cos(angle)]
        self.y[len(self.y):] = [scan.ranges[i]*np.sin(angle)]

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
    # try:
    #   laser_transform = self.tf.lookup_transform("base_laser_link", "map", rospy.Time(),rospy.Duration(0.0001))
    #   p1 = tf2_geometry_msgs.do_transform_pose(msg.pose, laser_transform)
    self.r_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
      # print(p1)
    #   rot = p1.pose.orientation
    #   r,p,y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
    #   self.r_dx = [np.cos(y), np.sin(y)]
    self.got_robot_pose = True
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      # pass

  def mapCB(self, map):
    self.info = map.info
    self.map = map.data

  def save_contours(self, event):
    self.locate_humans()
    # self.drawing = True
    # # print(self.corners)
    # if(self.saved and len(self.x)>0):
    #   if (len(self.x) == len(self.y)) and (len(self.corners[0])==len(self.corners[1])):
    #     plt.plot(self.x, self.y)
    #     plt.plot(self.x_vis, self.y_vis, 'r')
    #     plt.plot(self.corners[0],self.corners[1], 'yo')
    #     plt.plot(self.centers[0],self.centers[1], 'go')
    #     plt.arrow(-0.275,-0.55,1.0,0.0, length_includes_head=True,
    #         head_width=0.2, head_length=0.2)
    #     plt.plot(-0.275,-0.55,'ro')
    #     # plt.axis('scaled')
    #     plt.title("Laser Contour")
    #     plt.xlabel("x (m)")
    #     plt.ylabel("y (m)")
    #     plt.axis([-7.5, 7.5, -7.5, 7.5])
    #     plt.draw()
    #     plt.pause(0.001)
    #     # plt.savefig("contour"+str(self.data_counter)+".png")
    #     # self.data_counter += 1
    #     plt.clf()
    # self.drawing = False

  def locate_humans(self):
    self.locating = True
    self.centers = [[],[]]
    center_points = []
    marker_array = MarkerArray()
    if len(self.map) > 1 and self.got_robot_pose and len(self.corners[0])>0:
      for i in range(0,len(self.corners[0])):
        x = self.corners[0][i]
        y = self.corners[1][i]
        x1 = self.rays[0][i]
        y1 = self.rays[1][i]

        if math.isnan(x):
          x = 7.0
        if math.isnan(y):
          y = 7.0
        if math.isnan(x1):
          x1 = 7.0
        if math.isnan(y1):
          y1 = 7.0

        l_or_r = checkPoint([-0.275,-0.55], [1.0,0.0],[x,y])
        v_mag = np.linalg.norm([x1-x, y1-y])
        ux = (x1-x)/v_mag
        uy = (y1-y)/v_mag
        xt = x
        yt = y
        alp = 0.1
        center = [0,0]
        hum_rad = 0.3
        while(True):
          if(l_or_r == 1):
            pt = getLeftPoint([x,y],[x1,y1],[xt,yt],dist = hum_rad)
          elif(l_or_r == -1):
            pt = getRightPoint([x,y],[x1,y1],[xt,yt],dist = hum_rad)
          l_pt, r_pt = get2Points([xt, yt],pt, radius = hum_rad)
          in_pose_l = PoseStamped()
          in_pose_r = PoseStamped()
          in_pose_l.pose.position.x = l_pt[0]
          in_pose_l.pose.position.y = l_pt[1]
          in_pose_l.pose.orientation.w = 1
          in_pose_r.pose.position.x = r_pt[0]
          in_pose_r.pose.position.y = r_pt[1]
          in_pose_r.pose.orientation.w = 1
          point_transform = self.tf.lookup_transform("odom", "base_laser_link", rospy.Time(),rospy.Duration(0.001))
          p1 = tf2_geometry_msgs.do_transform_pose(in_pose_l, point_transform)
          p2 = tf2_geometry_msgs.do_transform_pose(in_pose_r, point_transform)
          mx_l, my_l = worldTomap(p1.pose.position.x,p1.pose.position.y, self.info)
          mx_r, my_r = worldTomap(p2.pose.position.x,p2.pose.position.y, self.info)
          # add +0.055 to y to get the pose in map frame
          m_idx_l = getIndex(mx_l, my_l, self.info)
          m_idx_r = getIndex(mx_r, my_r, self.info)
          xt = xt + alp*ux
          yt = yt + alp*uy
          if self.map[m_idx_l] == 0 and self.map[m_idx_r] == 0 or abs(xt)>=abs(x1) or abs(yt)>=abs(y1):
            center = [(xt+pt[0])/2, (yt+pt[1])/2]
            break

        self.centers[0][len(self.centers[0]):] = [center[0]]
        self.centers[1][len(self.centers[1]):] = [center[1]]
        p_temp = Point()
        p_temp.x = (p1.pose.position.x + p2.pose.position.x)/2
        p_temp.y = (p1.pose.position.y + p2.pose.position.y)/2 + 0.055
        p_temp.z = 0.6

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = p_temp.x
        marker.pose.position.y = p_temp.y
        marker.pose.position.z = p_temp.z
        t = rospy.Duration(2.0)
        marker.lifetime = t
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 1.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker_array.markers.append(marker)

      self.pub_invis_human.publish(marker_array)

    self.locating = False



if __name__ == '__main__':
  cont = LaserContour()
