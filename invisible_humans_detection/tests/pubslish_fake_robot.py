#!/usr/bin/env python
# Brief: Publish TF frames for emulating a fake robot
# Author: Phani Teja Singamaneni
import rospy
import tf2_msgs.msg
import geometry_msgs.msg
import random
import math
import yaml
import os
from transformations import quaternion_from_euler
import PySimpleGUI as sg
random.seed(3407)

class FakeTFBroadcaster(object):

    def __init__(self, map):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        with open(map,'r') as file:
          self.map = yaml.safe_load(file)

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = "map"
        self.t.header.stamp = rospy.Time.now()
        self.t.child_frame_id = "base_footprint"
        self.t.transform.translation.x = 2.0
        self.t.transform.translation.y = 3.5
        self.t.transform.translation.z = 0.0

        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0

    def update_pose(self):
      c, r, theta = self.pick_random_pose()
      self.t.transform.translation.x = c[0]+r*math.cos(theta)
      self.t.transform.translation.y = c[1]+r*math.sin(theta)

      print(c[0]+r*math.cos(theta), c[1]+r*math.sin(theta))

      q = quaternion_from_euler(0,0,random.uniform(-math.pi, math.pi))
      # print(q)
      self.t.transform.rotation.x = q[0]
      self.t.transform.rotation.y = q[1]
      self.t.transform.rotation.z = q[2]
      self.t.transform.rotation.w = q[3]

    def pick_random_pose(self):
      n = len(self.map['radii']) # Not radius but diameter
      idx = random.randrange(0,n)
      center = self.map['centers'][idx]
      radius = math.sqrt(random.random())*(self.map['radii'][idx]/2) #Divided by 2 because diameter
      theta = random.uniform(-math.pi, math.pi)

      return center, radius, theta


if __name__ == '__main__':
    sg.theme('DarkAmber')   # Add a touch of color
    # All the stuff inside your window.
    layout = [  [sg.Text('False Positives'), sg.InputText()],
                [sg.Button('Next'), sg.Button('Exit')]
             ]
    # Create the Window
    window = sg.Window('Get Data', layout)

    rospy.init_node('fake_robot_broadcaster')
    path = os.path.abspath(os.path.dirname(__file__))
    tfb = FakeTFBroadcaster(os.path.join(path, '../maps/areas/maze.yaml'))
    while not rospy.is_shutdown():
      tfb.t.header.stamp = rospy.Time.now()
      tfm = tf2_msgs.msg.TFMessage([tfb.t])
      tfb.pub_tf.publish(tfm)
      event, values = window.read(50)
      if event == sg.WIN_CLOSED or event == 'Exit': # if user closes window or clicks cancel
          break
      if event == 'Next':
        tfb.update_pose()
