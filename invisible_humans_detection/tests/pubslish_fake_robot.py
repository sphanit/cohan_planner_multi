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
from nav_msgs.msg import MapMetaData
from transformations import quaternion_from_euler
import PySimpleGUI as sg
from map_server.srv import LoadMap
random.seed(3407)

class FakeTFBroadcaster(object):

    def __init__(self, map):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        rospy.Subscriber("map_metadata", MapMetaData, self.update_map_limits)
        with open(map,'r') as file:
          self.map = yaml.safe_load(file)

        self.x_lim = 0
        self.y_lim = 0

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = "map"
        self.t.header.stamp = rospy.Time.now()
        self.t.child_frame_id = "base_footprint"
        self.t.transform.translation.x = 5.5
        self.t.transform.translation.y = 5.5
        self.t.transform.translation.z = 0.0

        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0

    def update_pose(self, center = None, rotate = None):
      if not center and not rotate:
        c, r, theta = self.pick_random_pose()
        self.t.transform.translation.x = c[0]+r*math.cos(theta)
        self.t.transform.translation.y = c[1]+r*math.sin(theta)
      elif center:
        self.t.transform.translation.x = self.x_lim/2
        self.t.transform.translation.y = self.y_lim/2
      # print(c[0]+r*math.cos(theta), c[1]+r*math.sin(theta))

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
      theta = random.uniform(0, 2*math.pi)

      return center, radius, theta

    def update_map_limits(self, msg):
      self.x_lim = msg.width* msg.resolution
      self.y_lim = msg.height* msg.resolution

    def update_pose_within_limits(self):
      self.t.transform.translation.x = random.uniform(0, self.x_lim)
      self.t.transform.translation.y = random.uniform(0, self.y_lim)
      q = quaternion_from_euler(0,0,random.uniform(-math.pi, math.pi))
      self.t.transform.rotation.x = q[0]
      self.t.transform.rotation.y = q[1]
      self.t.transform.rotation.z = q[2]
      self.t.transform.rotation.w = q[3]


def generate_random_map(path):
    idx = random.randint(0,3)
    if idx == 0:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -s 2 -m 2") #Honeycomb
    elif idx == 1:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -s 3 -m 1") #Hexagonal
    elif idx == 2:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -w 5 -h 5 -m 0") #Rectangular
    else:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -s 3 -m 4") #Hexagonal

def update_map(path):
    rospy.wait_for_service('change_map')
    try:
        change_map_ = rospy.ServiceProxy('change_map', LoadMap)
        resp = change_map_(path+"/maze.yaml")
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    sg.theme('DarkAmber')   # Add a touch of color
    # All the stuff inside your window.
    layout = [  [sg.Text('False Positives'), sg.InputText(), sg.Text('Overlapping'), sg.InputText()],
                [sg.Text('Total'), sg.InputText()],
                [sg.Button('AddData'),sg.Text('0', size=(50,1), key='-mytext-')],
                [sg.Button('NextPose'), sg.Button('CenterPose'), sg.Button('Rotate')],
                [sg.Button('NextMap'), sg.Button('Save'), sg.Button('Exit')]
             ]
    # Create the Window
    window = sg.Window('Get Data', layout)

    rospy.init_node('fake_robot_broadcaster')
    path = os.path.abspath(os.path.dirname(__file__))
    tfb = FakeTFBroadcaster(os.path.join(path, '../maps/areas/bremen_kitchen.yaml'))
    map_updated = False
    data = dict()
    data['false_postives'] = []
    data['overlap'] = []
    data['total'] = []
    while not rospy.is_shutdown():
      tfb.t.header.stamp = rospy.Time.now()
      tfm = tf2_msgs.msg.TFMessage([tfb.t])
      tfb.pub_tf.publish(tfm)
      event, values = window.read(50)
      if event == sg.WIN_CLOSED or event == 'Exit': # if user closes window or clicks cancel
          break
      if event == 'AddData':
        data['false_postives'].append(float(values[0]))
        data['overlap'].append(float(values[1]))
        data['total'].append(float(values[2]))
        window['-mytext-'].update(str(len(data['total'])))
      if event == 'NextPose':
        if not map_updated:
          tfb.update_pose()
        else:
          tfb.update_pose_within_limits()

      if event == 'CenterPose':
        tfb.update_pose(center = True)

      if event == 'Rotate':
        tfb.update_pose(rotate = True)

      if event == 'NextMap':
        generate_random_map(path)
        update_map(path)
        map_updated = True

      if event == 'Save':
        with open('data.yaml', 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
