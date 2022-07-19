# -*- coding: utf-8 -*-
#!/usr/bin/env python
# Brief: GUI for collecting map data
# Author: Phani Teja Singamaneni

import rospy
from geometry_msgs.msg import PointStamped
import PySimpleGUI as sg

class GetPos(object):
  """docstring for GetPos."""
  def __init__(self):
    super(GetPos, self).__init__()
    rospy.init_node('GetData')
    rospy.Subscriber('/clicked_point', PointStamped, self.posCB)
    self.center = [0, 0]
    self.triggered = False

  def posCB(self, msg):
    self.center = [msg.point.x, msg.point.y]
    self.triggered = True



sg.theme('DarkAmber')   # Add a touch of color
# All the stuff inside your window.
layout = [  [sg.Text('Center ='), sg.Text('0', size=(50,1), key='-mytext-')],
            [sg.Text('Enter the radius'), sg.InputText()],
            [sg.Button('Add'), sg.Button('Discard'), sg.Button('Close')] ]

# Create the Window
window = sg.Window('Get Data', layout)
# Event Loop to process "events" and get the "values" of the inputs

get_point = GetPos()

time = 100
while True:
    if get_point.triggered:
      window['-mytext-'].update('Got the Center')
      print(get_point.center)
      event, values = window.read()
      if event == 'Add':
        window['-mytext-'].update('Added the data Point')
        event, values = window.read(1000)
      get_point.triggered = False;
    else:
      event, values = window.read(100)
      window['-mytext-'].update('No center Found!')

    if event == sg.WIN_CLOSED or event == 'Close': # if user closes window or clicks cancel
        break
window.close()
