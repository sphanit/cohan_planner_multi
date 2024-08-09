# ROS1 - ROS2 Communication
This method uses the [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite), [roslibpy](https://roslibpy.readthedocs.io/en/latest/readme.html) and [rospy_message_converter](https://github.com/DFKI-NI/rospy_message_converter) to communicate between ROS1 and ROS2 via web sockets instead of using the ros1_bridge package. For this we are using ROS1 noetic and ROS2 humble (no restriction on selecting only these).

1. Install and build the packages
	--- On ROS1
	```
	sudo apt-get install -y ros-noetic-rosbridge-server
	pip install roslibpy
	mkdir -p ros1_ws/src
	cd ros1_ws/src
	git clone https://github.com/DFKI-NI/rospy_message_converter.git -b master
	cd .. 
	catkin build
	```  
		
	--- On ROS2
	```
	sudo apt-get install -y ros-humble-rosbridge-server
	pip install roslibpy
	mkdir -p ros2_ws/src
	cd ros2_ws/src
	git clone https://github.com/DFKI-NI/rospy_message_converter.git -b humble
	cd .. 
	colcon build
	```

2. Minimal example
	a. Run the bridge on ros2 machine (it can be run on ros1 machine as well, but only one is required)
	```
	# Recommended
	ros2 launch rosbridge_server rosbridge_websocket_launch.xml
	
	# You can use ROS1 version also (NOT NEEDED if the above one is launched) 
	roslaunch rosbridge_server rosbridge_websocket.launch 
	``` 
 	
	b. ROS2 Publisher
	```python
	import time
	import roslibpy
	from rclpy_message_converter import message_converter
	from geometry_msgs.msg import Twist
	
	client = roslibpy.Ros(host='localhost', port=9090)
	client.run()	
	talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

	# Publish the message until killed
	while client.is_connected:
		msg = Twist()
		msg.linear.x = 0.5
		dictionary = message_converter.convert_ros_message_to_dictionary(msg)
		talker.publish(roslibpy.Message(dictionary))
		print('Sending message...')
		time.sleep(0.1)
	
	# Stop publishing and kill the client
	talker.unadvertise()
	client.terminate()
	```
	c. ROS1 Subscriber
	```python
	import roslibpy
	from rospy_message_converter import message_converter
	from geometry_msgs.msg import Twist

 	# Callback function
	def print_msg(msg):
		message = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Twist', msg)
		print(message)

	# Enter the HOST IP Address to connect to ROS2 host
	client = roslibpy.Ros(host=<HOST_IP>, port=9090)
	client.run()
	listener = roslibpy.Topic(client,'/cmd_vel','geometry_msgs/Twist')
	listener.subscribe(print_msg) # print_msg is the callback function for subscriber

	# Looping
	try:	
		while True:	
			pass
	except KeyboardInterrupt:
		client.terminate()
	```
 
3. You should be able to see this on your terminals
   
   On Ubuntu 22.04
   ![](https://github.com/sphanit/cohan_planner_multi/blob/noetic-devel/bridge_and_talk.png).

   On Ubuntu 20.04
