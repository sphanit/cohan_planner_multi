# CoHAN Planner Multi

The extension of the [CoHAN Planner](https://github.com/sphanit/CoHAN_Planner) that allows the simultaneous running of multiple planners with namespace support. More details about the CoHAN Planner can be found [here](https://github.com/sphanit/CoHAN_Planner/blob/master/README.md).

# Installation (ros-melodic)
1. This installation assumes that the [ROS](http://wiki.ros.org/ROS/Installation) is already installed along with the [2D navigation stack](http://wiki.ros.org/navigation). Otherwise please install them before continuing to next steps.
2. Install the requirements
	```
	sudo apt install python-pip python-catkin-tools
	pip install scipy
	```
3. Clone the git repository
	```
	mkdir -p cohan_multi_ws/src
	cd 	cohan_multi_ws/src
	git clone https://github.com/sphanit/cohan_planner_multi.git -b master
	cd ..
	```
4. Install the dependencies using rosdep
	```
	rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
	```
5. Build
	```
	catkin build
	```
	Note: catkin build might make the cpu busy utilizing all threads if not configured properly, you can instead use it with less jobs. For example, ```catkin build -j4```, can run up to four jobs at a time.

# Usage
1. It gets a bit tricky with the namespaces and remapping. CoHAN Navigation provides some examples and configuration files to launch and run it in STAGE and MORSE simulators.

2. As in the CoHAN Planner, we need to publish the known agents (or humans) on ```/tracked_agents``` topic following message structures in [cohan_msgs](https://github.com/sphanit/cohan_planner_multi/tree/master/cohan_msgs/msg).     
