# CoHAN Planner Multi (For MultiVerse - euROBIN)

# Installation (ros-noetic)
1. Make sure that [ROS](http://wiki.ros.org/ROS/Installation) noetic is already installed on the system.

2. Make a directory for workspace and clone the git repository
	```
	mkdir -p cohan_multi_ws/src
	cd 	cohan_multi_ws/src
	git clone https://github.com/sphanit/cohan_planner_multi.git -b euROBIN_Bremen
 	cd cohan_planner_multi
	```
3. Install the dependent packages and build
	```
	./setup_and_build.sh
	```
4. Source (need to do it in every terminal - you can add it to .bashrc otherwise) 
	```
 	cd ../..
	source devel/setup.bash
	```
	Make sure you are in cohan_multi_ws before running the above command.
	
# Launching the planner
1. In a new terminal run the following (inside cohan_multi_ws)
	```
	source devel/setup.bash
	roslaunch cohan_navigation tiago_only.launch
	```
	This should run the planner and open RViz. You can now navigate the robot by using the 2D nav arrow.

2. Additionally, the invisible humans detection (anticipates and corrects the planning). For this, run these in a seperate terminal
	```
	source devel/setup.bash
	roslaunch invisible_humans_detection detect_invisible_humans.launch
	```
  
