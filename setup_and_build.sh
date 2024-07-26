# Install the dependencies
sudo apt install python3-pip python3-catkin-tools ros-noetic-navigation
pip3 install scipy

# Clone the config and launch files
git clone https://github.com/sphanit/CoHAN_Navigation.git -b euROBIN_Bremen

# Install the ros based dependencies
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y

# Build the packages
catkin build

