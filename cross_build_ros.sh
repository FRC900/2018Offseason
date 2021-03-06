# Run setup_cross_build.sh first 

# Actual ros build and install :

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential ninja-build
sudo rosdep init
rosdep update
mkdir ~/catkin_arm_cross_ws
cd ~/catkin_arm_cross_ws
rosinstall_generator ros ros_comm robot angles serial robot_localization controller_interface controller_manager combined_robot_hw joint_limits_interface transmission_interface controller_manager controller_interface hardware_interface controller_manager_tests controller_manager_msgs combined_robot_hw combined_robot_hw_tests tf2_tools tf2_eigen tf2_sensor_msgs rosparam_shortcuts rqt_controller_manager actionlib_tutorials image_transport rosbridge_suite --rosdistro kinetic --deps --wet-only > kinetic-ros_comm-wet.rosinstall

#edit kinetic-ros_comm-wet.rosinstall and remove entries for realtime_tools, filter
sed -i -e '/local-name: filters/{N;N;N;d}' kinetic-ros_comm-wet.rosinstall

mkdir -p ~/catkin_arm_cross_ws/src
cd ~/catkin_arm_cross_ws/src

git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
wget https://raw.githubusercontent.com/ros-gbp/urdfdom_headers-release/master/indigo/package.xml
# Fix the version in package.xml to read 1.0.0
sed -i -e 's/:{version}/1.0.0/' package.xml 

cd ~/catkin_arm_cross_ws/src
git clone https://github.com/jbeder/yaml-cpp.git

# Grab urdfdom, add a boilerplate package.xml in it
# so it will build as a ROS package
wget https://github.com/ros/urdfdom/archive/1.0.0.tar.gz
tar -xzvf 1.0.0.tar.gz
rm 1.0.0.tar.gz
mv urdfdom-1.0.0 urdfdom
cd urdfdom
echo ' 
<?xml version="1.0"?>
<package>
  <name>urdfdom</name>
  <version>1.0.0</version>
  <description>URDF DOM</description>
  <maintainer email="a@google.com">Nobody</maintainer>
  <license>BSD</license>
  <buildtool_depend>cmake</buildtool_depend>
  <build_depend>urdfdom_headers</build_depend>

  <export>
  </export>

</package>
' > package.xml

cd ~/catkin_arm_cross_ws/src

touch .rosinstall
wstool merge kinetic-ros_comm-wet.rosinstall
wstool update -j8

# add "<depend>urdfdom_headers</depend>" to src/urdf/urdf_parser_plugin/package.xml
sed -i -e '/<\/package>/i  <depend>urdfdom_headers<\/depend>' urdf/urdf_parser_plugin/package.xml 

# In a docker container : 
# docker run -it --user ubuntu -v /home/kjaget/2018Offseason:/home/ubuntu/2018Offseason -v ~/catkin_arm_cross_ws:/home/ubuntu/catkin_arm_cross_ws  frc900/zebros-beta2019-dev /bin/bash

# Do a fresh build - kill off any lingering dependencies
rm -rf ~/frc2019/roborio/arm-frc2019-linux-gnueabi/opt/ros/kinetic devel_isolated build_isolated

# Note - if this fails looking for gencpp*cmake, run from a new terminal
# window where no ROS setup.bash has previously been sourced
cd ~/catkin_arm_cross_ws
./src/catkin/bin/catkin_make_isolated --install --use-ninja -DCMAKE_INSTALL_PREFIX=$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018Offseason/zebROS_ws/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF

# Add newly built cross-libs to git repo so they are
# used for subsequent Rio imagings
cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi
tar -cjf ~/2018Offseason/roscore_roborio_2018.tar.bz2 opt/ros/kinetic

# I needed to add "-DYAML_CPP_INCLUDE_DIRS=/$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/include
# -DYAML_CPP_LIBRARIES=/$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/libyaml-cpp.a" to
# catkin_make_isolated to get it to find yaml-cpp

