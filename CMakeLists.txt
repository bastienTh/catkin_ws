cmake_minimum_required(VERSION 2.8.3)
project(ros4pro)

find_package(
  catkin REQUIRED
  message_generation
  message_runtime
  std_msgs
  sensor_msgs
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   VisionInfer.srv
 )

# Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
   CATKIN_DEPENDS
   std_msgs
   message_generation
   message_runtime
)

include_directories()
