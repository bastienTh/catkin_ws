# ------------ INIT ---------------------------------------------
# MASTER
roscore
# bringup (the turtle3 alive)
roslaunch turtlebot3_bringup turtlebot3_robot.launch


# ------------ UTILITIES ---------------------------------------
# Téléop start (using PC keyboard)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# LAUNCH rviz with slam algo (pre configured)
rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
# FRAMES
frames
# Monitoring values (plugin -> topics -> topic monitor)
rqt


# ------------ ROS BAG -------------------------------------------
# record all topics
rosbag record –a
# play a bag
rosbag play nameofbag -r   (-r repeat)


# ------------ MAP RELATED -------------------------------------------
# launch slam algo using gmapping
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
# Save map
rosrun map_server map_saver -f ~/map


# ------------ NAVIGATION RELATED -------------------------------------------------
# launch rviz for navigation separatly
rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_navigation.rviz
# will launch navigation and rviz together
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
# Estimate Initial pose :
    - Click the 2D Pose Estimate button.
    - Click on the approxtimate point in the map where the TurtleBot3 is located and drag the cursor to             indicate the direction where TurtleBot3 faces 
# Navigation goal :
    - Click the 2D Nav Goal button.
    - Click on a specific point in the map to set a goal position and drag the cursor to the direction              where TurtleBot should be facing at the end.
