#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import rospy
import tf
from math import pi
from ros4pro.simulation.gripper import Gripper
from ros4pro.simulation.camera import Camera
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf import TransformBroadcaster


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupInterface(object):
  # ================================================================================
  def __init__(self):
    super(MoveGroupInterface, self).__init__()

    rospy.init_node("manipulate_sawyer")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_names="right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_names)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()

    self. br = tf.TransformBroadcaster()
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  # publish a new tf with a small life expectency
  # Exemple : 
  # self.br_publish("le_distrib","base",0.3,0.3,0.3,0,0,0)
  def br_publish(self,name,basename,x,y,z,qx,qy,qz):
    self.br.sendTransform((x,y,z),
                          tf.transformations.quaternion_from_euler(qx,qy,qz),
                          rospy.Time.now(),
                          name,
                          basename)

  def init_env(self, timeout = 4):
    print("======= ADD Floor")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.z = -0.125
    box_name = "floor"
    self.scene.add_box(box_name, box_pose, size=(100, 100, 0.001))
    print("==== ")
    print("======= ADD PALETTE")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.z = -0.125/2
    box_pose.pose.position.x = -0.09
    box_pose.pose.position.y = 0
    box_name = "palette"
    self.scene.add_box(box_name, box_pose, size=(0.64, 0.8, 0.125))
    print("==== ")
    print("======= ADD DISTRIBUTEUR")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.z =  -0.125 + 0.37/2
    box_pose.pose.position.x = 0.23 - 0.34/2 
    box_pose.pose.position.y = 0.40 + 0.32/2
    box_name = "distributeur"
    self.scene.add_box(box_name, box_pose, size=(0.34, 0.32, 0.37))
    print("==== ")
    print("======= ADD MOTOR")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.z =  -0.125
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = -0.97
    box_name = "motor"
    self.scene.add_box(box_name, box_pose, size=(100, 0.001, 100))
    print("==== ")
    print("======= ADD RIGHT WALL")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.z =  -0.125 + 0.21/2
    box_pose.pose.position.x = 0.23 + 0.55/2
    box_pose.pose.position.y = 0.8/2
    box_name = "right_wall"
    self.scene.add_box(box_name, box_pose, size=(0.55, 0.001, 0.21))
    print("==== ")
    print("======= ADD LEFT WALL")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.w = 1
    box_pose.pose.position.z =  -0.125 + 0.21/2
    box_pose.pose.position.x = 0.23 + 0.55/2
    box_pose.pose.position.y = -0.8/2
    box_name = "left_wall"
    self.scene.add_box(box_name, box_pose, size=(0.55, 0.001, 0.21))
    print("==== ")
    

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
  # ================================================================================
  def add_cube(self):
    print("======= Cube")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.orientation.x = 1
    box_pose.pose.position.z =  0.37-0.125 + 0.025
    box_pose.pose.position.y = 0.52
    box_pose.pose.position.x = 0.23 - 0.34/2
    box_name = "cube"
    self.scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
    print("==== ")


  # ================================================================================
  def attach_cube(self):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "right_gripper_tip"
    box_pose.pose.orientation.x = 1
    box_pose.pose.position.z =  0
    box_pose.pose.position.y = 0
    box_pose.pose.position.x = 0
    box_name = "cube"
    grasping_group = 'right_arm'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, box_name, size=(0.05, 0.05, 0.05),touch_links=touch_links)

  def remove_cube(self):
    self.scene.remove_world_object(name="cube")

  def detach_cube(self):
    grasping_group = 'right_arm'
    box_name = "cube"
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.remove_attached_object(self.eef_link,name="cube")
    # **Note:** The object must be detached before we can remove it from the world
    #return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  # ================================================================================
  def go_to_pose_goal(self, x, y, z, ux, uy, uz, uw):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    pose_goal.orientation.x = ux
    pose_goal.orientation.y = uy
    pose_goal.orientation.z = uz
    pose_goal.orientation.w = uw

    self.move_group.set_pose_target(pose_goal)
    self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    return

  # ================================================================================
  def plan_cartesian_path(self, init_x, init_y, init_z, init_ux, init_uy, init_uz, init_uw, goal_x, goal_y, goal_z, nbpoint = 10):
    waypoints = []

    x_delta = goal_x - init_x
    y_delta = goal_y - init_y
    z_delta = goal_z - init_z


    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = init_x
    wpose.position.y = init_y
    wpose.position.z = init_z
    wpose.orientation.x = init_ux
    wpose.orientation.y = init_uy
    wpose.orientation.z = init_uz
    wpose.orientation.w = init_uw

    k = 1.0/nbpoint

    for i in range(1, nbpoint):
      wpose.position.x += k * x_delta
      wpose.position.y += k * y_delta

      wpose.position.z += k * z_delta
      waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       5)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  # ================================================================================
  def execute_plan(self, plan):
    # Use execute if you would like the robot to follow
    # the plan that has already been computed:
    self.move_group.execute(plan, wait=True)

  # ================================================================================
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # If the Python node dies before publishing a collision object update message, the message
    # could get lost and the box will not appear. To ensure that the updates are
    # made, we wait until we see the changes reflected in the
    # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    # For the purpose of this tutorial, we call this function after adding,
    # removing, attaching or detaching an object in the planning scene. We then wait
    # until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  # ================================================================================
  def attach_box(self, timeout=4):
    # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    # robot be able to touch them without the planning scene reporting the contact as a
    # collision. By adding link names to the ``touch_links`` array, we are telling the
    # planning scene to ignore collisions between those links and the box. For the Panda
    # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    # you should change this value to the name of your end effector group name.
    grasping_group = "right_arm"
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  # ================================================================================
  def do_scenario(self):
    camera = Camera()
    gripper = Gripper()

    # Command gripper open() or close()
    gripper.open()

    # Take a picture.
    # The image will be published on topic /io/internal_camera/right_hand_camera/image_rect. It can be retrieved with a subscriber
    # camera.shoot()

    # move_interface.add_cube()

    move_interface.init_env()
    move_interface.add_cube()    
    move_interface.go_to_pose_goal(0.23 -0.34/2, 0.4 + 0.32/2, -0.125+0.37 + 0.05+0.18, 1, 0, 0, 0)
    self.init_env()    
    self.go_to_pose_goal(0.23 -0.34/2, 0.4 + 0.32/2, -0.125+0.37 + 0.05+0.18, 1, 0, 0, 0)

    plan, fr = self.plan_cartesian_path(0.23 -0.34/2, 0.4 + 0.32/2, -0.125+0.37+ 0.05 + 0.18, 1, 0, 0, 0,
     0.23 - 0.34/2, 0.4 + 0.32/2, -0.125 + 0.37 +0.05, 50)
    self.execute_plan(plan)

    gripper.close()
    #move_interface.remove_cube()
    #rospy.sleep(1)
    move_interface.attach_cube()

    plan, fr = self.plan_cartesian_path(0.23 - 0.34/2, 0.4 + 0.32/2, -0.125 + 0.37 +0.05, 1, 0, 0, 0,
     0.23 -0.34/2, 0.4 + 0.32/2, -0.125+0.37+ 0.05 + 0.18, 50)
    self.execute_plan(plan)

    move_interface.go_to_pose_goal(0.5, 0, 0.1, 0.707, 0.707, 0, 0)
    
    self.go_to_pose_goal(0.5, 0, 0.1, 0.707, 0.707, 0, 0)

    gripper.open()
    move_interface.detach_cube()



# ================================================================================
# ================================================================================
# ================================================================================
def main():
  try:
    move_interface = MoveGroupInterface()

    # move_interface.do_scenario()

    compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionFK)

    ik_header = moveit_commander.MoveGroupCommander("right_arm")

    ik_pose = ik_header.get_random_pose()

    print(compute_ik(ik_header, ik_pose, ["right_arm_tip"]))

    # Tuto
    # move_interface.go_to_joint_state()
    # raw_input()
    # move_interface.go_to_pose_goal()
    # raw_input()

    # cartesian_plan, fraction = move_interface.plan_cartesian_path()
    # raw_input()

    # move_interface.display_trajectory(cartesian_plan)
    # raw_input()

    # move_interface.execute_plan(cartesian_plan)
    # raw_input()

    # move_interface.add_box()
    # raw_input()

    # move_interface.attach_box()
    # raw_input()

    # cartesian_plan, fraction = move_interface.plan_cartesian_path(scale=-1)
    # raw_input()

    # move_interface.execute_plan(cartesian_plan)
    # raw_input()

    # move_interface.detach_box()
    # raw_input()

    # move_interface.remove_box()
    # raw_input()


    # Exit
    rospy.loginfo("Stopped manipulation")  

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
