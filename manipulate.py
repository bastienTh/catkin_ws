#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from ros4pro.simulation.gripper import Gripper
from ros4pro.simulation.camera import Camera
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




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

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def init_env(self):
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
        box_pose.pose.position.x = 0.09
        box_pose.pose.position.y = 0
        box_name = "palette"
        self.scene.add_box(box_name, box_pose, size=(0.64, 0.8, 0.125))
        print("==== ")
        print("======= ADD DISTRIBUTEUR")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z =  -0.125 + 0.47/2
        box_pose.pose.position.x = -0.23 + 0.34/2 
        box_pose.pose.position.y = -0.40 - 0.32/2
        box_name = "distributeur"
        self.scene.add_box(box_name, box_pose, size=(0.34, 0.32, 0.47))
        print("==== ")
        print("======= ADD MOTOR")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z =  -0.125
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0.97
        box_name = "motor"
        self.scene.add_box(box_name, box_pose, size=(100, 0.001, 100))
        print("==== ")
        print("======= ADD RIGHT WALL")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z =  -0.125 + 0.21/2
        box_pose.pose.position.x = -0.23 - 0.55/2
        box_pose.pose.position.y = -0.8/2
        box_name = "right_wall"
        self.scene.add_box(box_name, box_pose, size=(0.55, 0.001, 0.21))
        print("==== ")
        print("======= ADD LEFT WALL")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z =  -0.125 + 0.21/2
        box_pose.pose.position.x = -0.23 - 0.55/2
        box_pose.pose.position.y = 0.8/2
        box_name = "left_wall"
        self.scene.add_box(box_name, box_pose, size=(0.55, 0.001, 0.21))
        print("==== ")

    def add_cube(self):
        print("======= CUBE")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z =  0.32
        box_pose.pose.position.x = 0.52
        box_pose.pose.position.y = 0.32        
        box_name = "cube"
        self.scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
        grasping_group = 'right_gripper_tip'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link,box_name,box_pose,size=(0.05, 0.05, 0.05), touch_links=touch_links)
        print("==== ")
    # ================================================================================
    def go_to_joint_state(self):
        joint_goal = [0,0,0,0,0,0,0]
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return

    # ================================================================================
    def go_to_pose_goal(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return

    # ================================================================================
    def plan_cartesian_path(self, scale=1):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    # ================================================================================
    def display_trajectory(self, plan):
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

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
    def add_box(self, timeout=4):
        # First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "right_gripper_tip"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07 # slightly above the end effector
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(0.05, 0.05, 0.05))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    # ================================================================================
    def attach_box(self, timeout=4):
        # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. For the Panda
        # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        # you should change this value to the name of your end effector group name.
        grasping_group = "right_gripper_tip"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    # ================================================================================
    def detach_box(self, timeout=4):
        # Detaching Objects from the Robot
        # We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    # ================================================================================
    def remove_box(self, timeout=4):
        # We can remove the box from the world.
        self.scene.remove_world_object(self.box_name)
        # **Note:** The object must be detached before we can remove it from the world
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)



# ================================================================================
# ================================================================================
# ================================================================================
def main():
  try:
    move_interface = MoveGroupInterface()
    camera = Camera()
    gripper = Gripper()

    # Command gripper open() or close()
    gripper.open()

    # Take a picture.
    # The image will be published on topic /io/internal_camera/right_hand_camera/image_rect. It can be retrieved with a subscriber
    camera.shoot()

    move_interface.init_env()    
    move_interface.add_cube()

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
