#!/usr/bin/env python3
import rospy
import sys, copy

from std_msgs.msg import Empty

import tf_conversions, tf2_ros, tf2_msgs.msg
from tf.transformations import *
from tf import TransformListener

import moveit_commander, moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Grasp

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

from path_planner.srv import *

class Planner():
  def __init__(self):
    self.posXarm = [-0.000001, 0.200001, 1.021000]

    #Moveit interface initialization
    moveit_commander.roscpp_initialize(sys.argv)

    #Commander and scene definition
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.arm_move_group = moveit_commander.MoveGroupCommander("xarm6")
    self.gripper_move_group = moveit_commander.MoveGroupCommander("xarm_gripper")

    #Publisher definition for trajectory
    self.display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size = 20, latch=True
    )
    # We can get the name of the reference frame for this robot
    self.planning_frame = self.arm_move_group.get_planning_frame()
    print("Planning frame: %s" % self.planning_frame)
    sys.stdout.flush()
    # We can also print the name of the end-effector link for this group
    self.eef_link = self.arm_move_group.get_end_effector_link()
    print("End effector link: %s" % self.eef_link)
    sys.stdout.flush()
    # We can get a list of all the groups in the robot
    self.group_names = self.robot.get_group_names()
    print("Available Planning Groups:", self.group_names)
    sys.stdout.flush()
    # print("Robot state:", xarm.get_current_state())
    #sys.stdout.flush()
    self.attachService = rospy.ServiceProxy("AttachObject", AttachObject)


  #Whenever we change something in moveit we need to make sure that the interface has been updated properly
  def wait_for_state_update(self, box_name, box_is_attached = False, timeout = 0.5):
    start = rospy.get_time()
    current = rospy.get_time()
    box_is_known = not box_is_attached
    while (current - start) < timeout and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in self.scene.get_known_object_names()
      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      current = rospy.get_time()
    # If we exited the while loop without returning then we timed out
    return False


  def publish_trajectory(self, plan):
    # Note: We are just planning, not asking self.arm_move_group to actually move the robot yet
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    self.display_trajectory_publisher.publish(display_trajectory)


  #Code used to move to a given position using move it
  def go_to_pose(self, goal):
    print('Going to:', goal)
    sys.stdout.flush()
    waypoints = []
    wpose = self.arm_move_group.get_current_pose().pose
    #Move to the position of the box 0.25 above on the z-axis
    wpose.position.x = goal[1] - self.posXarm[1]
    wpose.position.y = -goal[0]
    wpose.position.z = 0.25
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = 0.04
    waypoints.append(copy.deepcopy(wpose))

    (plan, _) = self.arm_move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step
      0.0,         # jump_threshold
    )
    self.publish_trajectory(plan)
    self.arm_move_group.execute(plan, wait=True)
    self.arm_move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm_move_group.clear_pose_targets()


  #Returns from its actual state to a "home" position defined by the programmer
  def return_from_pose(self, goal):
    waypoints = []
    wpose = self.arm_move_group.get_current_pose().pose

    wpose.position.z += 0.25
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.0
    wpose.position.x = 0.3
    waypoints.append(copy.deepcopy(wpose))

    (plan, _) = self.arm_move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step
      0.0,         # jump_threshold
    )
    self.publish_trajectory(plan)
    self.arm_move_group.execute(plan, wait=True)
    self.arm_move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm_move_group.clear_pose_targets()


  def change_grip(self, state, box_name):
    joint_goal = [0]*6

    if(state == 'closed'):
      joint_goal = [0.22]*6

      touch_links = self.robot.get_link_names(group = 'xarm_gripper')
      self.scene.attach_box(self.eef_link, box_name, touch_links = touch_links)
      self.attachService(True, box_name)
      self.wait_for_state_update(box_name, True)
    elif(state == 'open'):
      joint_goal = [0.01]*6

      self.scene.remove_attached_object(self.eef_link, box_name)
      self.attachService(False, box_name)
      self.wait_for_state_update(box_name, False)
    
    self.gripper_move_group.go(joint_goal, wait=True)
    self.gripper_move_group.stop()


class myNode():
  def __init__(self):
    rospy.wait_for_service('RequestTask')
    rospy.wait_for_service('AttachObject')


  def get_task(self, action):
    try:
      call_service = rospy.ServiceProxy('RequestTask', RequestTask)
      response = call_service(action)
      #response.model_name, response.position, response.error
      return response
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)
      sys.stdout.flush()

  def main(self):
    #Aplicacion de las funciones desarrolladas
    planner = Planner()

    for _ in range(3):
      pick = self.get_task("pick")
      box_name = pick.model_name
      box_pos = pick.position
      #box_pos = self.tf_lookup(box_name, 'sensor_frame')

      planner.go_to_pose(box_pos)
      planner.change_grip('closed', box_name)
      planner.return_from_pose(box_pos)

      place = self.get_task("place")
      container_name = place.model_name
      container_pos = place.position
      #container_pos = self.tf_lookup(container_name, 'sensor_frame')

      planner.go_to_pose(container_pos)
      planner.change_grip('open', box_name)
      planner.return_from_pose(container_pos)


if __name__ == '__main__':
  rospy.init_node("solution", anonymous=True)
  reset_env_topic = rospy.Publisher(
    'path_planner/environment/reset', Empty,
    queue_size=10, latch=True,
  )
  reset_env_topic.publish()
  rospy.sleep(2)

  while True:
    node = myNode()
    node.main()

    print('Restart solution? (Y/n)')
    sys.stdout.flush()
    res = input()
    if(res != 'n'):
      reset_env_topic.publish()
      rospy.sleep(2)
    else:
      break
  
  rospy.signal_shutdown("Task Completed")