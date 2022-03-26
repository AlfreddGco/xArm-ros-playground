#!/usr/bin/env python3
import rospy
import sys, copy

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
    rospy.init_node("moveit_py", anonymous=True)

    #Commander and scene definition
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.arm_move_group = moveit_commander.MoveGroupCommander("xarm6")
    self.gripper_move_group = moveit_commander.MoveGroupCommander("xarm_gripper")

    #Publisher definition for trajectory
    self.display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size = 20,
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
    wpose = self.arm_move_group.get_current_pose().pose
    #Move to the position of the box 0.25 above on the z-axis
    wpose.position.x = float(goal[1]) - self.posXarm[1]
    wpose.position.y = -float(goal[0])
    wpose.position.z = float(goal[2]) - self.posXarm[2] - 0.02 + 0.25
    waypoints = [copy.deepcopy(wpose)]
    (plan, fraction) = self.arm_move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step
      0.0,         # jump_threshold
    )
    self.publish_trajectory(plan)
    # Execute the planned trajectory
    self.arm_move_group.execute(plan, wait=True)
    self.arm_move_group.stop()

    #Go to the position of the cube
    wpose.position.z = -0.005
    waypoints = [copy.deepcopy(wpose)]
    (plan, fraction) = self.arm_move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step
      0.0,         # jump_threshold
    )
    self.publish_trajectory(plan)
    # Execute the planned trajectory
    self.arm_move_group.execute(plan, wait = True)
    self.arm_move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm_move_group.clear_pose_targets()


  #Returns from its actual state to a "home" position defined by the programmer
  def return_from_pose(self, goal):
    wpose = self.arm_move_group.get_current_pose().pose
    ##Aqui regresa a otro point
    wpose.position.z = (float(goal[2]) - self.posXarm[2] - 0.02) + 0.25
    waypoints = [copy.deepcopy(wpose)]
    (plan, fraction) = self.arm_move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step
      0.0,         # jump_threshold
    )
    self.publish_trajectory(plan)
    # Execute the planned trajectory
    self.arm_move_group.execute(plan, wait=True)
    self.arm_move_group.stop()
    #Home position designed for the robot to not collision himself and  finds a trajectory
    wpose.position.y = 0.0
    wpose.position.x = 0.3
    waypoints = [copy.deepcopy(wpose)]
    (plan, fraction) = self.arm_move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step
      0.0,         # jump_threshold
    )
    self.publish_trajectory(plan)
    # Execute the planned trajectory
    self.arm_move_group.execute(plan, wait=True)
    self.arm_move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm_move_group.clear_pose_targets()


  def change_grip(self, state, box_name):
    attachService = rospy.ServiceProxy("AttachObject", AttachObject)
    joint_goal = [0]*6

    if(state == 'closed'):
      joint_goal = [0.22]*6

      touch_links = self.robot.get_link_names(group = 'xarm_gripper')
      self.scene.attach_box(self.eef_link, box_name, touch_links = touch_links)

      attachService(True, box_name)
      self.wait_for_state_update(box_name, True)
    elif(state == 'open'):
      joint_goal = [0.01]*6

      self.scene.remove_attached_object(self.eef_link, name = box_name)

      attachService(False, box_name)
      self.wait_for_state_update(box_name, False)
    
    self.gripper_move_group.go(joint_goal, wait=True)
    self.gripper_move_group.stop()


class myNode():
  #Initialise ROS and create the service calls
  def __init__(self):
    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')


  def get_goal(self, action):
    #Call the service that will provide you with a suitable target for the movement
    try:
      call_service = rospy.ServiceProxy('RequestGoal', RequestGoal)
      response = call_service(action)
      #print(response.goal)
      #sys.stdout.flush()
      return response.goal
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)
      sys.stdout.flush()


  def tf_goal(self, goal_name, frame_id = 'sensor_frame'):
    #Use tf2 to retrieve the position of the target with respect to the proper reference frame
    #self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)
    transform = self.tf_buffer.lookup_transform(
      frame_id, goal_name,
      rospy.Time(), rospy.Duration(3) #timeout
    )
    target_position = [
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z,
    ]
    print(target_position)
    sys.stdout.flush()
    return target_position


  def main(self):
    #Aplicacion de las funciones desarrolladas
    planner = Planner()

    for _ in range(3):
      box_name = self.get_goal("pick")
      box_pos = self.tf_goal(box_name)
      
      box_pose = PoseStamped()
      moveit_box_pos = self.tf_goal(box_name, 'link_tcp')
      box_pose.header.frame_id = "link_tcp"
      box_pose.pose.position.x = moveit_box_pos[0]
      box_pose.pose.position.y = moveit_box_pos[1]
      box_pose.pose.position.z = moveit_box_pos[2]
      box_pose.pose.orientation.w = 1.0
      planner.scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))

      planner.go_to_pose(box_pos)
      planner.change_grip('closed', box_name)
      planner.return_from_pose(box_pos)

      container_name = self.get_goal("place")
      container_pos = self.tf_goal(container_name)

      planner.go_to_pose(container_pos)
      planner.change_grip('open', box_name)
      planner.return_from_pose(container_pos)

    rospy.signal_shutdown("Task Completed")


if __name__ == '__main__':
  try:
    node = myNode()
    node.main()
  except rospy.ROSInterruptException:
    pass