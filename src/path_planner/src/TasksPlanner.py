#!/usr/bin/env python3
import sys

import rospy
import tf2_ros
import numpy as np
import random

from geometry_msgs.msg import PoseStamped, TransformStamped
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from path_planner.srv import RequestGoal, RequestGoalResponse
from path_planner.srv import AttachObject, AttachObjectResponse


class TasksPlanner():
  def __init__(self):
    # We set up the names of each of the target frames that we want to pick or place 
    rospy.init_node('TaskPlanner')

    self.tasks = [
    {
      "type": "pick_place",
      "source": "RedBox",
      "destination": "DepositBoxRed"
    },
    {
      "type": "pick_place",
      "source": "BlueBox",
      "destination": "DepositBoxBlue"
    },
    {
      "type": "pick_place",
      "source": "GreenBox",
      "destination": "DepositBoxGreen"
    },
    ]

    self.task_idx = 0

    # Transform handlers
    self.br = tf2_ros.TransformBroadcaster()

    # Service publisher
    rospy.Service('RequestGoal', RequestGoal, self.send_goal)

    # We want to wait until Gazebo services are up before continuing with the code
    rospy.wait_for_service('/gazebo/set_link_state')

    rospy.Service('AttachObject', AttachObject, self.attach_object)
    self.attached_object = ''
    
    # Add tf2 handlers
    self.tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(self.tfBuffer)

    rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_model_states_cb)

    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "xarm_gripper_base_link"
    tf.child_frame_id = "link_attach"
    # Distance to be tuned manually so that collisions are prevented
    # calibrate to improve the visuals
    tf.transform.translation.z = 0.14 
    tf.transform.rotation.x = 0.717
    tf.transform.rotation.y = 0.697

    brStatic = tf2_ros.StaticTransformBroadcaster()
    brStatic.sendTransform(tf)


  def attach_object(self, data):
    rospy.wait_for_service('/link_attacher_node/attach')
    rospy.wait_for_service('/link_attacher_node/detach')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    #data.action, data.frame (box_name)
    req = AttachRequest()
    req.model_name_1 = "xarm6"
    req.link_name_1 = "link6"
    req.model_name_2 = data.frame
    req.link_name_2 = "link"
    if(data.action):
      self.attached_object = data.frame
      attach_srv.call(req)
    else:
      self.attached_object = ""
      detach_srv.call(req)

    return AttachObjectResponse(True)


  #Not totally reliable (nonzero delay) but reliable enough
  #Ref (known issue): https://github.com/ros-simulation/gazebo_ros_pkgs/issues/535
  def gazebo_model_states_cb(self, data):
    # We update tf2 frames so that they have the same postiion as in Gazebo,
    # if the object is attached we will keep the same position relative to the ee.
    objects = [
      'RedBox', 'GreenBox', 'BlueBox',
      'DepositBoxRed', 'DepositBoxGreen', 'DepositBoxBlue',
    ]
    for obj in set(data.name).intersection(objects):
      if obj == self.attached_object:
        tf = self.tfBuffer.lookup_transform(
          "sensor_frame",
          "link_attach",
          rospy.Time(),
          rospy.Duration(0.5)
        )
        tf.child_frame_id = obj
        #tf.header.stamp = rospy.Time.now()
        self.br.sendTransform(tf)
      else:
        obj_idx = data.name.index(obj)
        obj_pose = data.pose[obj_idx].position
        obj_orientation = data.pose[obj_idx].orientation

        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "sensor_frame"
        tf.child_frame_id = obj
        tf.transform.translation = obj_pose
        tf.transform.rotation = obj_orientation
        self.br.sendTransform(tf)


  def send_goal(self, req):
    # Proposed state machine to control the goals (currently 1)
    action = req.action
    ALLOWED_ACTIONS = ['pick', 'place']
    if(action not in ALLOWED_ACTIONS):
      return RequestGoalResponse('Invalid action', False)

    if(action == 'pick'):
      goal = self.tasks[self.task_idx]['source']
    elif(action == 'place'):
      goal = self.tasks[self.task_idx]['destination']
      self.task_idx += 1
      #reset index
      self.task_idx = self.task_idx % len(self.tasks)
    
    return RequestGoalResponse(goal, True)


if __name__ == '__main__':
  node = TasksPlanner()
  print('Ready to go!')
  sys.stdout.flush()
  rospy.spin()