#!/usr/bin/env python3
import sys

import rospy
import tf2_ros
import numpy as np
import random

from geometry_msgs.msg import PoseStamped, TransformStamped
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from gazebo_msgs.srv import GetModelState

from path_planner.srv import AttachObject, AttachObjectResponse
from path_planner.srv import RequestTask, RequestTaskResponse

import moveit_commander, moveit_msgs.msg

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

    rospy.Service('RequestTask', RequestTask, self.send_task)
    rospy.Service('AttachObject', AttachObject, self.attach_object)
    self.attached_object = ''

    rospy.wait_for_service('/gazebo/get_model_state')
    self.get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


  #Note: this shouldnt be in here
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


  def send_task(self, req):
    action = req.action
    ALLOWED_ACTIONS = ['pick', 'place']
    if(action not in ALLOWED_ACTIONS):
      return RequestTaskResponse('', [-1, -1, -1], 'Invalid action')

    if(action == 'pick'):
      goal = self.tasks[self.task_idx]['source']
    elif(action == 'place'):
      goal = self.tasks[self.task_idx]['destination']
      self.task_idx += 1
      #reset index
      self.task_idx = self.task_idx % len(self.tasks)
    
    state = self.get_model_state_srv.call(goal, '')
    position = [
      state.pose.position.x,
      state.pose.position.y,
      state.pose.position.z,
    ]
    return RequestTaskResponse(goal, position, '')

if __name__ == '__main__':
  node = TasksPlanner()
  print('Ready to go!')
  sys.stdout.flush()
  rospy.spin()