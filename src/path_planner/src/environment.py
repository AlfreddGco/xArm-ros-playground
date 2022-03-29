#!/usr/bin/env python3
import sys

import rospy

import tf_conversions, tf2_ros, tf2_msgs.msg
from tf.transformations import *
from tf import TransformListener

from gazebo_msgs.msg import ModelState, ModelStates

import moveit_commander, moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Grasp

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

TABLE_HEIGHT = 1.045

def GModel(name, position, orientation = {'x': 0, 'y': 0, 'z': 0, 'w': 1}):
  return {
    'name': name,
    'position': position,
    'orientation': orientation
  }


def GBox(name, position, orientation = {'x': 0, 'y': 0, 'z': 0, 'w': 1}):
  return {
    'name': name,
    'type': 'box',
    'position': position,
    'orientation': orientation,
    'size': (0.06, 0.06, 0.06),
  }


class Environment:
  def __init__(self):
    self.models = [
      GBox('GreenBox', {
        'x': -0.186303,
        'y': 0.411113,
        'z': TABLE_HEIGHT,
      }),
      GBox('RedBox', {
        'x': -0.5163,
        'y': 0.4,
        'z': TABLE_HEIGHT,
      }, {
        'x': 0,
        'y': 0,
        'z': -0.168,
        'w': 1,
      }),
      GBox('BlueBox', {
        'x': -0.560,
        'y': 0.2759,
        'z': TABLE_HEIGHT,
      }),
      GModel('DepositBoxGreen', {
        'x': 0.463934,
        'y': 0.454994,
        'z': TABLE_HEIGHT,
      }),
      GModel('DepositBoxRed', {
        'x': 0.458589,
        'y': 0.239243,
        'z': TABLE_HEIGHT,
      }),
      GModel('DepositBoxBlue', {
        'x': 0.460276,
        'y': 0.022851,
        'z': TABLE_HEIGHT,
      }),
    ]

    self.model_publisher = rospy.Publisher(
      '/gazebo/set_model_state', ModelState,
      queue_size=10, latch=True,
    )

    rospy.Subscriber(
      'gazebo/model_states',
      ModelStates,
      self.gazebo_broadcast_transforms
    )

    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.arm_move_group = moveit_commander.MoveGroupCommander("xarm6")
    self.gripper_move_group = moveit_commander.MoveGroupCommander("xarm_gripper")

    # Transform handlers
    self.br = tf2_ros.TransformBroadcaster()
    # Add tf2 handlers
    self.tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(self.tfBuffer)

    #Initial static transform (xArm)
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

    rospy.sleep(1)
    self.reset_moveit_objects()
    self.reset_gazebo_objects()


  # We update tf2 frames so that they have the same postiion as in Gazebo,
  # if the object is attached we will keep the same position relative to the ee.
  def gazebo_broadcast_transforms(self, data):
    objects = [
      'RedBox', 'GreenBox', 'BlueBox',
      'DepositBoxRed', 'DepositBoxGreen', 'DepositBoxBlue',
    ]

    attached_objects = self.scene.get_attached_objects().keys()

    for obj in set(data.name).intersection(objects):
      if obj in attached_objects:
        tf = self.tfBuffer.lookup_transform(
          'sensor_frame', 'link_attach',
          rospy.Time(), rospy.Duration(0.5)
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


  def tf_lookup(self, object_id, frame_id = 'sensor_frame'):
    # Use tf2 to retrieve the position of the target with respect
    # to the proper reference frame
    lookup = self.tfBuffer.lookup_transform(
      frame_id, object_id,
      rospy.Time(), rospy.Duration(3)
    )
    return lookup.transform


  #Not working yet
  def reset_moveit_objects(self):
    for model in self.models:
      model_pose = PoseStamped()

      model_transform = self.tf_lookup(model['name'], 'world')
      model_pose.header.frame_id = "world"

      model_pose.pose.position = model_transform.translation
      model_pose.pose.orientation = model_transform.rotation

      if('type' in model and model['type'] == 'box'):
        self.scene.add_box(model['name'], model_pose, size = model['size'])


  def reset_gazebo_objects(self):
    #little jump on render
    default_twist = Twist(Vector3(0, 0, 1), Vector3(0, 0, 0))
    for model in self.models:
      pose = Pose(
        Point(
          model['position']['x'],
          model['position']['y'],
          model['position']['z'],
        ),
        Quaternion(
          model['orientation']['x'],
          model['orientation']['y'],
          model['orientation']['z'],
          model['orientation']['w'],
        )
      )
      self.model_publisher.publish(model['name'], pose, default_twist , 'world')


if __name__ == '__main__':
  rospy.init_node("env_controller", anonymous=True)
  env = Environment()
  rospy.spin()

