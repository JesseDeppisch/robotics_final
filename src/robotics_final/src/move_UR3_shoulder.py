#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_mgpi',
                    anonymous=True)
    # TODO - change above "node" that is spawned

    ## Instantiate a `RobotCommander`_ object
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher

  def go_to_joint_state(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()

    # Shoulder singularity position
    joint_goal[0] = 1.9
    joint_goal[1] = -0.6
    joint_goal[2] = -2.1
    joint_goal[3] = 3.14
    joint_goal[4] = -1.5707
    joint_goal[5] = 0

    # TODO - alternatively, use group.set_named_target("singularity_wrist")

    # The go() command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling stop() ensures that there is no residual movement
    group.stop()

  def plan_cartesian_path(self, scale=1):
    group = self.group

    waypoints = []

    wpose = group.get_current_pose().pose
    #print wpose

    # Move across singularity
    wpose.position.x += scale * 0.15  # First, move right (x)
    wpose.position.y -= scale * 0.15
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a high resolution
    # (accomplished by setting `eef_step` to 0.01, i.e. 1 cm
    # Disable jump threshold by setting it to 0.0, ensuring we follow the path
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.00)        # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def execute_plan(self, plan):
    self.group.execute(plan, wait=True)

def main():
  try:
    print "============ Press `Enter` to set up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    mgpi = MoveGroupPythonInterface()

    print "============ Press `Enter` to go to start state (joint goal) ..."
    raw_input()
    mgpi.go_to_joint_state()

    print "============ Press `Enter` to move past shoulder singularity ..."
    raw_input()
    cartesian_plan, fraction = mgpi.plan_cartesian_path(scale=1)
    mgpi.execute_plan(cartesian_plan)

    print "============ Press `Enter` to go the opposite direction! ..."
    raw_input()
    cartesian_plan, fraction = mgpi.plan_cartesian_path(scale=-1.8)
    mgpi.execute_plan(cartesian_plan)

    print "============ Shoulder singularity demo complete!"
    print "Note: if the UR3 clipped into the ground, you must restart Gazebo"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
