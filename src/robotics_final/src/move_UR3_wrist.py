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

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory` publisher which is used later to publish
    ## trajectories for RViz to visualize:
    ## (This is NOT used in the Gazebo demo)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    
  def go_to_joint_state(self):
    group = self.group

    # Planning to a joint goal
    joint_goal = group.get_current_joint_values()
    
    # Wrist singularity position
    joint_goal[0] = -1.5707
    joint_goal[1] = -1.3
    joint_goal[2] = -1.83
    joint_goal[3] = -3.1415
    joint_goal[4] = 0.785
    joint_goal[5] = 0

    # TODO - alternatively, use group.set_named_target("singularity_wrist")

    # The go() command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling stop() ensures that there is no residual movement
    group.stop()

  def go_to_pose_goal(self):
    group = self.group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

  def plan_cartesian_path(self, scale=1):
    group = self.group

    ## Cartesian Paths
    waypoints = []
    wpose = group.get_current_pose().pose
    #print wpose

    # Move across singularity
    wpose.position.x += scale * 0.15
    wpose.position.y -= scale * 0.15
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 0.5 cm
    # which is why we will specify 0.005 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.005,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are planning, NOT moving yet!
    return plan, fraction

  def execute_plan(self, plan):
    self.group.execute(plan, wait=True)



def main():
  try:
    print "============ Press `Enter` to set up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    mgpi = MoveGroupPythonInterface()

    print "========= Note: for all lines that start with =========, press `Enter` to move to next line"

    print "============ Press `Enter` to go to start state (joint goal) ..."
    raw_input()
    mgpi.go_to_joint_state()

    print "============ Press `Enter` to move past wrist singularity ..."
    raw_input()
    cartesian_plan, fraction = mgpi.plan_cartesian_path()
    mgpi.execute_plan(cartesian_plan)

    print "============ Press `Enter` to go the opposite direction! ..."
    raw_input()
    cartesian_plan, fraction = mgpi.plan_cartesian_path(scale=-1)
    mgpi.execute_plan(cartesian_plan)

    print "============ Wrist singularity demo complete!"
    print "Note: if the UR3 clipped into the ground, you must restart Gazebo"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

