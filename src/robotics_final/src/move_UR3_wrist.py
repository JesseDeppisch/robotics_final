#!/usr/bin/env python

## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
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

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    #planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    #eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    #group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    #self.planning_frame = planning_frame
    #self.eef_link = eef_link
    #self.group_names = group_names

  def go_to_joint_state(self):
    group = self.group

    ## Planning to a Joint Goal
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

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

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

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    group = self.group

    ## Cartesian Paths
    waypoints = []
    wpose = group.get_current_pose().pose
    #print wpose

    # Move across singularity
    wpose.position.x += scale * 0.15  # First, move right (x)
    wpose.position.y -= scale * 0.15
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 0.5 cm
    # which is why we will specify 0.005 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.005,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def execute_plan(self, plan):
    group = self.group

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
  try:
    print "============ Press `Enter` to set up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    mgpi = MoveGroupPythonInterface()

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

