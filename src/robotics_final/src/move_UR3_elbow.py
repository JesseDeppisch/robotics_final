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

  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object. 
    ## We're interested in the arm, whih is 'manipulator'
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    # (Note that this is not used in the Gazebo demo)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher

  def go_to_joint_state(self):
    group = self.group

    joint_goal = group.get_current_joint_values()

    # Near goal (elbow singularity)
    joint_goal[0] = 0	
    joint_goal[1] = -pi/2 # So as not to collide with the table
    joint_goal[2] = pi/2
    joint_goal[3] = 0
    joint_goal[4] = pi/2
    joint_goal[5] = 0

    # TODO - alternatively, use group.set_named_target("singularity_wrist")

    # The go() command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling stop() ensures that there is no residual movement
    group.stop()

  def get_singularity_pose(self):
    group = self.group

    # Our goal is the elbow singularity, so make this the joint goal
    joint_goal = group.get_current_joint_values()

    # Goal (elbow singularity)
    joint_goal[0] = 0	
    joint_goal[1] = -.2 # So as not to collide with the table
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = pi/2
    joint_goal[5] = 0

    group.go(joint_goal, wait=True)

    # Calling stop() ensures that there is no residual movement
    group.stop()

    # We're at the desired pose, so return it
    return self.group.get_current_pose().pose
    #return geometry_msgs.msg.Pose() # BAD, and does NOT work

  def go_to_pose_goal(self):
    group = self.group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.w = 0.0164 # shouldn't chnange, I think
    pose_goal.position.x = 0.545
    pose_goal.position.y = 0.112
    pose_goal.position.z = 0.175

    # TODO - if this fails, simply don't change orientation from start to finish
    pose_goal.orientation.x = -0.995
    pose_goal.orientation.y = 3.637
    pose_goal.orientation.z = -0.09
    pose_goal.orientation.w = 0
   
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

  def go_to_pose(self, pose_goal):
    group = self.group

    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

  def plan_cartesian_path_waypoint(self, wpose):
    group = self.group

    # Copy the end waypoint pose to the list
    waypoints = []
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a high resolution
    # (accomplished by setting `eef_step` to 0.01, i.e. 1 cm)
    # Disabled jump threshold by setting it to 0.0, ensuring we follow the path
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.00)        # jump_threshold

    # Note: We are planning, NOT moving yet!
    return plan, fraction

  def execute_plan(self, plan):
    # **Note:** The robot's current joint state must be within some tolerance of the
    # first waypoint in the `RobotTrajectory`, else `execute()` will fail
    self.group.execute(plan, wait=True)

def main():
  # Wrap in try s.t. errors are handled
  try:
    print "============ Press `Enter` to set up the moveit_commander (press CTRL-D to exit) ..."
    raw_input()
    mgpi = MoveGroupPythonInterface()

    ########## [ New addition ] ###############################
    # First, go to target state by moving joints
    # 	Copy the pose() values
    # Next, go to some other orientation (by moving joints)
    #	Go to the previous pose values

    print "========= Note: for all lines that start with =========, press `Enter` to move to next line"

    # First, get the goal pose
    print "========= Go to target state (by moving joints) to obtain target pose values ..."
    raw_input()
    singularity_pose = mgpi.get_singularity_pose()

    # First demo: move to a starting state
    print "========= Go to start state ..."
    raw_input()
    mgpi.go_to_joint_state()

    # Then, move to the end state USING pose targeting
    # Note: this takes a curved path, because path is not planned explicitly
    print "========= Go to target joint state using pose targeting ..."
    raw_input()
    mgpi.go_to_pose(singularity_pose)

    # Go back to the same start state to set up the second demo
    print "**** Go to start state ..."
    raw_input()
    mgpi.go_to_joint_state() 

    # Use a cartesian path to target the same end state
    # The major difference is that the end effector follows a CONSTANT linear path
    # While the above method just targets the same pose, so the intermediate steps are neglected
    print "**** Plan and execute a cartesian path to the end state ..."
    raw_input()
    cartesian_plan, fraction = mgpi.plan_cartesian_path_waypoint(singularity_pose)
    mgpi.execute_plan(cartesian_plan)

    print "============ Elbow singularity demo complete!"
    print "Note: if the UR3 clipped into the ground, you must restart Gazebo"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
