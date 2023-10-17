#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv
from moveit_commander.conversions import pose_to_list
from ros_unity.msg import RobotMoverMessage


class MoveGroupInterface:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_node_subscriber')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.group_name = "manipulator"
        self.plan_execute_path_subscriber = rospy.Subscriber('/moveit_topic', RobotMoverMessage, self.PlanExecutePath)

        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        print("Ready to plan")
        #self.move_group_goal_publisher = rospy.Publisher('move_group/goal', moveit_msgs.msg.)
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 1)
        self.plan = None
        self.fraction = None
        rospy.spin()

    # Plans or executes the path (or both) depending on the button pressed in Unity (Plan, Execute, Plan and Execute)
    def PlanExecutePath(self, req):

        self.group_name = req.group_name.data
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        planning_frame = self.move_group.get_planning_frame()
        print "Reference frame: %s" % planning_frame
        eef_link = self.move_group.get_end_effector_link()
        print ("End effector link: %s" % eef_link)
        current_joint_state = self.move_group.get_current_joint_values()
        print(req.plan_execute.data)
        
        if (req.plan_execute.data == "Plan" or req.plan_execute.data == "Plan and Execute"):
            self.plan = None
            self.fraction = None
            (self.plan, self.fraction) = self.move_group.compute_cartesian_path(req.targets, 0.1, 0.0)
            display_trajectory =  moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(self.plan)
            #self.display_trajectory_publisher.publish(display_trajectory)

            print("Planning")
        
        if (req.plan_execute.data == "Execute" or req.plan_execute.data == "Plan and Execute"):
            self.move_group.execute(self.plan, wait=True)
            self.move_group.clear_pose_targets()
            self.move_group.stop()

            print("Executing")
            
            
        print(self.fraction)
        



if __name__ == '__main__':

    movegroup_object = MoveGroupInterface()
