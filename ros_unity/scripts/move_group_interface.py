#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv
from moveit_commander.conversions import pose_to_list
from ros_unity.srv import RobotMoverService, RobotMoverServiceRequest, RobotMoverServiceResponse
from ros_unity.srv import JointGoal, JointGoalRequest, JointGoalResponse
from ros_unity.srv import CylindricalWorkpiece, CylindricalWorkpieceRequest, CylindricalWorkpieceResponse


class MoveGroupInterface:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_node')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.group_name = "manipulator"
        
        self.plan_execute_path_subscriber = rospy.Service('/moveit', RobotMoverService, self.PlanExecutePath)
        self.joint_goal_service = rospy.Service('/joint_goal', JointGoal, self.GoToJointGoal)
        self.workpiece_origin_service = rospy.Service('/cylindrical_workpiece_parameters', CylindricalWorkpiece, self.CreateCylindricalObstacle)


        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        print("Unity services are running...")
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 1)
        self.plan = None
        self.fraction = None
        rospy.spin()













    # Plans or executes the path (or both) depending on the button pressed in Unity (Plan, Execute, Plan and Execute)
    def PlanExecutePath(self, req):

        # print(req)
        print("Request: \n{}".format(req))
        self.group_name = req.group_name.data
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        planning_frame = self.move_group.get_planning_frame()
        print("Planning frame: %s" % planning_frame)
        eef_link = self.move_group.get_end_effector_link()
        print("End effector link: %s" % eef_link)
        
        
        response = RobotMoverServiceResponse()
        
        if (req.plan_execute.data == "Plan" or req.plan_execute.data == "Plan and Execute"):
            self.plan = None
            self.fraction = None
            (self.plan, self.fraction) = self.move_group.compute_cartesian_path(req.targets, 0.1, 0.0)
            display_trajectory =  moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(self.plan)
            if (self.plan != None):
                response.status.data = "Successfully planned the path."
            print("Planning")
        
        if (req.plan_execute.data == "Execute" or req.plan_execute.data == "Plan and Execute"):
            self.move_group.execute(self.plan, wait=True)
            self.move_group.clear_pose_targets()
            self.move_group.stop()
            response.status.data = "Executed planned path."
            print("Executing")
        
        print(self.fraction)
        
        return response
        
        









        
        
        

    def GoToJointGoal(self, req):
        print(self.move_group.get_active_joints())
        joint_names = self.move_group.get_active_joints()
        joint_goal = self.move_group.get_current_joint_values()

        i = 0
        j = 0

        for joint in joint_names:
            print("looking for " + joint)
            j = 0
            for joint_srv in req.name:
                if joint == joint_srv:
                    joint_goal[i] = req.position[j]
                    print("Found " + joint)
                j += 1
            i += 1

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        response = JointGoalResponse()
        response.success = True
        
        print(self.move_group.get_current_joint_values())
        return response
    









    def CreateCylindricalObstacle(self, req):
        print("Creating cylindrical obstacle...")
        
        
        print("Request: \n{}".format(req))

        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = self.move_group.get_planning_frame()
        cylinder_pose.pose = req.origin

        self.scene.add_cylinder("workpiece", cylinder_pose, req.height.data, req.radius.data)

        response = CylindricalWorkpieceResponse()
        response.status.data = True

        return response
    













    def wait_for_state_update(
        self, box_is_known=False, timeout=4, object_name="workpiece"):
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if we are in the expected state
            if (object_name in self.scene.get_known_object_names()):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False













if __name__ == '__main__':

    movegroup_object = MoveGroupInterface()
