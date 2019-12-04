#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# adapted from moveit online tutorial

import sys
import copy
import rospy
import numpy as np
from tf import transformations as tfs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from math import pi
from std_msgs.msg import String, Header

from ur_control_wrapper.srv import SetPose, SetPoseResponse
from ur_control_wrapper.srv import GetPose, GetPoseResponse
from ur_control_wrapper.srv import CheckPose, CheckPoseResponse
from ur_control_wrapper.srv import SetJoints, SetJointsResponse
from ur_control_wrapper.srv import GetJoints, GetJointsResponse
from ur_control_wrapper.srv import SetTrajectory, SetTrajectoryResponse
from ur_control_wrapper.srv import AddBox, AddBoxResponse
#from ur_control_wrapper.srv import ForwardKinematics, ForwardKinematicsResponse
from ur_control_wrapper.srv import GetInterpolatePoints, GetInterpolatePointsResponse

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
        goal_list = pose_to_list(goal)
        actual_list = pose_to_list(actual)
        
        print "goal list: ", pose_to_list(goal)
        print "actual list: ", pose_to_list(actual)
        
        return all_close(goal_list[:3], actual_list[:3], tolerance) and all_close(goal_list[3:], actual_list[3:], tolerance * 10)

    return True

def pose_to_list(pose_msg):
    pose_list = []
    pose_list.append(pose_msg.position.x)
    pose_list.append(pose_msg.position.y)
    pose_list.append(pose_msg.position.z)
    pose_list += tfs.euler_from_quaternion(np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]))
    return pose_list

class Kinematics(object):
    def __init__(self):
        super(Kinematics, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()  

        group_name = "manipulator"
        #group_name = "endeffector"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        #print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        #print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        #print "============ Available Planning Groups:", robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        rospy.Service('/ur_control_wrapper/set_pose', SetPose, self.set_pose)
        rospy.Service('/ur_control_wrapper/check_pose', CheckPose, self.check_pose)
        rospy.Service('/ur_control_wrapper/get_pose', GetPose, self.get_pose)
        rospy.Service('/ur_control_wrapper/set_joints', SetJoints, self.set_joints)
        rospy.Service('/ur_control_wrapper/get_joints', GetJoints, self.get_joints)
        rospy.Service('/ur_control_wrapper/follow_trajectory', SetTrajectory, self.set_trajectory)
        rospy.Service('/ur_control_wrapper/add_box', AddBox, self.add_box)
        #rospy.Service('/ur_control_wrapper/forward_kinematics', ForwardKinematics, self.forward_kinematics)
        rospy.Service('/ur_control_wrapper/get_interpolate_points', GetInterpolatePoints, self.get_interpolate_points)
    
    def convert_joint_msg_to_list(self, joint_msg):
        joint_names = joint_msg.name
        joints = joint_msg.position
        return [joints[joint_names.index(name)] for name in self.joint_names]
        
    def convert_joint_list_to_message(self, joints):
        msg = sensor_msgs.msg.JointState()
        msg.name = self.joint_names
        msg.position = joints
        return msg
    
    def get_joints(self, data):
        return GetJointsResponse(self.convert_joint_list_to_message(self.move_group.get_current_joint_values()))

    def set_joints(self, data):
        move_group = self.move_group

        joints = self.convert_joint_msg_to_list(data.request_joints)

        move_group.go(joints, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        
        is_reached = all_close(joints, current_joints, 0.01)
        
        return SetJointsResponse(is_reached, self.convert_joint_list_to_message(current_joints))

    def get_pose(self, data):
        return GetPoseResponse(self.move_group.get_current_pose().pose)

    def check_pose(self, data):
        move_group = self.move_group
        pose_goal = data.request_pose

        (plan, fraction) = move_group.compute_cartesian_path([pose_goal], 10.0, 0.0)
        plan = plan.joint_trajectory
        
        move_group.clear_pose_targets()
        
        joint_names = plan.joint_names
        joints = plan.points[-1].positions
        
        msg = sensor_msgs.msg.JointState()
        msg.name = plan.joint_names
        msg.position = joints
        
        current_angle = self.convert_joint_msg_to_list(self.convert_joint_list_to_message(self.move_group.get_current_joint_values())) # to ensure the name list sequence is the same
        planned_angle = self.convert_joint_msg_to_list(msg) # to ensure the name list sequence is the same
        changes = abs(np.array(planned_angle) - np.array(current_angle))
        
        could_reach = len(plan.points) > 1
        joint_changes = sum(changes)
        
        move_group.clear_pose_targets()
        
        return CheckPoseResponse(could_reach, joint_changes, msg)

    def set_pose(self, data):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = data.request_pose

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        is_reached = all_close(pose_goal, current_pose, 0.01)
        return SetPoseResponse(is_reached, current_pose)
    
    def set_trajectory(self, data):
        move_group = self.move_group
        (plan, fraction) = move_group.compute_cartesian_path(data.trajectory, 0.01, 0.0)
        move_group.execute(plan, wait=True)
        
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()        
        
        current_pose = self.move_group.get_current_pose().pose
        is_reached = all_close(data.trajectory[-1], current_pose, 0.01)
        
        return SetTrajectoryResponse(is_reached, current_pose)
    
    def add_plane(self, data):
        box_name = data.name
        pose = data.pose
        normal = (data.normal.x, data.normal.y, data.normal.z)
        offset = data.offset
        
        scene = self.scene

        scene.add_plane(name, pose, normal, offset)

    def forward_kinematics(self, joint_state_msg):
        # https://groups.google.com/forum/#!topic/moveit-users/Wb7TqHuf-ig
        pose = None
        rospy.wait_for_service('compute_fk')
        try:
            fk = rospy.ServiceProxy('compute_fk', moveit_msgs.msg.GetPositionFK)
            header = Header(0, rospy.Time.now(), "/world")
            fk_link_names = [self.move_group.get_end_effector_link()]
            robot_state = self.robot.get_current_state()
            robot_state.joint_state = joint_state_msg
            pose = fk(header, fk_link_names, robot_state).pose_stamped[0].pose
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)
        
        return pose

    def get_interpolate_points(self, data):
        move_group = self.move_group
        (plan, fraction) = move_group.compute_cartesian_path([data.start, data.end], data.step, 0.0)
        plan = plan.joint_trajectory
        
        move_group.clear_pose_targets()
        
        joint_names = plan.joint_names
        
        poses = []
        
        for joints in plan.points:
            msg = sensor_msgs.msg.JointState()
            msg.name = plan.joint_names
            msg.position = joints.positions
            pose_msg = forward_kinematics(msg)
            poses.pose_msg
            move_group.clear_pose_targets()
        
        return GetInterpolatePointsResponse(poses)

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL


    def add_box(self, data):
        timeout = 4
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose = data.pose
        box_name = data.name
        box_size = (data.size.x, data.size.y, data.size.z)
        scene.add_box(box_name, box_pose, size=box_size)

        return AddBoxResponse(self.wait_for_state_update(box_is_known=True, timeout=timeout))

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

if __name__ == '__main__':
    try:
        rospy.init_node('ur_control_wrapper_kinematics', anonymous=True)

        kinematics = Kinematics()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass