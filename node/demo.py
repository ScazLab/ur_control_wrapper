#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from ur_control_wrapper.srv import SetPose
from ur_control_wrapper.srv import GetPose
from ur_control_wrapper.srv import GetJoints
from ur_control_wrapper.srv import SetJoints

from ur_control_wrapper.srv import AddObject, AddObjectRequest
from ur_control_wrapper.srv import AttachObject, AttachObjectRequest
from ur_control_wrapper.srv import DetachObject, DetachObjectRequest
from ur_control_wrapper.srv import RemoveObject, RemoveObjectRequest

class Demo:
    def __init__(self):
        self.free_drive_pub = rospy.Publisher("/ur_control_wrapper/enable_freedrive", Bool, queue_size=10)
        self.gripper_pub = rospy.Publisher("/ur_control_wrapper/gripper", Bool, queue_size=10)
        self.connect_pub = rospy.Publisher("/ur_control_wrapper/connect", Bool, queue_size=10)
    
    def get_pose(self):
        rospy.wait_for_service("/ur_control_wrapper/get_pose")
        get_current_pose = rospy.ServiceProxy("/ur_control_wrapper/get_pose", GetPose)
        current_pose = None
        try:
            current_pose = get_current_pose().pose
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc) 
        return current_pose
    
    def get_angle(self):
        rospy.wait_for_service("/ur_control_wrapper/get_joints")
        get_current_joints = rospy.ServiceProxy("/ur_control_wrapper/get_joints", GetJoints)
        current_joints = None
        try:
            current_joints = get_current_joints().joints
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc) 
        return current_joints
    
    def set_default_angles(self):
        rospy.wait_for_service("/ur_control_wrapper/set_joints")
        set_joints = rospy.ServiceProxy("/ur_control_wrapper/set_joints", SetJoints)
        joints = JointState()
        joints.name = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joints.position = [2.242737118397848, -1.9379054508604945, -4.880746666585104, -0.3946760457805176, 1.2857670783996582, 4.764250755310059]
        try:
            response = set_joints(joints)
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)      
    
    def add_box(self):
        rospy.wait_for_service("/ur_control_wrapper/add_object")
        add_box = rospy.ServiceProxy("/ur_control_wrapper/add_object", AddObject)
        try:
            name = "box_object"
            pose = Pose(Point(-0.34, -0.0075, -0.023), Quaternion(0.0, 0.0, 0.0, 1.0))
            size = Vector3(0.02, 0.02, 0.02)
            object_type = AddObjectRequest.TYPE_BOX
            response = add_box(name, pose, size, AddObjectRequest.TYPE_BOX).is_success
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)
    
    def attach_box(self):
        print "wait for service"
        rospy.wait_for_service("/ur_control_wrapper/attach_object")
        print "service found"
        attach_box = rospy.ServiceProxy("/ur_control_wrapper/attach_object", AttachObject)
        try:
            name = "box_tool"
            pose = Pose(Point(-0.34, -0.0075, -0.023), Quaternion(0.0, 0.0, 0.0, 1.0))
            size = Vector3(0.02, 0.02, 0.02)
            object_type = AttachObjectRequest.TYPE_BOX
            response = attach_box(name, pose, size, object_type).is_success
            if response:
                print "successfully attached!"
            else:
                print "did not attach successfully"
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)
    
    def detach_box(self):
        rospy.wait_for_service("/ur_control_wrapper/detach_object")
        detach_box = rospy.ServiceProxy("/ur_control_wrapper/detach_object", DetachObject)
        try:
            name = "box_tool"
            response = detach_box(name).is_success
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)
    
    def remove_box(self):
        rospy.wait_for_service("/ur_control_wrapper/remove_object")
        remove_box = rospy.ServiceProxy("/ur_control_wrapper/remove_object", RemoveObject)
        try:
            name = "box_object"
            response = remove_box(name).is_success
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)
    
    def move_arm(self, direction):
        current_pose = self.get_pose()
        
        rospy.wait_for_service("/ur_control_wrapper/set_pose")
        set_current_pose = rospy.ServiceProxy("/ur_control_wrapper/set_pose", SetPose)
        try:
            if direction == "x+":
                current_pose.position.x += 0.05
            elif direction == "x-":
                current_pose.position.x -= 0.05
            elif direction == "y+":
                current_pose.position.y += 0.05
            elif direction == "y-":
                current_pose.position.y -= 0.05
            elif direction == "z+":
                current_pose.position.z += 0.05
            elif direction == "z-":
                current_pose.position.z -= 0.05
            response = set_current_pose(current_pose)
            pose = response.response_pose
            is_reached = response.is_reached
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)
    
    def run(self):
        while not rospy.is_shutdown():
            print "====================================================="
            command_input = raw_input("Freedrive: fs(start);fe-end: \nGripper: go(open); gc(close);\nConnect: c(connect);\nGet End Effector Pose: ep; \nGet joint angles: ja; \nGo to Default Position: d; \nBoxes: ab(add box); atb(attach box); db(detach box); rb(remove box) \nMove arm: x+(x direction move up 5 cm); x-; y+; y-; z+; z-: \n")
            if command_input == "fs":
                self.free_drive_pub.publish(True)
            elif command_input == "fe":
                self.free_drive_pub.publish(False)
            elif command_input == "go":
                self.gripper_pub.publish(True)
            elif command_input == "gc":
                self.gripper_pub.publish(False)
            elif command_input == "c":
                self.connect_pub.publish(True)
            elif command_input == "ep":
                print self.get_pose()
            elif command_input == "ja":
                print self.get_angle()
            elif command_input == "d":
                self.set_default_angles()
            elif command_input == "ab":
                self.add_box()
            elif command_input == "atb":
                self.attach_box()
            elif command_input == "db":
                self.detach_box()
            elif command_input == "rb":
                self.remove_box()
            else: # move arm
                direction = command_input
                self.move_arm(direction)

if __name__ == '__main__':
    try:
        rospy.init_node('ur_control_wrapper_demo_mode', anonymous=True)

        demo = Demo()

        demo.run()
    except rospy.ROSInterruptException:
        pass