#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool

class Demo:
    def __init__(self):
        self.free_drive_pub = rospy.Publisher("/ur_control_wrapper/enable_freedrive", Bool, queue_size=10)
        self.gripper_pub = rospy.Publisher("/ur_control_wrapper/gripper", Bool, queue_size=10)
        self.connect_pub = rospy.Publisher("/ur_control_wrapper/connect", Bool, queue_size=10)
    
    def run(self):
        while not rospy.is_shutdown():
            command_input = raw_input("Freedrive: fs-start; fe-end: Gripper: go-open; gc-close; connect: c-connect: ")
            if command_input == "fs":
                self.free_drive_pub(True)
            elif command_input == "fe":
                self.free_drive_pub(False)
            elif command_input == "go":
                self.gripper_pub(True)
            elif command_input == "gc":
                self.gripper_pub(False)
            elif command_input == "c":
                self.connect_pub(True)

if __name__ == '__main__':
    try:
        rospy.init_node('ur_control_wrapper_demo_mode', anonymous=True)

        demo = Demo()

        demo.run()
    except rospy.ROSInterruptException:
        pass