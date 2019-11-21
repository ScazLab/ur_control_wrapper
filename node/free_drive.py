#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Bool

class FreeDrive:
    def __init__(self):
        self.free_drive_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=10)
        self.connect_pub = rospy.Publisher("/ur_control_wrapper/connect", Bool, queue_size=10)
        
        rospy.Subscriber("/ur_control_wrapper/enable_freedrive", Bool, self.enable)
        
    def enable(self, data):
        #self.connect_pub.publish(False)
        if data.data:        
            self.free_drive_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')
        else:
            self.free_drive_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
            rospy.sleep(0.1)
            self.connect_pub(True)
        #rospy.sleep(1.0)
        #self.connect_pub.publish(True)
    
if __name__ == '__main__':
    try:
        rospy.init_node('ur_control_wrapper_set_free_drive_mode', anonymous=True)

        free_drive = FreeDrive()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass