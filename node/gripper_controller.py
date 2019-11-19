#!/usr/bin/env python

import rospy

class Gripper:
	def __init__(self):
		# to update?? open/close with https://github.com/ctaipuj/luc_control/blob/master/robotiq_85_control/src/gripper_ur_control.cpp
		
		self.gripper_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=10)
		self.gripper_commands = self.get_gripper_command()
		self.command = "{{GRIPPER_COMMAND}}"
		
		self.activate_gripper()
		
		rospy.Subscriber("/ur_control_wrapper/gripper", Bool, control)
		
	def get_gripper_command(self):
		commands = ""

		rospack = rospkg.RosPack()

		with open(rospack.get_path("ur_control_wrapper") + "/resources/gripper.script", "r") as command_file:
			commands = command_file.read()

		return commands + "\n"

	def activate_gripper():
		command = self.gripper_commands.replace(self.command, "rq_activate_and_wait()")
		self.gripper_pub.publish(command)
	
	def control(self, data):
		if data.data:
			self.open_gripper()
		else:
			self.close_gripper()

	def open_gripper():
		command = self.gripper_commands.replace(self.command, "rq_open()")
		self.gripper_pub.publish(command)

	def close_gripper():
		command = self.gripper_commands.replace(self.command, "rq_close()")
		self.gripper_pub.publish(command)
	
	def deactivate_gripper():
		command = self.gripper_commands.replace(self.command, "")
		self.gripper_pub.publish(command)

if __name__ == '__main__':
	try:
		rospy.init_node('gripper_control_mode', anonymous=True)

		gripper_control = Gripper()

		rospy.spin()
		
		gripper_control.deactivate_gripper()
		
	except rospy.ROSInterruptException:
		pass	





