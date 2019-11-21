#!/usr/bin/env python

import rospy
import rospkg

from std_msgs.msg import String, Bool

class ExternalControl:
    def __init__(self):
        self.connect_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=10)
        self.external_control = self.get_external_control_command()
        
        rospy.Subscriber("/ur_control_wrapper/connect", Bool, self.connect_to_robot)
    
    def get_external_control_command(self):
        commands = ""
        
        tool_voltage = rospy.get_param("/ur_hardware_interface/tool_voltage")
        tool_parity = rospy.get_param("/ur_hardware_interface/tool_parity")
        tool_baud_rate = rospy.get_param("/ur_hardware_interface/tool_baud_rate")
        tool_stop_bits = rospy.get_param("/ur_hardware_interface/tool_stop_bits")
        tool_rx_idle_chars = rospy.get_param("/ur_hardware_interface/tool_rx_idle_chars")
        tool_tx_idle_chars = rospy.get_param("/ur_hardware_interface/tool_tx_idle_chars")
        
        mult_jointstate = 1000000
        servo_j_replace = "lookahead_time=0.03, gain=2000"
        local_ip = "192.168.1.114" # may needs to be changed if your local host is with a different ip
        reverse_port = 50001
        
        rospack = rospkg.RosPack()
        
        with open(rospack.get_path("ur_control_wrapper") + "/resources/externalcontrol_urscript.txt", "r") as command_file:
            commands = command_file.read()
        
        commands = commands.replace("{{TOOL_VOLTAGE}}", str(tool_voltage))
        commands = commands.replace("{{TOOL_PARITY}}", str(tool_parity))
        commands = commands.replace("{{TOOL_BAUD_RATE}}", str(tool_baud_rate))
        commands = commands.replace("{{TOOL_STOP_BITS}}", str(tool_stop_bits))
        commands = commands.replace("{{TOOL_RX_IDLE_CHARS}}", str(tool_rx_idle_chars))
        commands = commands.replace("{{TOOL_TX_IDLE_CHARS}}", str(tool_tx_idle_chars))
        
        commands = commands.replace("{{JOINT_STATE_REPLACE}}", str(mult_jointstate))
        commands = commands.replace("{{SERVO_J_REPLACE}}", str(servo_j_replace))
        commands = commands.replace("{{SERVER_IP_REPLACE}}", str(local_ip))
        commands = commands.replace("{{SERVER_PORT_REPLACE}}", str(reverse_port))
        
        return commands + "\n"
    
    def connect_to_robot(self, data):
        if data.data:
            self.connect_pub.publish(self.external_control)

if __name__ == '__main__':
    try:
        rospy.init_node('ur_control_wrapper_custom_external_control_mode', anonymous=True)

        external_control = ExternalControl()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass