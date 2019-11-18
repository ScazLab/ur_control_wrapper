#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Bool

class FreeDrive:
    def __init__(self):
        self.free_drive_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=10)
        
        #rospy.Subscriber("/crow_inverse_kinematic/robot_control/enable_freedrive", Bool, self.enable)
    
    def connect(self):
        "def externalControl():\n" + 
            "set_tool_voltage(24)\n"
            "set_tool_communication(True, 115200, 0, 1, 2, 4)\n"
        
            "steptime = get_steptime()\n"
            "textmsg(\"steptime=\", steptime)\n"
            "MULT_jointstate = 1000000\n"
        
            #Constants
            SERVO_STOPPED = -2
            SERVO_UNINITIALIZED = -1
            SERVO_IDLE = 0
            SERVO_RUNNING = 1
        
            #Global variables are also showed in the Teach pendants variable list
            global cmd_servo_state = SERVO_UNINITIALIZED
            global cmd_servo_q = get_actual_joint_positions()
            global cmd_servo_q_last = get_actual_joint_positions()
            global extrapolate_count = 0
            global extrapolate_max_count = 0
        
            def set_servo_setpoint(q):
                cmd_servo_state = SERVO_RUNNING
                cmd_servo_q_last = cmd_servo_q
                cmd_servo_q = q
            end
        
            def extrapolate():
                diff = [cmd_servo_q[0] - cmd_servo_q_last[0], cmd_servo_q[1] - cmd_servo_q_last[1], cmd_servo_q[2] - cmd_servo_q_last[2], cmd_servo_q[3] - cmd_servo_q_last[3], cmd_servo_q[4] - cmd_servo_q_last[4], cmd_servo_q[5] - cmd_servo_q_last[5]]
                cmd_servo_q_last = cmd_servo_q
                cmd_servo_q = [cmd_servo_q[0] + diff[0], cmd_servo_q[1] + diff[1], cmd_servo_q[2] + diff[2], cmd_servo_q[3] + diff[3], cmd_servo_q[4] + diff[4], cmd_servo_q[5] + diff[5]]
        
                return cmd_servo_q
            end
        
            thread servoThread():
                state = SERVO_IDLE
                while state > SERVO_STOPPED:
                    enter_critical
                    q = cmd_servo_q
                    do_extrapolate = False
                    if (cmd_servo_state == SERVO_IDLE):
                        do_extrapolate = True
                    end
                    state = cmd_servo_state
                    if cmd_servo_state > SERVO_UNINITIALIZED:
                        cmd_servo_state = SERVO_IDLE
                    end
        
                    if do_extrapolate:
                        extrapolate_count = extrapolate_count + 1
                        if extrapolate_count > extrapolate_max_count:
                            extrapolate_max_count = extrapolate_count
                        end
        
                        q = extrapolate()
                        servoj(q, t=steptime, lookahead_time=0.03, gain=2000)
        
                    elif state == SERVO_RUNNING:
                        extrapolate_count = 0
                        servoj(q, t=steptime, lookahead_time=0.03, gain=2000)
        
                    else:
                        extrapolate_count = 0
                        sync()
                    end
                    exit_critical
                end
                textmsg("servo thread ended")
                stopj(0.1)
            end
            socket_open("192.168.1.114", 50001, "reverse_socket")
        
            thread_servo = run servoThread()
            keepalive = -2
            textmsg("External control active")
            params_mult = socket_read_binary_integer(1+6, "reverse_socket")
            keepalive = params_mult[1]
            while keepalive > 0:
                enter_critical
                socket_send_line(1, "reverse_socket")
                params_mult = socket_read_binary_integer(1+6, "reverse_socket", 0.02) # steptime could work as well, but does not work in simulation
                if params_mult[0] > 0:
                    keepalive = params_mult[1]
                    if params_mult[1] > 1:
                        q = [params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate, params_mult[7] / MULT_jointstate]
                        set_servo_setpoint(q)
                    end
                else:
                    keepalive = keepalive - 1
                end
                exit_critical
            end
        
            textmsg("Stopping communication and servoing")
            cmd_servo_state = SERVO_STOPPED
            sleep(.1)
            socket_close("reverse_socket")
            kill thread_servo
        end
        
    
    def enable(self, enabled):
        if enabled:
            #self.free_drive_pub.publish('set robotmode freedrive')
            #self.free_drive_pub.publish('freedrive_mode()')
            #self.free_drive_pub.publish('teach_mode()')         
            self.free_drive_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')
        else:
            #self.free_drive_pub.publish('set robotmode run')
            #self.free_drive_pub.publish('end_freedrive_mode()')
            #self.free_drive_pub.publish('end_teach_mode()')
            self.free_drive_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
    
    def run(self):
        while not rospy.is_shutdown():
            command = raw_input("press s to start and o to stop")
            if command == "s":
                self.enable(True)
            if command == "o":
                self.enable(False)
                
            
    

    #def enable(self, data):
        #if data.data:
            ##self.free_drive_pub.publish('set robotmode freedrive')
            ##self.free_drive_pub.publish('freedrive_mode()')
            ##self.free_drive_pub.publish('teach_mode()')
            #self.free_drive_pub.publish('def myProg():\n\twhile (True):\n\t\tfreedrive_mode()\n\t\tsync()\n\tend\nend\n')
        #else:
            ##self.free_drive_pub.publish('set robotmode run')
            ##self.free_drive_pub.publish('end_freedrive_mode()')
            ##self.free_drive_pub.publish('end_teach_mode()')
            #self.free_drive_pub.publish('def myProg():\n\twhile (True):\n\t\tend_freedrive_mode()\n\t\tsleep(0.5)\n\tend\nend\n')
    
if __name__ == '__main__':
    try:
        rospy.init_node('set_free_drive_mode', anonymous=True)

        free_drive = FreeDrive()
        
        free_drive.run()

        #rospy.spin()
    except rospy.ROSInterruptException:
        pass