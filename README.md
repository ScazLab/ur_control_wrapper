# NOTE

We released this open source control wrapper, hoping that it may help more people to have fun with the robot:) . But due to time constraints, we will not be able to help with installing/setting up the system. For those questions, you may get help from the Universal Robot's forum. But if you notice a bug that is specific to running this package, you are very welcome to let us know. Thank you!

# ur_control_wrapper

This is a ros wrapper to control The UR arm with the Robotiq 2F 85 gripper, with moveit. The setup instruction could be found here: <https://scazlab.github.io/ur5e_setup_guide.html>

The UR ros driver is the latest one found here: <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>

The control currently include the following functionalities:

- get/set the joint angles
- get/set the end effector pose (e.g., kinematics and inverse kinematics)
- open/close the gripper
- start/end the freedrive mode
- follow a trajectory
- add/remove, attach/detach an object

## Usage:

Make sure in the robot driver launch file, the arg headless_mode is set to True

Launch the ros driver launch file and moveit launch file

Then launch `ur_control_wrapper.launch` 

Or you can run `ur5e_cam_2f85_control.launch` without running the three launch files mentioned above. Or `sim_ur5e_cam_2f85_control.launch` if you works with the simulator.

It has the following topics:

- /ur_control_wrapper/connect: It takes `Bool`, to start the externalcontrol program to connect the driver to the robot to enable robot arm control.
- /ur_control_wrapper/enable_freedrive: It takes `Bool` to start or end the freedrive mode. It will stop the externalcontrol program. If you send True, then you will need to manually restart the externalcontrol program by send true to the connect topic (as it is not known how long it would need to be in this state). However, if you exit the mode by sending False, you don't need to reconnect it manually, as the program already do it for you.
- /ur_control_wrapper/gripper: It takes `Bool` to open or close the gripper. It will also stop the externalcontrol program. But you don't need to start externalcontrol in this case, since the program already does it for you.

It runs the folliwng services:

- /ur_control_wrapper/get_joints: get the joint angles
- /ur_control_wrapper/get_pose: get the end effector pose
- /ur_control_wrapper/set_joints: set the joint angles
- /ur_control_wrapper/set_pose: set the end effector pose
- /ur_control_wrapper/check_pose: check whether a pose could be reached
- /ur_control_wrapper/follow_trajectory: follow a trajectory. NOTE: do not add the starting pose if it is the same as the current pose. This is a moveit bug. [https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/]
- /ur_control_wrapper/add_object: add an object to scene. Currently support Box and Cylinder. For cylinder, the size is (height, radius, not use)
- /ur_control_wrapper/attach_object: attach an object to the end effector (there is no need to call add object as it already handles it). Currently support Box and Cylinder.
- /ur_control_wrapper/detach_object: attach an object from the end effector. If to_remove is set to True, it also remove the object from the scene.
- /ur_control_wrapper/remove_object: remove an object form the scene.
- /ur_control_wrapper/forward_kinematics: get the pose with joint angles.
- /ur_control_wrapper/get_interpolate_points: get the waypoints.

They are all blocking calls.

To run the demo, launch: `demo.launch` or run demo.py directly with rosrun if ur_control_warpper.launch already launched.

## Notes:

local_ip in connect.py needs to be updated if used on a different computer
