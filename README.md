# My_robotic_arm

This is a ros package for a 6DOF robotic arm which is able to pick and place objects.

# Instructions for use:

Start Gazebo using the following command:

roslaunch robotic_arm_pkg robotic_arm_bringup_gazebo.launch 

Start motion planning of arm in Gazebo from MoveIt! RViz GUI using the following command:

roslaunch moveit_config_arm_mesh robotic_arm_bringup_rviz.launch 

