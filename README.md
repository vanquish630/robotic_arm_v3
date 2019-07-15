# My_robotic_arm

This is a ros package for a 6DOF robotic arm which is able to pick and place objects.

# Instructions for use with gazebo :

Start Gazebo using the following command:

roslaunch robotic_arm_pkg robotic_arm_bringup_gazebo.launch 

Start motion planning of arm in Gazebo from MoveIt! RViz GUI using the following command:

roslaunch moveit_config_arm_mesh robotic_arm_bringup_rviz.launch 

Start planning in rviz and hit execute. The arm model in gazebo should also move.


# Instructions for use with real robot :

This uses the demo.launch file made by moveit setup assistant. The arduino subscribes to joint_states topic and uses the values published by moveit(angle in radians)to move the servo motor accordingly. 

This solution is a bit hacky and not how it is done with industrial robots. Ideally you are supposed to publish the movement trajectory on /FollowJointState topic and then receive the feedback on /JointState topic.But in our arm the hobby servos can't provide the feedback, so we'll just directly subscribe to /JointState topic, published by FakeRobotController node.

1) launch demo.launch
2)in a new terminal type the following :

    sudo chmod -R 777 /dev/ttyUSB0
    
    rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
    
  where you can replace ttyUSB0 with what appears in the port section of your arduino IDE. In my case it was ttyACM0.
  
3) Plan and execute in demo.launch. You should see the real robot move.
