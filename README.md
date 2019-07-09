# My_robotic_arm

This is a ros package for a 6DOF robotic arm which is able to pick and place objects.
This uses the demo.launch file made by moveit setup assistant. The arduino subscribes to joint_states topic and uses the values published by moveit(angle in radians)to move the servo motor accordingly. 

This is a work-around method and is not recommended. The recommended method would be to use controllers and to post feedback from motors to joint  states.
