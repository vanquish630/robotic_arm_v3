#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>


ros::NodeHandle  nh;
Servo gripper;
Servo wrist;
Servo wrist_rot;
Servo elbow;
Servo shoulder;
Servo claw;
Servo base;


double base_angle=0;
double shoulder_angle=0;
double elbow_angle=0;
double wrist_angle=0;
double gripper_angle=0;
double wrist_rot_angle=0;
void servo_cb(const sensor_msgs::JointState& cmd_msg){
  base_angle=radiansToDegrees(cmd_msg.position[0]);
  shoulder_angle=radiansToDegrees(cmd_msg.position[1]);
  elbow_angle=radiansToDegrees(-1*cmd_msg.position[2]);
  wrist_angle=radiansToDegrees(cmd_msg.position[3]);
  wrist_rot_angle=radiansToDegrees(cmd_msg.position[4]);
  gripper_angle=radiansToDegrees(cmd_msg.position[5]);
  Serial.println(base_angle);
  Serial.println(shoulder_angle);
  Serial.println(elbow_angle);
  Serial.println(wrist_angle);
  Serial.println(wrist_rot_angle);
  
  base.write(base_angle);
  shoulder.write(shoulder_angle);
  wrist_rot.write(wrist_rot_angle);
  elbow.write(elbow_angle);
  wrist.write(wrist_angle);
  claw.write(gripper_angle);

  /*
   base.write(constrain(map(base_angle,-90,90,0,180),0,180));
  shoulder.write(constrain(map(shoulder_angle,-90,90,0,180),0,180));
  elbow.write(constrain(map(elbow_angle,-90,90,0,180),0,180));
  wrist.write(constrain(map(wrist_angle,-90,90,0,180),0,180));
  wrist_rot.write(constrain(map(wrist_rot_angle,-90,90,0,180),0,180));
  claw.write(gripper_angle);
   */
  
  
}


ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  claw.attach(6); 
  wrist_rot.attach(7);
  base.attach(8);
  shoulder.attach(9);
  elbow.attach(10); 
  wrist.attach(11);
  

  delay(10);
  
  base.write(90);
  shoulder.write(90);
  wrist_rot.write(90);
  elbow.write(90);
  wrist.write(90);
  claw.write(90);
  
}

void loop(){
  nh.spinOnce();
}

double angle_to_steps(double angle){
  return (4096/360)*angle;
  }
double radiansToDegrees(float position_radians)
{

  position_radians = position_radians + 1.6;

  return position_radians * 57.2958;

}
