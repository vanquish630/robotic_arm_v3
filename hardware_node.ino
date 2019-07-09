#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>
#define motorPin1  4      // IN1 on the ULN2003 driver
#define motorPin2  5      // IN2 on the ULN2003 driver
#define motorPin3  6     // IN3 on the ULN2003 driver
#define motorPin4  7     // IN4 on the ULN2003 driver
#define MotorInterfaceType 8
AccelStepper base = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
ros::NodeHandle  nh;
Servo gripper;
Servo wrist;
Servo wrist_rot;
Servo elbow;
Servo shoulder;



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
  double steps = angle_to_steps(base_angle);
  base.moveTo(steps);
  base.setSpeed(100);
  base.setAcceleration(20);
  while(base.distanceToGo() != 0){
    base.run();
    }
  shoulder.write(shoulder_angle);
  wrist_rot.write(wrist_rot_angle);
  elbow.write(elbow_angle);
  wrist.write(wrist_angle);
  
  
}


ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  base.setCurrentPosition(0);
  base.setSpeed(100);
  base.setMaxSpeed(1000);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  
  //gripper.attach(8); 
  wrist.attach(10);
  wrist_rot.attach(11);
  elbow.attach(9); 
  shoulder.attach(8);
 

  delay(10);
  shoulder.write(90);
  wrist_rot.write(90);
  elbow.write(90);
  wrist.write(90);
  
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
