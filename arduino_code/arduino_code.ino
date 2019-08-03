
//rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200


#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif



#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <robotic_arm_pkg/Floats_array.h>
#include <std_msgs/String.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h> 


float degree_to_rad(int deg){
  return (3.14/180)*deg;
  
  }
  
ros::NodeHandle  nh;

int a0,a4;
int cur_pos[5]={0,0,0,0,0};

sensor_msgs::JointState joint_state;
ros::Publisher joint_pub("arm_mesh/joint_states", &joint_state);

Servo base,shoulder,elbow,wrist1,wrist2,claw;
float readbase=0,readshoulder=0,readelbow=0,readwrist1=0,readwrist2=0,readservo5=0;


void rotate_servo(int servo,int new_pos,int cur_pos,int dir)
{
 
}



int new_pos[5];
void servo_cb( const rospy_tutorials::Floats& cmd_msg){
 // nh.loginfo("Command Received");
  
 new_pos[0]=cmd_msg.data[0];
 new_pos[1]= cmd_msg.data[1];
 new_pos[2]= cmd_msg.data[2];
 new_pos[3]= cmd_msg.data[3];
 new_pos[4]=cmd_msg.data[4];
 a0 = new_pos[0];
 a4 = new_pos[4];
  
  int i=0;

  for(i=0;i<5;i++)
  {
    if (new_pos[i]>cur_pos[i])
    {
      rotate_servo(i,new_pos[i],cur_pos[i],1);
      cur_pos[i]=new_pos[i];
        
    }
    else if(new_pos[i]<cur_pos[i])
    {
      rotate_servo(i,new_pos[i],cur_pos[i],-1);
      cur_pos[i]=new_pos[i];
    }
    
  }
  
}


ros::Subscriber<rospy_tutorials::Floats> sb("/joints_to_aurdino", servo_cb);


void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sb);
//  nh.advertiseService(server);

  claw.attach(6);
  wrist2.attach(7);
  base.attach(8);
  shoulder.attach(9);
  elbow.attach(10);
  wrist1.attach(11);
  
  base.write(90);
  shoulder.write(90);
  elbow.write(90);
  wrist1.write(90);
  wrist2.write(90);
  //claw.write(90);

  char *joint_names[] ={"base_link1","link1_link2","link2_link3","link3_link4","link4_gripper_base"};
  float pos[5]; /// stores arduino time
  joint_state.name_length = 5;
  joint_state.position_length = 5;
  joint_state.name = joint_names;
  //joint_state.header.frame_id = "base";
  joint_state.position = pos;
  nh.advertise(joint_pub);


 // joint_state.header.stamp = nh.now();
  
    delay(100);
  
}

void loop(){
  
   joint_state.header.stamp = nh.now();
   //int readshoulder=analogRead(A1);
   //int readelbow=analogRead(A2);
   //int readwrist=analogRead(A3);
   joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    joint_pub.publish(&joint_state);
/*
   int sd = constrain(map(readshoulder,110,438,0,180),0,180);
   int ed = constrain(map(readelbow,114,430,0,180),0,180);
   int wd = constrain(map(readwrist,116,444,0,180),0,180);
   Serial.println(sd);
   Serial.println(ed);
   Serial.println(wd);
   Serial.println();

   int shoulder_angle = constrain(map(sd,0,180,-90,90),-90,90);
   int elbow_angle=constrain(map(ed,0,180,-90,90),-90,90);
   int wrist_angle=constrain(map(wd,0,180,-90,90),-90,90);

   Serial.println(shoulder_angle);
   Serial.println(elbow_angle);
   Serial.println(wrist_angle);
   Serial.println();

    float shoulder_angle_rad = degree_to_rad(shoulder_angle);
    float elbow_angle_rad=degree_to_rad(elbow_angle);
    float wrist_angle_rad=degree_to_rad(wrist_angle);
    joint_state.position[1] = shoulder_angle_rad;
    joint_state.position[2] = elbow_angle_rad;
    joint_state.position[3] = wrist_angle_rad;
    joint_pub.publish(&joint_state);
    delay(100);
    */


  nh.spinOnce();
}
