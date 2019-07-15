#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <robotic_arm_pkg/Floats_array.h>
#include <Servo.h> 

ros::NodeHandle  nh;


int cur_pos[3]={0,0,0};

Servo shoulder,elbow,wrist1,wrist2;

float readshoulder=0,readelbow=0,readwrist1=0,readwrist2=0,readservo5=0;

void rotate_servo(int servo,int new_pos,int cur_pos,int dir)
{
  int pos = 0;
  
  if (servo==0)
  {
    if (new_pos<5)
      new_pos=5;
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        shoulder.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        shoulder.write(pos);
        delay(10);
      }      
    }
  }
  if (servo==1)
  {
    if (new_pos<12)
      new_pos=12;
    
    
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        elbow.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        elbow.write(pos);
        delay(10);
      }      
    }
  }
  if (servo==2)
  {
    if (new_pos<6)
      new_pos=6;
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        wrist1.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        wrist1.write(pos);
        delay(10);
      }      
    }
  }
  if (servo==3)
  {
    if (new_pos<6)
      new_pos=6;
    if (dir == 1)
    {
      for(pos=cur_pos;pos<= new_pos;pos += 1)
      {
        wrist2.write(pos);
        delay(10);
      }      
    }
    else if(dir == -1)
    {
      for(pos=cur_pos;pos>= new_pos;pos -= 1)
      {
        wrist2.write(pos);
        delay(10);
      }      
    }
  }
  
}


void servo_cb( const rospy_tutorials::Floats& cmd_msg){
  //nh.loginfo("Command Received");
  
  int new_pos[5]={cmd_msg.data[0],cmd_msg.data[1],cmd_msg.data[2],cmd_msg.data[3],cmd_msg.data[4]};
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

void callback(const robotic_arm_pkg::Floats_array::Request & req, robotic_arm_pkg::Floats_array::Response & res)
{
  // Simulate function running for a non-deterministic amount of time
  

  res.res_length=3;
  readshoulder=analogRead(A0);
  readelbow=analogRead(A1);
  readwrist1=analogRead(A2);
  readwrist2=analogRead(A3);
  readservo5=analogRead(A4);

  res.res[0]=(readshoulder-100) * (180.0/325.0);
  res.res[1]=(readelbow-103) * (180.0/314.0);
  res.res[2]=(readwrist1-96) * (180.0/339.0);
  res.res[3]=(readwrist2-96) * (180.0/339.0); 

  return;
  
}



ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", servo_cb);
ros::ServiceServer<robotic_arm_pkg::Floats_array::Request, robotic_arm_pkg::Floats_array::Response> server("/read_joint_state",&callback);


void setup(){

  nh.initNode();
  nh.subscribe(sub);
  nh.advertiseService(server);
  
  shoulder.attach(8);
  elbow.attach(9);
  wrist1.attach(10);
  wrist2.attach(11);

  shoulder.write(0);
  elbow.write(0);
  wrist1.write(0);
  wrist2.write(0);
  

  
  
}

void loop(){

  nh.spinOnce();
}

