#include <robotic_arm_pkg/robot_hardware_interface.h>


float joint_position[5]={0,0,0,0,0};
void callback(const sensor_msgs::JointState& msg){
    joint_position[0]=msg.position[0];
    joint_position[1]=msg.position[1];
    joint_position[2]=msg.position[2];
    joint_position[3]=msg.position[3];
    joint_position[4]=msg.position[4];


}

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
    pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
    //client = nh_.serviceClient<robotic_arm_pkg::Floats_array>("/read_joint_state");
    sub = nh_.subscribe("arm_mesh/joint_states", 100, callback);
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    num_joints_=5;
    joint_names_[0]="base_link1";   
    joint_names_[1]="link1_link2";
    joint_names_[2]="link2_link3";
    joint_names_[3]="link3_link4";
    joint_names_[4]="link4_gripper_base";


    for (int i = 0; i < num_joints_; ++i) {

        //ROS_INFO("Loading joint name: " << joint_names_[i]);
         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        //hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);

        hardware_interface::JointHandle jointPositionHandle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    //ROS_INFO("GenericHWInterface Ready.");
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void ROBOTHardwareInterface::read() {


    
        joint_position_[0]=joint_position[0];
        joint_position_[1]=joint_position[1];
        joint_position_[2]=joint_position[2];
        joint_position_[3]=joint_position[3];
        joint_position_[4]=joint_position[4];
        
    

    
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    
    joints_pub.data.clear();
    joints_pub.data.push_back((angles::to_degrees(joint_position_command_[0])));
    joints_pub.data.push_back((angles::to_degrees(joint_position_command_[1])));
    joints_pub.data.push_back((angles::to_degrees(joint_position_command_[2])));
    joints_pub.data.push_back((angles::to_degrees(joint_position_command_[3])));
    joints_pub.data.push_back((angles::to_degrees(joint_position_command_[4])));
    //ROS_INFO("Publishing j1: %.2f, j2: %.2f, j3: %.2f",joints_pub.data[0],joints_pub.data[1],joints_pub.data[2]);
    pub.publish(joints_pub);    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    //ROBOTHardwareInterface arm_mesh;
    
    ros::AsyncSpinner spinner(2); 
    ROBOTHardwareInterface ROBOT(nh);// 2 threads for controller service and for the Service client used to get the feedback from ardiuno
    spinner.start();

    //controller_manager::ControllerManager cm(&ROBOT,nh);
   // ros::spin();
    ros::waitForShutdown();
    return 0;
}
