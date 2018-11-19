// example_block_grabber: 
// wsn, Nov, 2018; 
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

//launch with roslaunch irb140_description irb140.launch, which places a block at x=0.5, y=0
// launch action server: rosrun irb140_planner irb140_cart_move_as
// then run this node

//for manual gripper control,  rosrun cwru_sticky_fingers finger_control_dummy_node /sticky_finger/link6 false

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_srvs/SetBool.h>
using namespace std;

// Position of block, has some default values
double pos_x = 0.5;
double pos_y = 0.0;
double pos_z = 0.035;
//double roll = 0.0, pitch = 0.0, yaw = 0.0;
// Attempt at using msg to store the pos of the block from the message
geometry_msgs::PoseStamped blockPose;
bool updatePos = true;

// Update value of the position, only if the flag is true, then set to false
void callBack(const geometry_msgs::PoseStamped& msg){
  if(updatePos){  
  	blockPose = msg;
	pos_x = msg.pose.position.x;
	pos_y = msg.pose.position.y;
	//roll = msg.pose.orientation.x;
	//pitch = msg.pose.orientation.y;
	//yaw = msg.pose.orientation.z;
	updatePos = false;
  }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    
    // Subscribe to the openCV node and receives coords that it is publishing
    // Command robot to grab the block
   
    
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/sticky_finger/link6");
    //Subscriber to the block finder 
    ros::Subscriber sub = nh.subscribe("block_pose",1,callBack);
    std_srvs::SetBool srv;
    srv.request.data = true;

    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    nsteps = 10;
    arrival_time = 2.0;



    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;

    //- Translation: [0.450, -0.000, 0.367]
	
    O_des << 0.5, 0.3, 0.3;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation() = O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    while (ros::ok()) {
        //move out of way for camera view:
        ROS_INFO("moving out of camera view");
        tool_pose.pose.position.y = -0.2; 
        tool_pose.pose.position.z = 0.3; //0.01;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }        
        
        //Make sure subscriber gets block centroid coordinates
        updatePos = true;
        for(int i = 0; i < 10; i++){
          ros::spinOnce();
          ros::Duration(0.5).sleep();
        }
        
        //Move arm to just above block
        ROS_INFO("moving to approach pose"); 
        tool_pose.pose.position.x = pos_x;
        tool_pose.pose.position.y = pos_y;
        // Put the tool piece in the appropriate orientation
        //tool_pose = blockPose; 
        tool_pose.pose.position.z = 0.05; //0.01;  
        // Here, set orientation of Z to what we get from the publisher
        //tool_pose.pose.orientation.x = roll;
        //tool_pose.pose.orientation.y = pitch;
        //tool_pose.pose.orientation.z = yaw;
        
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }           
        
		//Move arm to touch block
        ROS_INFO("moving to approach pose");
        tool_pose.pose.position.z = 0.0343; //0.01;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }           
        
        //enable the vacuum gripper:
        srv.request.data = true;
        ros::spinOnce();
        while (!client.call(srv) && ros::ok()) {
	        ROS_WARN("Inside the loop for the sucction");
            ROS_INFO("Sending command to gripper...");
            ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	        ROS_INFO("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");

            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Gripped object from vacuum");
        


        ROS_INFO("requesting plan to depart with grasped object:");
        tool_pose.pose.position.z = 0.3;
        
        // Reset the gripper values
        //tool_affine.linear() = R_gripper;
    	//tool_affine.translation() = O_des;   
        //tool_pose.pose.orientation.x = 0.0;
        //tool_pose.pose.orientation.y = 0.0;
        //tool_pose.pose.orientation.z = 0.0;
       
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	 	// Move block to specified coordinates of (xxx,xxx,xxx) 
        tool_pose.pose.position.x = 0.3;         
		tool_pose.pose.position.y = 0.3;         
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
        
        //Descend until block touches ground
        tool_pose.pose.position.z = 0.035;       
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
        
        //disable the vacuum gripper:

        srv.request.data = false;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        
        //Ascend to height of z = 0.1
        tool_pose.pose.position.z = 0.1;      
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
        ros::Duration(1).sleep();
        ROS_INFO("Moved the block to the appropriate position");
        updatePos = true; // reset the check, update value of the block 
        
    }

    return 0;
}

