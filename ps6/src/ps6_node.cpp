//example ROS client:
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client



#include <ros/ros.h>
#include <std_srvs/Trigger.h> // this message type is defined in the current package
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/DroneControl.h>
#include <iostream>
#include <string>

using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam_data;

void camCB(const osrf_gear::LogicalCameraImage& message_holder){
	if(g_take_new_snapshot){
		ROS_INFO_STREAM("Camera Image: " <<message_holder<<endl);
		g_cam_data = message_holder;
		g_take_new_snapshot = false;
	
	}
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    
    ros::Subscriber cam_subscriber = n.subscribe("/ariac/logical_camera_2", 1, camCB);
	
    // Do startup for system
    startup_srv.response.success = false;
    while (!startup_srv.response.success) {
        ROS_WARN("not successful starting");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successful startup response");
    
    // Define the client and service for the conveyor
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;

    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful conveyor");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box");
    
    ROS_INFO("got success from conveyor");



	//take a snapshot to figure out whereabouts of box
	g_take_new_snapshot = true;

	//keep going until you see a box
	while (g_cam_data.models.size() < 1) {
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		g_take_new_snapshot = true;
	}
	ROS_INFO("Box visible to camera");

	// wait until the pos of the boz is under the camera 
	
	// g_cam_data.models[0].pose.position.z > -0.1 && g_cam_data.models[0].pose.position.z < 0.1
	
	while (abs(g_cam_data.models[0].pose.position.z) > 0.1) {
		ROS_INFO("Waiting for box to be under camera: %f", g_cam_data.models[0].pose.position.z);	
		ros::spinOnce();
		g_take_new_snapshot = true;
		ros::Duration(0.5).sleep();
	}

	ROS_INFO("box below camera");

	// stop the belt w/ box
	conveyor_srv.request.power = 0.0; 
	conveyor_srv.response.success = false;
	while (!conveyor_srv.response.success) {
		ROS_WARN("not successful stop");
		conveyor_client.call(conveyor_srv);
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Successful stop, waiting for 5 seconds");
	// stop conveyor, wait for 5 secs w/ box under camera
	ros::Duration(5.0).sleep(); 

	//restart the conveyor
	conveyor_srv.request.power = 100.0;
	conveyor_srv.response.success = false;
	while (!conveyor_srv.response.success) {
		ROS_WARN("not successful conveyor");
		conveyor_client.call(conveyor_srv);
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Successful start, waiting for box to be at end stop");

	// No longer need to check box
	g_take_new_snapshot = false;
    
    // Create the drone client service
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;
    
    // Wait here to call drone until box has (estimated) reached the final spot
	ros::Duration(13.0).sleep();
    // Call drone to take away box
    drone_srv.request.shipment_type = "order_0_shipment_0";
    drone_srv.response.success = false;
    while (!drone_srv.response.success) {
        ROS_WARN("not successful drone");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Drone successfully called");
}

    
