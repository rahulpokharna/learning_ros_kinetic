// sin controller node
// rkp43 Rahul pokharna 
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<iostream> 
#include<cmath> 
#include<minimal_nodes/service_message.h>

double frequency = -1;
double amplitude = -1;

bool callback(minimal_nodes::service_messageRequest& request, minimal_nodes::service_messageResponse& response)
{
    ROS_INFO("callback activated");
    //string in_name(request.name); //let's convert this to a C++-class string, so can use member funcs
    //cout<<"in_name:"<<in_name<<endl;
    //response.done=false;
    
    // here is a dumb way to access a stupid database...
    // hey: this example is about services, not databases!
    frequency = request.frequency;
    amplitude = request.amplitude;
    response.done = true;
  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_controller"); //name this node 

    ros::NodeHandle nh; // node handle 

    //publish a force command computed by this controller; 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);
	
    //double frequency;
    //ROS_INFO("Enter frequency: ");
    //std::cin >> frequency;

    //double amplitude;
    //ROS_INFO("Enter amplitude: ");
    //std::cin >> amplitude;

    ros::ServiceServer service = nh.advertiseService("lookup_by_name", callback);
    ROS_INFO("Ready to look up names.");
    if(amplitude == frequency && amplitude == -1){
	ros::spinOnce();
    }

    std_msgs::Float64 vel_out;


    double dt = 0.1;
    double current_pos = 0;
    double sample_rate = 1 / dt;
    ros::Rate naptime(sample_rate);

    while (ros::ok()) {
	if(amplitude == frequency && amplitude == -1){
	    ros::spinOnce();
	}else{
	    double omega = frequency * 2 * M_PI;
            current_pos += dt;
	    if(current_pos > 2 * M_PI)
	        current_pos -= 2 * M_PI;
	    vel_out.data = omega * amplitude * cos(current_pos);
	    my_publisher_object.publish(vel_out);
	    ros::spinOnce();
            naptime.sleep(); // wait for remainder of specified period; 
	}
    }
    return 0; // should never get here, unless roscore dies 
}
