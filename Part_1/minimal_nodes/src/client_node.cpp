//example ROS client:
// first run: rosrun minimal_nodes sin_commander
// then start this node:  rosrun minimal_nodes client_node



#include <ros/ros.h>
#include <minimal_nodes/service_message.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<minimal_nodes::service_message>("lookup_by_name");
    minimal_nodes::service_message srv;
    bool done = false;
    string in_name;
    while (ros::ok()) {
        cout<<endl;
        
        double frequency;
        ROS_INFO("Enter frequency: ");
        std::cin >> frequency;
    
        double amplitude;
        ROS_INFO("Enter amplitude, -1 to quit: ");
        std::cin >> amplitude;
       
	if(amplitude == -1){
	    return 0;
	}
        //cout<<"you entered "<<in_name<<endl;
        srv.request.amplitude = amplitude; //"Ted";
	srv.request.frequency = frequency;
        if (client.call(srv)) {
            if (srv.response.done) {
                cout << "Completed message, sent to sin controller"  << endl;
            } else {
                cout << "Service failed to complete" << endl;
            }

        } else {
            ROS_ERROR("Failed to call service lookup_by_name");
            return 1;
        }
    }
    return 0;
}
