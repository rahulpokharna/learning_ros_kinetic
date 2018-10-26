// example_action_server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<minimal_nodes/minimalAction.h>
#include<cmath>
#include <std_msgs/Float64.h>

//int numCycles = 0;
bool done = false;
int g_count = 0;
bool g_count_failure = false;

class SinActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<minimal_nodes::minimalAction> as_;
    
    // here are some message types to communicate with our client(s)
    minimal_nodes::minimalGoal goal_; // goal message, received from client
    minimal_nodes::minimalResult result_; // put results here, to be sent back to the client when done w/ goal
    minimal_nodes::minimalFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;
    int completedCycles;
    int totalCycles;
    int numCycles;
    double frequency;
    double amplitude;


public:
    SinActionServer(); //define the body of the constructor outside of class definition

    ~SinActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<minimal_nodes::minimalAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

SinActionServer::SinActionServer() :
   as_(nh_, "timer_action", boost::bind(&SinActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void SinActionServer::executeCB(const actionlib::SimpleActionServer<minimal_nodes::minimalAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    frequency = goal->frequency;
    amplitude = goal->amplitude;
    numCycles = goal->numCycles;
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    completedCycles = 0;
    ros::Rate timer(1.0); // 1Hz timer
    //countdown_val_ = goal->input;
    std_msgs::Float64 vel_out;
    double omega = frequency * 2 * M_PI;
    double dt = 0.1;
    double current_pos = 0;
    double sample_rate = 1 / dt;
    ros::Rate naptime(sample_rate);
    long numIters = 0;
    ros::NodeHandle nh; 
    ros::Publisher my_publisher = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    while (ros::ok()&&completedCycles<numCycles) {
       //ROS_INFO("countdown = %d",countdown_val_);
       
       // each iteration, check if cancellation has been ordered
       if (as_.isPreemptRequested()){	
          ROS_WARN("goal cancelled!");
          //result_.output = countdown_val_;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
 	}
	
	current_pos += dt;
	if(current_pos >= 2 * M_PI) {
	    current_pos -= 2 * M_PI;
	    completedCycles++;
	}

	vel_out.data = omega * amplitude * cos(current_pos);
	my_publisher.publish(vel_out);

	// Values below to show accruate representations of time, converted to int.	
	numIters++;
	feedback_.fdbk = numIters * sample_rate;
 	   //if here, then goal is still valid; provide some feedback
 	//feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
 	as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
	//countdown_val_--; //decrement the timer countdown


	naptime.sleep(); //wait 1 sec between loop iterations of this timer
    }
    vel_out.data = 0;
    my_publisher.publish(vel_out);
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.done = true; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sin_action_commander"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    SinActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

