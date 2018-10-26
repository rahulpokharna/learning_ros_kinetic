// example_action_client: 
// wsn, October, 2014

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream> 

//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../example_action_server/action/demo.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (demo) and appended name (Action)
// If you write a new client of the server in this package, you will need to include example_action_server in your package.xml,
// and include the header file below
//#include<example_action_server/demoAction.h>
#include<minimal_nodes/minimalAction.h>
using namespace std;

bool g_goal_active = false; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;
int g_count = 0;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const minimal_nodes::minimalResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());

    //ROS_INFO("got result output = %d", result->output);
    //g_result_output= result->output;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const minimal_nodes::minimalFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status = %d",fdbk_msg->fdbk);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client"); // name this node 
    ros::NodeHandle n;
    ros::Rate main_timer(1.0);

    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    minimal_nodes::minimalGoal goal;

    // use the name of our server, which is: example_action (named in example_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)

    // SERVER NOT NAMED YET, UNCOMMENT WHEN CREATED
    actionlib::SimpleActionClient<minimal_nodes::minimalAction> action_client("timer_action", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    while (!server_exists) {
        ROS_WARN("could not connect to server; halting");
	server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        //return 0; // bail out; optionally, could print a warning message and retry
    }


    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    double amplitude = 0;
    double frequency = 0;
    int numCycles = 0;

    while (ros::ok()) {

        cout<<endl;
        
        double frequency;
        ROS_INFO("Enter frequency: -1 to quit");
        std::cin >> frequency;
    
        
        ROS_INFO("Enter amplitude, -1 to quit: ");
        std::cin >> amplitude;

        int numCycles;
        ROS_INFO("Enter number of cycles, -1 to quit, 0 to cancel goal: ");
        std::cin >> numCycles;

	if(amplitude == -1 || frequency == -1 || numCycles == -1){
	    ROS_INFO("Exiting Client...");
	    return 0;
	}

	if(numCycles == 0){
	    ROS_INFO("Cancelling Goal");
	    action_client.cancelGoal();
	}

	goal.frequency = frequency;
	goal.amplitude = amplitude;
	goal.numCycles = numCycles;
	
        //action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        //bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            //ROS_WARN("giving up waiting on result for goal number %d", g_count);
            return 0;
        } else {
            //if here, then server returned a result to us
        }

    }

    return 0;
}

