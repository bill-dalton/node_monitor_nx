/*
 * $ rosrun node_monitor_nx node_monitor_nx_node
 */

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <time.h>		/* ROS time functions */

//variables
int minion_controller_last_counter = 0;
int pick_place_last_counter = 0;
bool minion_controller_good = false;
bool pick_place_good = false;
bool debug = false;

//subscribers
ros::Subscriber minion_controller_sub;

void minionControllerRecdCallback(const std_msgs::Int16& minion_controller_msg_){
	if (minion_controller_msg_.data > minion_controller_last_counter) {
		minion_controller_good = true;
		minion_controller_last_counter = minion_controller_msg_.data;
	} else {
		minion_controller_good = false;
	}
}//end minionControllerRecdCallback()

int main(int argc, char* argv[]){	
	//vars
	char debug_cmd[] = "debug";   //debug command line arg
	
	//init
	ros::init(argc, argv, "node_monitor_nx_node");
	ros::NodeHandle nh;

	if (argc > 1 && strcmp(argv[1], debug_cmd) == 0){
		debug = true;
		ROS_INFO("debug=true");
	}
	
	ros::AsyncSpinner spinner(1);	//one thread
	spinner.start();
	ros::Rate r(3.0);
	
	//subscribers
	ros::Subscriber minion_controller_sub = nh.subscribe("minion_controller_watchdog", 10, minionControllerRecdCallback);

	while (nh.ok()){
		//check to see if nodes are good
		
		//print current time so I know this node is working
		ros::Time time_now = ros::Time::now();
		std::cout << ("%T", time_now) << std::endl;
		
		if (minion_controller_good && pick_place_good) {
			std::cout << "all good" << std::endl;
		}
		if (!minion_controller_good) {
			std::cout << "minion_controller FAIL" << std::endl;
		}
		if (!minion_controller_good) {
			std::cout << "pick_place FAIL" << std::endl;
		}		
				
		//spin and sleep
		ros::spinOnce();
		r.sleep();
	}// end while()
	
	return 0;	
}//end main()
