/*
 * $ rosrun node_monitor_nx node_monitor_nx_node verbose
 */

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <time.h>		/* ROS time functions */

//variables
int minion_controller_last_counter = 0;
int pick_place_last_counter = 0;
int my_node_detectnet_last_counter = 0;
int fiducial_service_last_counter = 0;
int finder_service_last_counter = 0;
bool minion_controller_good = false;
bool pick_place_good = false;
bool my_node_detectnet_good = false;
bool fiducial_service_good = false;
bool finder_service_good = false;
bool verbose = false;

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

void pickPlaceRecdCallback(const std_msgs::Int16& pick_place_msg_){
	if (pick_place_msg_.data > pick_place_last_counter) {
		pick_place_good = true;
		pick_place_last_counter = pick_place_msg_.data;
	} else {
		pick_place_good = false;
	}
}//end pickPlaceRecdCallback()

void myNodeDetectnetRecdCallback(const std_msgs::Int16& my_node_detectnet_msg_){
	if (my_node_detectnet_msg_.data > my_node_detectnet_last_counter) {
		my_node_detectnet_good = true;
		my_node_detectnet_last_counter = my_node_detectnet_msg_.data;
	} else {
		my_node_detectnet_good = false;
	}
}//end myNodeDetectnetRecdCallback()

void fiducialServiceRecdCallback(const std_msgs::Int16& fiducial_service_msg_){
	if (fiducial_service_msg_.data > fiducial_service_last_counter) {
		fiducial_service_good = true;
		fiducial_service_last_counter = fiducial_service_msg_.data;
	} else {
		fiducial_service_good = false;
	}
}//end fiducialServiceRecdCallback()

void finderServiceRecdCallback(const std_msgs::Int16& finder_service_msg_){
	if (finder_service_msg_.data > finder_service_last_counter) {
		finder_service_good = true;
		finder_service_last_counter = finder_service_msg_.data;
	} else {
		finder_service_good = false;
	}
}//end finderServiceRecdCallback()

int main(int argc, char* argv[]){	
	//vars
	char verbose_cmd[] = "verbose";   //verbose command line arg
	
	//init
	ros::init(argc, argv, "node_monitor_nx_node");
	ros::NodeHandle nh;

	if (argc > 1 && strcmp(argv[1], verbose_cmd) == 0){
		verbose = true;
		ROS_INFO("verbose=true");
	} else {
		ROS_INFO("verbose=false");
	}		
	
	ros::AsyncSpinner spinner(1);	//one thread
	spinner.start();
	ros::Rate r(0.33333);	//loop rate in Hertz 0.3333 means loop attempt to run once every 3 seconds
	
	//subscribers
	ros::Subscriber minion_controller_sub = nh.subscribe("minion_controller_watchdog", 10, minionControllerRecdCallback);
	ros::Subscriber pick_place_sub = nh.subscribe("pick_place_watchdog", 10, pickPlaceRecdCallback);
	ros::Subscriber my_node_detectnet_sub = nh.subscribe("my_node_detectnet_watchdog", 10, myNodeDetectnetRecdCallback);
	ros::Subscriber fiducial_service_sub = nh.subscribe("fiducial_service_watchdog", 10, fiducialServiceRecdCallback);
	ros::Subscriber finder_service_sub = nh.subscribe("finder_service_watchdog", 10, finderServiceRecdCallback);

	while (nh.ok()){
		//check to see if nodes are good
		
		//print current time so I know this node is working
		ros::Time time_now = ros::Time::now();
		std::cout << ("%T", time_now) << std::endl;
		
		//if (minion_controller_good && pick_place_good) {
			//std::cout << "all good" << std::endl;
		//}
		if (minion_controller_good) {
			std::cout << "minion_controller GOOD" << std::endl;
		} else {
			std::cout << "minion_controller       FAIL" << std::endl;
		}
				
		if (pick_place_good) {
			std::cout << "pick_place        GOOD" << std::endl;
		} else {
			std::cout << "pick_place              FAIL" << std::endl;
		}
					
		if (my_node_detectnet_good) {
			std::cout << "my_node_detectnet GOOD" << std::endl;
		} else {
			std::cout << "my_node_detectnet       FAIL" << std::endl;
		}
				
		if (fiducial_service_good) {
			std::cout << "fiducial_service  GOOD" << std::endl;
		} else {
			std::cout << "fiducial_service        FAIL" << std::endl;
		}
					
		if (finder_service_good) {
			std::cout << "finder_service    GOOD" << std::endl;
		} else {
			std::cout << "finder_service          FAIL" << std::endl;
		}			
		
		//ouput a blank line for clearer formating
		std::cout << " " << std::endl;
			
		//spin and sleep
		ros::spinOnce();
		r.sleep();
	}// end while()
	
	return 0;	
}//end main()
