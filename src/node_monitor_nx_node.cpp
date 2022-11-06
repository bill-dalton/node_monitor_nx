/*
 * Functionality
 * 	runs in its own terminal window, started as follows
 * 		$ rosrun node_monitor_nx node_monitor_nx_node verbose
 * 	Monitors my ROS nodes and my minions for failures
 * 	runs as ros::rate of 0.333 so it published every 3 seconds
 *  Indicates 'GOOD' or 'FAIL' for each
 * 
 * Nodes
 * 	Each of the following nodes periodically publishes an ever increasing Int16 counter.
 *  As long as the counter keeps increasing the node is considered good:
 * 		minion_controller
 * 		pick_place
 * 		my_node_detectnet
 * 		fiducial_service
 * 		finder_service
 * 
 * Minions
 * 	The minion messages are simply monitored to see that they are still sending. No counter is used
 * 		BR_minion
 * 		SL_minion
 * 		EL_minion
 * 		LR_minion
 * 		WL_minion
 * 		WR_minion
 * 		finger_012_minion
 * 		finger_345_minion
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <time.h>		/* ROS time functions */
#include <geometry_msgs/Vector3.h>
#include <my_robotic_arm_nx/MinionState.h>

//variables
//************ VARIABLES ****************//
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
bool BR_minion_good = false;
bool SL_minion_good = false;
//bool UR_minion_good = false;
bool EL_minion_good = false;
bool LR_minion_good = false;
bool WL_minion_good = false;
bool WR_minion_good = false;
bool gripper_dist_cam_good = false;
bool gripper_pos_force_good = false;
bool orientation_hand_good = false;
bool verbose = false;

//subscribers
//ros::Subscriber minion_controller_sub;

//************ NODE MONITORS ****************//
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

//************ MINION STATE MONITORS ****************//
//receiving a message from a minion shows that minion is still operating
void BRMinionStateRecdCallback(const my_robotic_arm_nx::MinionState& minion_state_msg_){
	BR_minion_good = true;
}
void SLMinionStateRecdCallback(const my_robotic_arm_nx::MinionState& minion_state_msg_){
	SL_minion_good = true;
}
//void URMinionStateRecdCallback(const my_robotic_arm_nx::MinionState& minion_state_msg_){
	//UR_minion_good = true;
//}
void ELMinionStateRecdCallback(const my_robotic_arm_nx::MinionState& minion_state_msg_){
	EL_minion_good = true;
}
void LRMinionStateRecdCallback(const my_robotic_arm_nx::MinionState& minion_state_msg_){
	LR_minion_good = true;
}
void WLMinionStateRecdCallback(const geometry_msgs::Vector3& minion_Vector3_state_msg_){
	WL_minion_good = true;
}
void WRMinionStateRecdCallback(const geometry_msgs::Vector3& minion_Vector3_state_msg_){
	WR_minion_good = true;
}
void GripperDistCamRecdCallback(const geometry_msgs::Vector3& msg_){
	gripper_dist_cam_good = true;
}
void GripperPosForceRecdCallback(const geometry_msgs::Vector3& msg_){
	gripper_pos_force_good = true;
}
void OrientationHandRecdCallback(const geometry_msgs::Vector3& msg_){
	orientation_hand_good = true;
}

//************ MAIN ****************//
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
	ros::Subscriber minion_controller_sub = nh.subscribe("minion_controller_watchdog", 1, minionControllerRecdCallback);
	ros::Subscriber pick_place_sub = nh.subscribe("pick_place_watchdog", 1, pickPlaceRecdCallback);
	ros::Subscriber my_node_detectnet_sub = nh.subscribe("my_node_detectnet_watchdog", 1, myNodeDetectnetRecdCallback);
	ros::Subscriber fiducial_service_sub = nh.subscribe("fiducial_service_watchdog", 1, fiducialServiceRecdCallback);
	ros::Subscriber finder_service_sub = nh.subscribe("finder_service_watchdog", 1, finderServiceRecdCallback);
	ros::Subscriber BR_minion_state_sub = nh.subscribe("BR_minion_state", 1000, BRMinionStateRecdCallback);
	ros::Subscriber SL_minion_state_sub = nh.subscribe("SL_minion_state", 1000, SLMinionStateRecdCallback);
	//ros::Subscriber UR_minion_state_sub = nh.subscribe("UR_minion_state", 1000, URMinionStateRecdCallback);
	ros::Subscriber EL_minion_state_sub = nh.subscribe("EL_minion_state", 1000, ELMinionStateRecdCallback);
	ros::Subscriber LR_minion_state_sub = nh.subscribe("LR_minion_state", 1000, LRMinionStateRecdCallback);
	ros::Subscriber WL_minion_state_sub = nh.subscribe("WL_joint_state", 1000, WLMinionStateRecdCallback);//note xx_joint_state for Dynamixel versions versus xx_minion_state for Teensy versions
	ros::Subscriber WR_minion_state_sub = nh.subscribe("WR_joint_state", 1000, WRMinionStateRecdCallback);
	ros::Subscriber gripper_dist_cam_sub = nh.subscribe("gripper_dist_cam", 1000, GripperDistCamRecdCallback);
	ros::Subscriber gripper_pos_force_sub = nh.subscribe("gripper_pos_force", 1000, GripperPosForceRecdCallback);
	ros::Subscriber orientation_hand_sub = nh.subscribe("orientation_hand", 1000, OrientationHandRecdCallback);

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
	//******	
		if (BR_minion_good) {
			std::cout << "BR_minion	  GOOD" << std::endl;
		} else {
			std::cout << "BR_minion	        FAIL" << std::endl;
		}			
		
		if (SL_minion_good) {
			std::cout << "SL_minion	  GOOD" << std::endl;
		} else {
			std::cout << "SL_minion	        FAIL" << std::endl;
		}			
		
		//if (UR_minion_good) {
			//std::cout << "UR_minion	  GOOD" << std::endl;
		//} else {
			//std::cout << "UR_minion	        FAIL" << std::endl;
		//}			
		
		if (EL_minion_good) {
			std::cout << "EL_minion	  GOOD" << std::endl;
		} else {
			std::cout << "EL_minion	        FAIL" << std::endl;
		}			
		
		if (LR_minion_good) {
			std::cout << "LR_minion	  GOOD" << std::endl;
		} else {
			std::cout << "LR_minion	        FAIL" << std::endl;
		}			
		
		if (WL_minion_good) {
			std::cout << "WL_minion	  GOOD" << std::endl;
		} else {
			std::cout << "WL_minion	        FAIL" << std::endl;
		}			
		
		if (WR_minion_good) {
			std::cout << "WR_minion	  GOOD" << std::endl;
		} else {
			std::cout << "WR_minion	        FAIL" << std::endl;
		}			
		
		if (gripper_dist_cam_good) {
			std::cout << "gripper_dist_cam  GOOD" << std::endl;
		} else {
			std::cout << "gripper_dist_cam 		FAIL" << std::endl;
		}			
		
		if (gripper_pos_force_good) {
			std::cout << "gripper_pos_force GOOD" << std::endl;
		} else {
			std::cout << "gripper_pos_force 	FAIL" << std::endl;
		}			

		if (orientation_hand_good) {
			std::cout << "orientation_hand  GOOD" << std::endl;
		} else {
			std::cout << "orientation_hand 		FAIL" << std::endl;
		}			
		
		//ouput a blank line for clearer formating
		std::cout << " " << std::endl;
			
		//set all flags to fail so that inactivity on a topic will cause a FAIL message
		minion_controller_good = false;
		pick_place_good = false;
		my_node_detectnet_good = false;
		fiducial_service_good = false;
		finder_service_good = false;
		BR_minion_good = false;
		SL_minion_good = false;
		//UR_minion_good = false;
		EL_minion_good = false;
		LR_minion_good = false;
		WL_minion_good = false;
		WR_minion_good = false;
		gripper_dist_cam_good = false;
		gripper_pos_force_good = false;
		orientation_hand_good = false;
		
		//spin and sleep
		ros::spinOnce();
		r.sleep();
	}// end while()
	
	return 0;	
}//end main()
