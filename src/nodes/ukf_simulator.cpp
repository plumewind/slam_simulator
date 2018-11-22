#include "slam_simulator/ukf_console.h"

int main(int argc, char **argv)
{
	// Set up ROS node
	ros::init(argc, argv, "ukf_simulator");
	slam_simulator::ukf_console console(false);//Declaring the key class
	
	ROS_INFO("The ukf_simulator is start !");
	console.Initialize();				//Post the corresponding topic
	
 	console.run();					//the main loop

	return 0;
}  // end main()



