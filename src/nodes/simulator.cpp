#include "slam_simulator/ukf_console.h"
#include "slam_simulator/ekf_console.h"

int main(int argc, char **argv)
{
	// Set up ROS node
	ros::init(argc, argv, "simulator");
	ros::NodeHandle handle;

	std::string controller;
	handle.getParam("/simulator/controller", controller);
	
	ROS_INFO("The simulator is start...... !");
	if(controller == "ukf")
	{
		slam_simulator::ukf_console console;
		console.start();				//Post the corresponding topic
		console.run();					//the main loop
	}else if(controller == "ekf")
	{
		slam_simulator::ekf_console console;
		console.start();				//Post the corresponding topic
		console.run();					//the main loop
		
	}

	return 0;
}  // end main()



