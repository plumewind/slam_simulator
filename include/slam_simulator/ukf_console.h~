#ifndef SLAM_SIMULATOR_UKF_CONSOLE_H
#define SLAM_SIMULATOR_UKF_CONSOLE_H

#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "slam_simulator/unscented_kf.h"
#include "slam_simulator/tool.h"

namespace slam_simulator
{

	class ukf_console
	{
	public:
		ukf_console(bool debug = false);
		~ukf_console();
		
		bool start();
		bool getLandmarks(const std::string& path);
		bool getMeasurements(const std::string& path);
		
		void LandmarksPublish();
		void EstimatePosePublish(const Eigen::Vector3d robot_pose);
		void AssociationPublish(const std::vector<Eigen::Vector3f>& radars);
		void UncertaintyEllipseShow();
		
		void run();
		
	private:
		ros::NodeHandle ukf_Handle;
		ros::Publisher Landmarks_pub, Assoc_pub;
		ros::Publisher Estpose_pub, Uncertainty_pub;
		
		bool Debug;
		
		std::string Sensordata_filepath;
		std::string Worlddata_filepath;
		std::vector<Eigen::Vector3f> Landmarks;			//路标点集, 点的id. x坐标, y坐标组成一个点信息组
		std::vector<Record> Measurements;
		
		float marker_scale;
		
		unscented_kf* ukf_tool;
		
		
	};


}
#endif
