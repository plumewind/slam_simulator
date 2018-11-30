#include "slam_simulator/ukf_console.h"

namespace slam_simulator
{
	ukf_console::ukf_console(bool debug)
		:ukf_Handle("~"), Debug(debug)
	{
		ukf_Handle.getParam("sensor_data_filepath", Sensordata_filepath);
		ukf_Handle.getParam("world_data_filepath", Worlddata_filepath);
		ukf_Handle.getParam("marker_scale", marker_scale);
		
	}
	ukf_console::~ukf_console()
	{
		delete ukf_tool;
	}
	bool ukf_console::start()
	{
		Landmarks_pub = ukf_Handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/landmarks", 1);
		Assoc_pub = ukf_Handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/associations", 1);
		Estpose_pub = ukf_Handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/estimate_pose", 5); 
		Uncertainty_pub = ukf_Handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/uncertainty", 5); 
		
		// 确保发布标记时，已经打开rviz进行显示

		while (Landmarks_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())	
				return  false;
			ROS_WARN_ONCE("Please create a subscriber to the marker or launch rviz ! ");
			sleep(1);
		}
		
		return true;
	}
	void ukf_console::run()
	{
		//read the map data for all landmarks
		getLandmarks(Worlddata_filepath);

		//read the measurements with odometry and radar data
		getMeasurements(Sensordata_filepath);
		//std::cout <<"已经读入测量数据: "<< Measurements.size() << std::endl;
		
		LandmarksPublish();
		
		ukf_tool = new unscented_kf(Landmarks.size());
		
		ros::Rate rat(10);
		for (const auto& record : Measurements) 
		{ 
			if (!ros::ok())	
				break;
			
			ukf_tool->ProcessMeasurement(record);
			EstimatePosePublish((ukf_tool->Mu_x).head(3));
			AssociationPublish(record.radar);
			UncertaintyEllipseShow();
			
			rat.sleep();
		}

	}
	bool ukf_console::getLandmarks(const std::string& path)
	{
		std::ifstream in_file(path, std::ifstream::in);
		if (!in_file.is_open()) 
		{
			std::cerr << "Cannot open input file: " << (path) << std::endl;
			exit(EXIT_FAILURE);
		}

		std::string line;
		while(std::getline(in_file, line)) 
		{
			std::istringstream ss(line);
			Eigen::Vector3f mp;
			ss>>mp(0);
			ss>>mp(1);
			ss>>mp(2);
			Landmarks.push_back(mp);
			if (Debug)
				std::cout << (Landmarks.back())(0) << ": " << (Landmarks.back())(1)  << ": " << (Landmarks.back())(2)  << std::endl;
		}

		if (in_file.is_open()) 
			in_file.close();
		
	}
	bool ukf_console::getMeasurements(const std::string& path)
	{
		std::ifstream in_file(path, std::ifstream::in);
		if (!in_file.is_open()) {
			std::cerr << "Cannot open input file: " << path << std::endl;
			exit(EXIT_FAILURE);
		}

		std::string line;
		Record record;
		int index = 0;
		while(getline(in_file, line)) 
		{
			std::string sensor_type;
			std::istringstream ss(line);
			ss >> sensor_type;
			//measurement type r1 t r2
			if (sensor_type.compare("ODOMETRY") == 0) 
			{//end the first record;
				if (record.radar.size() != 0) 
				{
					Measurements.push_back(record);
					record.radar.clear();
					if (Debug && index < 50)
						std::cout << index << "-----------" << std::endl;
					index++;
				}
				auto& odo = record.odom;
				ss >> odo(0);
				ss >> odo(1);
				ss >> odo(2);
				if (Debug && index < 50)
					std::cout << (record.odom)(0) << ": " << (record.odom)(1) << ": " << (record.odom)(2) << std::endl;
			} else if (sensor_type.compare("SENSOR") == 0) 
			{
				auto& radars = record.radar;
				Eigen::Vector3f radarR;
				ss >> radarR(0);
				ss >> radarR(1);
				ss >> radarR(2);
				radars.push_back(radarR);
				if (Debug && index < 50)
					std::cout << (radars.back())(0) << ": " << (radars.back())(1) << ": " << (radars.back())(2) << std::endl;
			}
		}
		if (record.radar.size() != 0) 
			Measurements.push_back(record);
		
		if (in_file.is_open()) 
			in_file.close();			
	}
	void ukf_console::LandmarksPublish()
	{
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker marker;
		// 设置该标记的命名空间和ID，ID应该是独一无二的
		// 具有相同命名空间和ID的标记将会覆盖前一个
		marker.header.frame_id = "map";
		marker.ns = "2Dlandmarks";
		// 设置标记行为：ADD（添 加），DELETE（删 除）
		marker.action = visualization_msgs::Marker::ADD;
		// 设置标记类型，初始值为球体。在立方体、球体、箭头和 圆柱体之间循环
		marker.type = visualization_msgs::Marker::CUBE;// 设置初始形状为圆柱体
		marker.id = 0;
		
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// 设置标记的比例，所有方向上尺度1表示1米
		marker.scale.x = marker_scale;
		marker.scale.y = marker_scale;
		marker.scale.z = marker_scale;
		//设置标记颜色，确保alpha（不透明度）值不为0, 紫色
		marker.color.r = 160.0/255.0;
		marker.color.g = 32.0/255.0;
		marker.color.b = 240.0/255.0;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0);
		
		int markers_size = Landmarks.size();
		for(int i=0 ; i < markers_size ; i++)
		{
			// 设置帧 ID和时间戳
			marker.header.stamp = ros::Time::now();
			marker.id++;
			//设置标记位姿。 
			marker.pose.position.x = (Landmarks[i])(1);
			marker.pose.position.y = (Landmarks[i])(2);
			ma.markers.push_back(marker);
		}
		Landmarks_pub.publish(ma);
	}
	void ukf_console::EstimatePosePublish(const Eigen::Vector3d robot_pose)
	{
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker marker;
		
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "robot_pose";

		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0.1);
		
		//设置标记位姿。 
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// 设置标记的比例，所有方向上尺度1表示1米
		marker.scale.x = marker_scale;
		marker.scale.y = marker_scale;
		marker.scale.z = marker_scale;
		//设置标记颜色，确保alpha（不透明度）值不为0
		marker.color.a = 1.0;
		
		// robot true pose
		marker.id = 0;
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 1.0;
		marker.pose.position.x = robot_pose(0);
		marker.pose.position.y = robot_pose(1);
		marker.pose.position.z = 2;
		ma.markers.push_back(marker);

		Estpose_pub.publish(ma);
	}
	void ukf_console::AssociationPublish(const std::vector< Eigen::Vector3f >& radars)
	{
		// visualization of the data association
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker line_strip;

		line_strip.header.frame_id = "map";
		line_strip.header.stamp = ros::Time::now();

		line_strip.id = 0;
		line_strip.ns = "data_association";
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		line_strip.action = visualization_msgs::Marker::ADD;

		line_strip.lifetime = ros::Duration(0.1);
		line_strip.pose.position.x = 0;
		line_strip.pose.position.y = 0;
		line_strip.pose.position.z = 0;
		line_strip.pose.orientation.x = 0.0;
		line_strip.pose.orientation.y = 0.0;
		line_strip.pose.orientation.z = 0.0;
		line_strip.pose.orientation.w = 1.0;
		line_strip.scale.x = marker_scale*0.3;  // line uses only x component
		line_strip.scale.y = marker_scale*0.3;
		line_strip.scale.z = marker_scale*0.3;
		line_strip.color.a = 1.0;
		line_strip.color.r = 255/255;
		line_strip.color.g = 0;
		line_strip.color.b = 0;

		// robot pose		
		geometry_msgs::Point pointRobotPose;
		pointRobotPose.x = (ukf_tool->Mu_x)(0);
		pointRobotPose.y = (ukf_tool->Mu_x)(1);
		pointRobotPose.z = 0;
		
		//draw observation lines
		int visible_num = radars.size();
		for(int i = 0; i < visible_num; i++) 
		{
			int index = 0;
			for ( ; index < ukf_tool->LandmarkINmap.size() ; index++) 
			{
				if(ukf_tool->LandmarkINmap[index] == (int)(radars[i])(0)) 
					break;
			}
		
			line_strip.points.clear();
			line_strip.points.push_back(pointRobotPose);
			
			geometry_msgs::Point pointLm;
			pointLm.x = ukf_tool->Mu_x(2 * index + 3);
			pointLm.y = ukf_tool->Mu_x(2 * index + 4);
			pointLm.z = 0.0;
			line_strip.points.push_back(pointLm);
			ma.markers.push_back(line_strip);
			line_strip.id++;
		}   

		Assoc_pub.publish(ma);
	}
	void ukf_console::UncertaintyEllipseShow()
	{
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.id = 0;
		marker.ns = "ekf_predict";
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0.5);
		
		marker.pose.position.z = 0.1;
		marker.color.r = 135/255.0;
		marker.color.g = 206/255.0;
		marker.color.b = 235/255.0;
		marker.color.a = 0.8;//设置标记颜色，确保alpha（不透明度）值为0
		marker.scale.x = marker_scale;
		marker.scale.y = marker_scale;
		marker.scale.z = marker_scale;
		
		// covariance ellipses
		tf::Quaternion orientation;
		tf::Matrix3x3 tf3d;
		Eigen::Matrix2d eigenvectors;
		Eigen::Vector2d eigenvalues;
		Eigen::Matrix2d covariance;// get the covariance matrix 2x2 for each ellipsoid including robot pose
		
		// Count the number of landmarks + robot pose
		unsigned int objs_counter = ((ukf_tool->Mu_x.rows())-3 )/2 + 1;
		unsigned int pose_id=0;
		double quantiles=1 ;
		double ellipse_scale_=5;
		for (size_t i = 0; i < objs_counter; i++)
		{
			marker.id++;
			if( i != 0)
				pose_id = i * 2 + 1;
			
			covariance=ukf_tool->Sigma_y.block(pose_id, pose_id, 2, 2);
			if(tool::computeEllipseOrientationScale2D(eigenvectors, eigenvalues, covariance) == false)
				continue;
			
			// Rotation matrix around  z axis
			tf3d.setValue(eigenvectors(0, 0), eigenvectors(0, 1), 0, eigenvectors(1, 0), eigenvectors(1, 1), 0, 0, 0, 1);

			// get orientation from rotation matrix
			tf3d.getRotation(orientation);
			
			marker.pose.position.x = (ukf_tool->Mu_x)(pose_id);
			marker.pose.position.y = (ukf_tool->Mu_x)(pose_id+1);
			marker.pose.position.z = 0;
			marker.pose.orientation.x = orientation.x();
			marker.pose.orientation.y = orientation.y();
			marker.pose.orientation.z = orientation.z();
			marker.pose.orientation.w = orientation.w();
			marker.scale.x = ellipse_scale_ * quantiles * sqrt(eigenvalues(0));
			marker.scale.y = ellipse_scale_ * quantiles * sqrt(eigenvalues(1));
			marker.scale.z = 0.00001;  // Z can't be 0, limitation of ROS
			ma.markers.push_back(marker);
		}
		
		Uncertainty_pub.publish(ma);
	}


	
	
	
	
	


}
