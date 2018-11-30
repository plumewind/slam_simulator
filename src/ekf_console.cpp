#include "slam_simulator/ekf_console.h"

namespace slam_simulator
{

	ekf_console::ekf_console()
		:ekf_handle("~")
	{
		ekf_cal = new extend_kf();//类的实例化
		
		//获取路标点以及行驶点
		double dimension=1.0;
		XmlRpc::XmlRpcValue landmarks_xmlrpc;
		XmlRpc::XmlRpcValue waypoints_xmlrpc;
		//获取参数的值  
		ekf_handle.getParam("lm", landmarks_xmlrpc);
		ekf_handle.getParam("wp", waypoints_xmlrpc);
		
		if (landmarks_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			landmarks_size = landmarks_xmlrpc.size();
			std::cout<<"We obtained "<<landmarks_size<<" landmark points......"<<std::endl;
			landmarks.resize(2,  landmarks_size);
			landmarks.setZero(2,  landmarks_size);
			for(int i=0 ; i < landmarks_size ; i++)
			{
				XmlRpc::XmlRpcValue point = landmarks_xmlrpc[i];
				landmarks(0, i)=double(point[0])/dimension;
				landmarks(1, i)=double(point[1])/dimension;
			}
			ROS_INFO("The landmarks data is recorded !");
		}else ROS_ERROR("The dimension of landmarks arry is wrong !");
		
		if (waypoints_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			waypoints_size = waypoints_xmlrpc.size();
			std::cout<<"We obtained "<<waypoints_size<<" waypoints points......"<<std::endl;
			waypoints.resize(2,  waypoints_size);
			waypoints.setZero(2,  waypoints_size);
			for(int i=0 ; i < waypoints_size ; i++)
			{
				XmlRpc::XmlRpcValue point = waypoints_xmlrpc[i];
				waypoints(0, i)=double(point[0])/dimension;
				waypoints(1, i)=double(point[1])/dimension;
			}
			ROS_INFO("The waypoints data is recorded !");
		}else ROS_ERROR("The dimension of waypoints arry is wrong !");
		
		ekf_handle.getParam("DT_CONTROLS", dt);
		dtsum = 0;
		ekf_cal->da_table = new int[landmarks_size];
		memset(ekf_cal->da_table, 0, sizeof(ekf_cal->da_table));
		iwp= 0;//eigen矩阵也是从0开始，和MATLAB不同
		G= 0;
		dt_observe=8*dt;
		
		double sigmaV,sigmaG;
		ekf_handle.getParam("sigmaV", sigmaV);
		ekf_handle.getParam("sigmaG", sigmaG);
		Eigen::Matrix2d Q;
		Q(0,0)=sigmaV*sigmaV;
		Q(0,1)=Q(1,0)=0;//Q: [sigmaV^2 0; 0 sigmaG^2] 
		Q(1,1)=sigmaG*sigmaG;
		ekf_cal->Q=Q;
		ekf_cal->QE=Q;
		
		double sigmaR,sigmaB;
		ekf_handle.getParam("sigmaR", sigmaR);
		ekf_handle.getParam("sigmaB", sigmaB);
		Eigen::Matrix2d R;
		R(0,0)=sigmaR*sigmaR;
		R(0,1)=R(1,0)=0;////R: [sigmaR^2 0; 0 sigmaB^2]
		R(1,1)=sigmaB*sigmaB;
		ekf_cal->R=R;
		ekf_cal->RE=R;
		
		int Switch_inflate_noise;
		ekf_handle.getParam("SWITCH_INFLATE_NOISE", Switch_inflate_noise);
		if(Switch_inflate_noise)
		{
			ekf_cal->QE=2*Q;
			ekf_cal->RE=8*R;
		}
		
		ekf_handle.getParam("AT_WAYPOINT", at_waypoint);
		ekf_handle.getParam("MAXG", maxG);
		ekf_handle.getParam("RATEG", rateG);
		ekf_handle.getParam("NUMBER_LOOPS", number_loops);
		ekf_handle.getParam("WHEELBASE", ekf_cal->wheel_base);
		ekf_cal->set_vehicle_animation();
		ekf_handle.getParam("V", velocity);
		ekf_handle.getParam("SWITCH_CONTROL_NOISE", switch_control_noise);
		ekf_handle.getParam("SWITCH_SEED_RANDOM", switch_seed_random);
		ekf_handle.getParam("SWITCH_HEADING_KNOWN", switch_heading_know);
		ekf_handle.getParam("MAX_RANGE", max_range);
		ekf_handle.getParam("SWITCH_SENSOR_NOISE", switch_sensor_noise);
		ekf_handle.getParam("SWITCH_ASSOCIATION_KNOWN", switch_association_know);
		ekf_handle.getParam("SWITCH_USE_IEKF", switch_use_Iekf);
		ekf_handle.getParam("GATE_REJECT", gate_reject);
		ekf_handle.getParam("GATE_AUGMENT", gate_augment);
		ekf_handle.getParam("SWITCH_BATCH_UPDATE", switch_batch_update);
		if(switch_seed_random)//if SWITCH_SEED_RANDOM, randn('state',SWITCH_SEED_RANDOM), end
			ekf_cal->set_random_seed(switch_seed_random);
		ROS_INFO("All parameters have been initialized !");
	}
	ekf_console::~ekf_console()
	{

	}
	void ekf_console::start()
	{
		landmarks_pub = ekf_handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/visual_landmarks", 1);//landmarks
		waypoints_pub = ekf_handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/visual_waypoints", 1);//waypoints
		
		state_viz_pub_ = ekf_handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/state_viz", 1);  // map
		data_association_viz_pub_ = ekf_handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/data_association_viz", 1);  // data_association
		
		odom_pub = ekf_handle.advertise<nav_msgs::Odometry>("/slam_simulator/odom", 5);
		xest_pub = ekf_handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/xest_viz", 5);  // robot_pose
		xtrue_pub = ekf_handle.advertise<visualization_msgs::MarkerArray>("/slam_simulator/xtrue_viz", 5);  // robot_pose
		path_pub = ekf_handle.advertise<nav_msgs::Path>("/slam_simulator/xest_trajectory",1, true);
		
		// 确保发布标记时，已经打开rviz进行显示
		while (landmarks_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())	
				return  ;
			ROS_WARN_ONCE("Please create a subscriber to the marker or launch rviz ! ");
			sleep(1);
		}
		
		landmarks_pulish();//Posting landmarks  information for display
		waypoints_publish();//Posting waypoints  information for display
	}

	void ekf_console::run()
	{//main loop
		
		ekf_cal->initialise_store();		//stored data for off-line
		while((iwp !=-1)&&(ros::ok()))
		{
			compute_steering();		//compute true data
			//perform loops: if final waypoint reached, go back to first
			if((iwp==-1)&&(number_loops>1))
			{
				iwp=0;
				number_loops=number_loops-1;
			}
			ekf_cal->vehicle_model(velocity, G, dt);
			ekf_cal->add_control_noise(velocity, G, switch_control_noise);	//默认添加噪声
			ekf_cal->predict(dt);

			//if heading known, observe heading
			//ekf_cal->observe_heading(switch_heading_know);			//默认此函数不用，转向未知

			//EKF update step
			dtsum= dtsum + dt;
			if(dtsum>=dt_observe)
			{
				dtsum=0;
				get_observations();
				ekf_cal->add_observation_noise(switch_sensor_noise);
				if(switch_association_know == 1)
					ekf_cal->data_associate_known(ftag_visible);			//已知数据关联，路标观测顺序已知
				else ekf_cal->data_associate(gate_reject, gate_augment);		//默认进程

				if(switch_use_Iekf == 1)
					ekf_cal->update_iekf(5);
				else ekf_cal->update(switch_batch_update);				//默认进程

			}
			
			//offline data store
			ekf_cal->store_data();
			rviz_state();
			rviz_dataAssociation();
			odometry_publisher();
			rviz_robotpose();
			//r.sleep();
		}
	}
	void ekf_console::landmarks_pulish()
	{
		int markers_size=landmarks.cols();
		
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker marker;
		// 设置该标记的命名空间和ID，ID应该是独一无二的
		// 具有相同命名空间和ID的标记将会覆盖前一个
		marker.header.frame_id = "landmarks_link";
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
		marker.scale.x = 4;
		marker.scale.y = 4;
		marker.scale.z = 6;
		//设置标记颜色，确保alpha（不透明度）值不为0, 紫色
		marker.color.r = 160.0/255.0;
		marker.color.g = 32.0/255.0;
		marker.color.b = 240.0/255.0;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0);
		for(int i=0 ; i < markers_size ; i++)
		{
			// 设置帧 ID和时间戳
			marker.header.stamp = ros::Time::now();
			marker.id++;
			//设置标记位姿。 
			marker.pose.position.x = landmarks(0, i);
			marker.pose.position.y = landmarks(1, i);
			ma.markers.push_back(marker);
		}
		landmarks_pub.publish(ma);
	}
	void ekf_console::waypoints_publish()
	{
		int markers_size=waypoints.cols();

		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker point_line;
		
		point_line.header.frame_id = "waypoints_link";
		point_line.header.stamp = ros::Time::now();
		point_line.ns = "2Dwaypoints";
		point_line.action = visualization_msgs::Marker::ADD;
		point_line.lifetime = ros::Duration(0);					
		point_line.color.a = 0.5;							
		point_line.pose.orientation.w = 1.0;
		
		point_line.type = visualization_msgs::Marker::POINTS;	
		point_line.id = 0;
		point_line.scale.x = 2;
		point_line.scale.y = 2;
		point_line.scale.z = 2;
		point_line.color.r = 255.0/255.0;
		point_line.color.g = 0;
		point_line.color.b = 0;

		//存入路径点
		for(int i=0 ; i < markers_size ; i++)
		{
			geometry_msgs::Point p;

			//设置标记位姿。 
			p.x = waypoints(0, i);
			p.y = waypoints(1, i);
			p.z = 0;
			
			point_line.points.push_back(p);
		}
		ma.markers.push_back(point_line);
		
		point_line.type = visualization_msgs::Marker::LINE_STRIP;// 设置标记类型，线
		point_line.id = 1;
		// 设置标记的比例，所有方向上尺度1表示1米
		point_line.scale.x = 1;
		point_line.scale.y = 1;
		point_line.scale.z = 1;
		//设置标记颜色， 绿色
		point_line.color.r = 0;
		point_line.color.g = 255.0/255.0;
		point_line.color.b = 0;

		ma.markers.push_back(point_line);
		
		waypoints_pub.publish(ma);
	}
	void ekf_console::compute_steering()
	{
		Eigen::Vector2d current_wp;
		current_wp = waypoints.col(iwp);
		double d2= pow((current_wp(0) - ekf_cal->xtrue(0)), 2) + pow((current_wp(1) - ekf_cal->xtrue(1)), 2);
 		
		if(d2<(at_waypoint*at_waypoint))
		{
			iwp= iwp+1; 					// switch to next
			if(iwp>(waypoints.cols()-1))		//reached final waypoint, flag and return
			{
				iwp=-1;
				return ;
			}
			current_wp = waypoints.col(iwp);//next waypoint
		}
		
		//compute change in G to point towards current waypoint
		double deltaG=tool::normalize_angle( (atan2( current_wp(1) - ekf_cal->xtrue(1), current_wp(0) - ekf_cal->xtrue(0) ) - ekf_cal->xtrue(2) - G)  );
		
		//limit rate
		double maxDelta=rateG*dt;
		if(abs(deltaG)>maxDelta)
			deltaG= maxDelta*(sign(deltaG));
		G=G+deltaG;
		if(abs(G)>maxG)
			G=maxG*(sign(G));
	}
	void ekf_console::get_observations()
	{
		Eigen::RowVectorXd dx=landmarks.row(0).array() - ekf_cal->xtrue(0);
		Eigen::RowVectorXd dy=landmarks.row(1).array() - ekf_cal->xtrue(1);
		double phi=ekf_cal->xtrue(2);
		
		//仿真器计算符合测距范围内的路标点并保存id
		ftag_visible.clear();
		for(int i=0 ; i < landmarks_size; i++)			//incremental tests for bounding semi-circle
		{
			if( (abs(dx(i)) < max_range) && (abs(dy(i)) < max_range)		
				&&  ( (dx(i)*cos(phi) + dy(i)*sin(phi)) > 0)			
				&&  ( (dx(i)*dx(i) + dy(i)*dy(i)) < (max_range*max_range) )  )  // bounding box bounding line bounding circle
			{
				ftag_visible.push_back(i);
			}
		}
		
		//提取路标点的x，y坐标信息
		int visible_num=ftag_visible.size();
		Eigen::MatrixXd  Z_coord = Eigen::MatrixXd::Zero(2, visible_num); 
		for(int i=0 ; i< visible_num ; i++)
		{
			int id=ftag_visible[i];
			Z_coord(0, i)=landmarks(0, id);
			Z_coord(1, i)=landmarks(1, id);
		}
		
		//Compute exact observation计算路标点与当前真实位置的距离
		Z_coord.row(0) = Z_coord.row(0).array()-ekf_cal->xtrue(0);
		Z_coord.row(1) = Z_coord.row(1).array()-ekf_cal->xtrue(1);

		ekf_cal->Z.resize(2, visible_num);
		ekf_cal->Z.setZero(2, visible_num);
		//计算每个路标点的观测距离并保存 range
		ekf_cal->Z.row(0) =  (Z_coord.row(0).array().square()  + Z_coord.row(1).array().square()).sqrt();
		//计算每个路标点的观测角度并保存 bearing
		for(int i=0 ; i < visible_num ; i++)//atan2(dy,dx) - phi
			ekf_cal->Z(1,i)=atan2(Z_coord(1, i), Z_coord(0, i)) - phi;	
	}
	void ekf_console::rviz_state()
	{
		//double ellipse_scale_=10;
		//float quantiles=5 ;
		double quantiles=1 ;
		double ellipse_scale_=5;
		double pose_x=0, pose_y=0;// pose of the robot and landmarks (x,y,z=0)
		unsigned int pose_id=0;
		
		// covariance ellipses
		tf::Quaternion orientation;
		tf::Matrix3x3 tf3d;
		Eigen::Matrix2d eigenvectors;
		Eigen::Vector2d eigenvalues;
		Eigen::Matrix2d covariance;// get the covariance matrix 2x2 for each ellipsoid including robot pose
		
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "odom";
		marker.id = 0;
		marker.ns = "ekf_predict";
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0);

		// Count the number of landmarks + robot pose
		unsigned int objs_counter = ((ekf_cal->x.rows())-3 )/2 + 1;

		for (size_t i = 0; i < objs_counter; i++)
		{
			marker.id++;
			marker.color.a = 0.6;//设置标记颜色，确保alpha（透明度）值为0
			if (i == 0)
			{  // robot position
				marker.color.r = 128/255.0;//棕色
				marker.color.g = 42/255.0;
				marker.color.b = 42/255.0;
			}else{
				marker.color.r = 255/255.0;//黄色
				marker.color.g = 255/255.0;
				marker.color.b = 0;
				
				pose_id = i * 2 + 1;
			}
			// mean position
			pose_x=ekf_cal->x(pose_id);
			pose_y=ekf_cal->x(pose_id+1);
			
			// covariance ellipses
			covariance=ekf_cal->P.block(pose_id, pose_id, 2, 2);
			if(tool::computeEllipseOrientationScale2D(eigenvectors, eigenvalues, covariance) == false)
				continue;
			
			// Rotation matrix around  z axis
			tf3d.setValue(eigenvectors(0, 0), eigenvectors(0, 1), 0, eigenvectors(1, 0), eigenvectors(1, 1), 0, 0, 0, 1);

			// get orientation from rotation matrix
			tf3d.getRotation(orientation);
			
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.pose.position.x = pose_x;//pose.x();
			marker.pose.position.y = pose_y;//pose.y();
			marker.pose.position.z = 0;
			marker.pose.orientation.x = orientation.x();
			marker.pose.orientation.y = orientation.y();
			marker.pose.orientation.z = orientation.z();
			marker.pose.orientation.w = orientation.w();
			marker.scale.x = ellipse_scale_ * quantiles * sqrt(eigenvalues[0]);
			marker.scale.y = ellipse_scale_ * quantiles * sqrt(eigenvalues[1]);
			marker.scale.z = 0.00001;  // Z can't be 0, limitation of ROS
			ma.markers.push_back(marker);

			//写入标记数字
			marker.id++;
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			if (i == 0)
				marker.text = "robot";
			else marker.text = std::to_string(i);

			marker.pose.position.x = pose_x+5;//pose.x();
			marker.pose.position.y = pose_y+5;//pose.y();
			marker.pose.position.z = 0.1;
			marker.color.r = 135/255.0;
			marker.color.g = 206/255.0;
			marker.color.b = 235/255.0;
			marker.color.a = 1.0;//设置标记颜色，确保alpha（不透明度）值为0
			marker.scale.x = 10;
			marker.scale.y = 10;
			marker.scale.z = 10;
			ma.markers.push_back(marker);
		}
		state_viz_pub_.publish(ma);
	}
	void ekf_console::rviz_dataAssociation()
	{// robot pose
		double pose_x=ekf_cal->x(0);
		double pose_y=ekf_cal->x(1);
		
		geometry_msgs::Point pointRobotPose;
		pointRobotPose.x = pose_x;
		pointRobotPose.y = pose_y;
		pointRobotPose.z = 0;

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
		line_strip.scale.x = 1;  // line uses only x component
		line_strip.scale.y = 1;
		line_strip.scale.z = 1;
		line_strip.color.a = 1.0;
		line_strip.color.r = 255/255;
		line_strip.color.g = 0;
		line_strip.color.b = 0;

		// Draw latest data association:
		Eigen::MatrixXd Z_in; 
		Eigen::Vector3d x_in;
		Eigen::MatrixXd Z_out;

		int visible_num = ekf_cal->Z.cols();
		Z_in.resize(2, visible_num);
		for(int i=0 ;i < visible_num ; i++)
		{
			Z_in(0, i) = (ekf_cal->Z(0, i))*cos((ekf_cal->Z(1, i)));
			Z_in(1, i) = (ekf_cal->Z(0, i))*sin((ekf_cal->Z(1, i)));
		}
		x_in << (ekf_cal->x(0)), (ekf_cal->x(1)), (ekf_cal->x(2));
		ekf_cal->transformtoglobal(Z_in, x_in, Z_out);
		//visible_num = Z_out.cols();

		for (int i=0; i<visible_num; i++)
		{
			line_strip.points.clear();
			line_strip.points.push_back(pointRobotPose);
			
			geometry_msgs::Point pointLm;
			pointLm.x = Z_out(0, i);
			pointLm.y = Z_out(1, i);
			pointLm.z = 0.0;
			line_strip.points.push_back(pointLm);
			ma.markers.push_back(line_strip);
			line_strip.id++;
		}
		data_association_viz_pub_.publish(ma);
	}
	void ekf_console::odometry_publisher()
	{// robot estimate  pose 
		double xest_x=ekf_cal->x(0);
		double xest_y=ekf_cal->x(1);
		double xest_th=ekf_cal->x(2);
		double height=1.0;
		
		// odom to map tree broadcaster
		tf::TransformBroadcaster odom_broadcaster;
		ros::Time current_time= ros::Time::now();
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion xest_quat = tf::createQuaternionMsgFromYaw(xest_th);
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = xest_x;
		odom_trans.transform.translation.y = xest_y;
		odom_trans.transform.translation.z = height;
		odom_trans.transform.rotation = xest_quat;
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		//set the position
		odom.pose.pose.position.x = xest_x;
		odom.pose.pose.position.y = xest_y;
		odom.pose.pose.position.z = height;
		odom.pose.pose.orientation = xest_quat;
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = velocity*cos(xest_th);
		odom.twist.twist.linear.y = velocity*sin(xest_th);
		odom.twist.twist.angular.z = xest_th;
		//publish the message
		odom_pub.publish(odom);
		
		nav_msgs::Path xest_path;
		xest_path.header.stamp=current_time;
		xest_path.header.frame_id="odom";
		geometry_msgs::PoseStamped xest_stamped;
		xest_stamped.pose.position.z = height;
		
		size_t xest_paths=ekf_cal->OffLine_data.estimate_path.size();
		Eigen::Vector3d x_est;
		for(size_t i=0;i< xest_paths; i++)
		{
			x_est=ekf_cal->OffLine_data.estimate_path[i];
			xest_stamped.pose.position.x = x_est(0);
			xest_stamped.pose.position.y = x_est(1);
			
			xest_quat = tf::createQuaternionMsgFromYaw((x_est(2)));
			xest_stamped.pose.orientation.x = xest_quat.x;
			xest_stamped.pose.orientation.y = xest_quat.y;
			xest_stamped.pose.orientation.z = xest_quat.z;
			xest_stamped.pose.orientation.w = xest_quat.w;
			xest_stamped.header.stamp=current_time;
			xest_stamped.header.frame_id="odom";
			xest_path.poses.push_back(xest_stamped);
		}
		path_pub.publish(xest_path);
	}
	void ekf_console::rviz_robotpose()
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
		marker.scale.x = 6;
		marker.scale.y = 6;
		marker.scale.z = 2;
		//设置标记颜色，确保alpha（不透明度）值不为0
		marker.color.a = 1.0;
		
		// robot true pose
		marker.id = 0;
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 1.0;
		marker.pose.position.x = ekf_cal->xtrue(0);
		marker.pose.position.y = ekf_cal->xtrue(1);
		marker.pose.position.z = 2;
		ma.markers.push_back(marker);
		
		// robot estimate pose
		marker.id = 1;
		marker.color.r = 1.0;
		marker.color.g = 0;
		marker.color.b = 0;
		marker.pose.position.x = ekf_cal->x(0);
		marker.pose.position.y = ekf_cal->x(1);
		marker.pose.position.z = 2;
		ma.markers.push_back(marker);

		xtrue_pub.publish(ma);
	}




}
