#ifndef SLAM_SIMULATOR_EKF_CONSOLE_H
#define SLAM_SIMULATOR_EKF_CONSOLE_H

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// add ros msgs
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "slam_simulator/extend_kf.h"
#include "slam_simulator/tool.h"

namespace slam_simulator
{
	class ekf_console
	{
		public:
			ekf_console();
			~ekf_console();
			
			void start();
			void landmarks_pulish();
			void waypoints_publish();
			void run();//the main loop
			
			/**
			* @brief Calculate the new current steering angle and new current waypoint  in  the next moment
			*/
			void compute_steering();
			
			/**
			* @brief set the range-bearing observations and  landmark index tag for each observation
			*/
			void get_observations();
			

			/**
			* @brief compute the correct orientation and scale of covariance ellipsoids (make sure that  we output covariance
			* ellipsoids for right handed system of coordinates)
			*
			* @param eigenvectors the 2x2 matrix of eigenvectors
			* @param eigenvalues the 2d vector of eigen values
			*/
			void makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues);
			
			/**
			* @brief compute the orientation and scale of covariance ellipsoids
			*
			* @param orientation the orientation of the ellipsoid in Quaternions
			* @param scale the vector of the eigen values for calculating the size of the ellipse
			* @param covariance covariance matrix for current landmarks or robot pose
			*/
			void computeEllipseOrientationScale2D(tf::Quaternion& orientation, Eigen::Vector2d& scale, const Eigen::Matrix2d covariance);
			
			/**
			* @brief visualize the covariance ellipsoids for robot and landmarks
			*/
			void rviz_state();
			
			/**
			* @brief visualize the data associations for the landmarks observed by robot at the each step
			*/
			void rviz_dataAssociation();
			
			/**
			* @brief visualize the robotics odometry and the estimated trajectories at the each step
			*/
			void odometry_publisher();
			
			/**
			* @brief visualize the true pose and estimated pose of robot at the each step
			*/
			void rviz_robotpose();
		
		private:
			ros::NodeHandle ekf_handle;
		
			//publishers to rviz
			ros::Publisher landmarks_pub;
			ros::Publisher waypoints_pub;
			ros::Publisher data_association_viz_pub_, state_viz_pub_;  
			ros::Publisher odom_pub, path_pub;
			ros::Publisher xest_pub, xtrue_pub;
			
			Eigen::Matrix<double, 2, Eigen::Dynamic> landmarks;//Saves all the landmarks location information.
			Eigen::Matrix<double, 2, Eigen::Dynamic> waypoints;//Saves all the waypoints location information.
			int landmarks_size;//the size of landmarks
			int waypoints_size;//the size of waypoints
			
			extend_kf* ekf_cal;//Important ekf algorithm class, used to complete the main algorithm calculations.
			
			double dt;// change in time between predicts
			double dtsum;// change in time since last observation
			int iwp; //index to first waypoint 
			double G; // the steer angle in true pose
			double at_waypoint;// metres, distance from current waypoint at which to switch to next waypoint
			double maxG;// radians, maximum steering angle (-MAXG < g < MAXG)
			double rateG;//rad/s, maximum rate of change in steer angle
			int number_loops;// number of loops through the waypoint list
			double velocity;//the speed of robot
			int switch_control_noise;// if 0, velocity and gamma are perfect
			float switch_seed_random;// if not 0, seed the randn() with its value at beginning of simulation (for repeatability)
			int switch_heading_know;// if 1, the vehicle heading is observed directly at each iteration
			int switch_sensor_noise;// if 0, measurements are perfect
			int switch_association_know;//if 1, associations are given, if 0, they are estimated using gates
			int switch_use_Iekf;//if 1, use iterated EKF for updates, if 0, use normal EKF
			int switch_batch_update;// if 1, process scan in batch, if 0, process sequentially
			double gate_reject;// maximum distance for association
			double gate_augment;// minimum distance for creation of new feature
			double dt_observe;// 8*DT_CONTROLS, seconds, time interval between observations
			double max_range;//The maximum distance measured by the sensor
			
			std::vector<int> ftag_visible;//Used to save the id of the landmark point visible in the current state.
			
			
	};



}
#endif