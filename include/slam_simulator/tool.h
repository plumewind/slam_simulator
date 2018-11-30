#ifndef SLAM_SIMULATOR_TOOL_H
#define SLAM_SIMULATOR_TOOL_H

#include <iostream>

#include <limits.h>

// Eigen 部分
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#define sign(x) ((x)>0 ? (1):(-1))
#define random(a,b) (rand()%(b-a+1)+a)

namespace slam_simulator
{

	struct Record {
		Eigen::Vector3f odom;					//里程计数据, 与x轴夹角, 行走距离, 角度误差
		std::vector<Eigen::Vector3f> radar;		//激光数据, 路标id号, 射线距离, 射线角度组成一组激光数据
	};
	
	typedef struct state_data
	{//the SLAM state for robot
		Eigen::MatrixXd x;//the SLAM state vector at time k
		Eigen::MatrixXd P;//the diagonals of the SLAM covariance matrix at time k
	}STATE;
	
	typedef struct stored_data
	{//stored data for off-line
		size_t i;
		std::vector<Eigen::Vector3d> estimate_path;//path-x the vehicle path estimate (ie, where SLAM estimates the vehicle went)
		std::vector<Eigen::Vector3d> real_path;//true-xtrue the vehicle 'true'-path (ie, where the vehicle *actually* went)
		std::vector<STATE> state;//the SLAM state vector at time k
	}Sdata;
	
	class tool
	{
	public:
		tool();
		~tool();
		
		/**
		* A helper method to calculate RMSE.
		*/
		static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

		/**
		* A helper method to calculate Jacobians.
		*/
		static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

		/**
		* @brief Adjust a float to the interval [-pi, pi], which is a 2*pi loop period.
		 * @param angle Need to adjust the floating  number.
		*/
		static float normalize_angle(float phi);

		/**
		* Normalized the angles from observation
		*/
		static void normalize_bearing(Eigen::VectorXd& Z);
		
		/**
		* @brief Deformation of the mod function in MATLAB.
		*/
		static double fmod(double x, double y);
		
		static void makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues);
		
		static bool computeEllipseOrientationScale2D(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues, const Eigen::Matrix2d& covariance);
		
	};

}
#endif