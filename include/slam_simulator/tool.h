#ifndef SLAM_SIMULATOR_TOOL_H
#define SLAM_SIMULATOR_TOOL_H

#include <iostream>

#include <limits.h>

#include <Eigen/Eigen>
#include <Eigen/Core>


namespace slam_simulator
{

	struct Record {
		Eigen::Vector3f odom;					//里程计数据, 与x轴夹角, 行走距离, 角度误差
		std::vector<Eigen::Vector3f> radar;		//激光数据, 路标id号, 射线距离, 射线角度组成一组激光数据
	};
	
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
		* Noarmlize the angle
		*/
		static float normalize_angle(float phi);

		/**
		* Normalized the angles from observation
		*/
		static void normalize_bearing(Eigen::VectorXd& Z);
		
		static void makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues);
		
		static bool computeEllipseOrientationScale2D(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues, const Eigen::Matrix2d& covariance);
		
	};

}
#endif