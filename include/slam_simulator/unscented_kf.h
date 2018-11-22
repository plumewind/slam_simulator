#ifndef SLAM_SIMULATOR_UNSCENTED_KF_H
#define SLAM_SIMULATOR_UNSCENTED_KF_H

#include <iostream>  
#include <vector>  


#include "slam_simulator/tool.h"

namespace slam_simulator
{

	class unscented_kf
	{
	public:
		unscented_kf(unsigned int landmark_size, float _motion_noise = 0.01);
		~unscented_kf();
		
		/**
		*  Prediction the state after motion
		*/
		void Prediction(const Eigen::Vector3f& motion);

		/**
		*  Correctthe state after observation
		*/
		void Correction(const std::vector<Eigen::Vector3f>& observation);

		/**
		* Run the whole flow of the Kalman Filter from here.
		*/
		void ProcessMeasurement(const Record& record);

		/**
		* Computes the 2n+1 sigma points according to the unscented transform,
		* where n is the dimensionality of the mean vector mu.
		* The sigma points should form the columns of sigma_points,
		* sigma_points is an nx2n+1 matrix
		**/
		void compute_sigma_points(Eigen::MatrixXd& sigma_points, bool pred = false);

		/**
		* add new landmakr to the map if not observed before, update the database
		**/
		void add_landmark_to_map(const Eigen::Vector3f& z);

		/**
		* Recover mu and sigma from unscented points after transformation
		**/
		void recover_mu_sigma(const Eigen::MatrixXd& sig_pts);

		/**
		* Update the mu and sigma in the correction steps
		**/
		void update(const Eigen::MatrixXd& sigma_points, const Eigen::Vector3f& Z);
		
		Eigen::MatrixXd Sigma_y;
		Eigen::VectorXd Mu_x;
		
		std::vector<int> LandmarkINmap;
	private:
		
		//motion noise
		Eigen::MatrixXd R_;
		//measurement noise
		Eigen::MatrixXd Q_;
		
		// check whether the tracking toolbox was initialized or not (first measurement)
		bool IS_initialized_;

		float Scale;
		
		
	};


}
#endif
