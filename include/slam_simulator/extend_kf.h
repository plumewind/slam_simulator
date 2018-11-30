#ifndef SLAM_SIMULATOR_EXTEND_KF_H
#define SLAM_SIMULATOR_EXTEND_KF_H

#include <iostream>  
#include <vector>  

#include "slam_simulator/tool.h"

namespace slam_simulator
{
	class extend_kf
	{
		public:
			extend_kf();
			~extend_kf();
			/**
		* @brief Initialize offline storage
		*/
		void initialise_store(void);
		
		/**
		* @brief Set the random number generator seed, which is not used here.
		 * @param seed Random number generator seed, fixed random number generated sequence.
		*/
		void set_random_seed(double seed);
		
		/**
		* @brief Set up a motion machine model, 2X2 matrix
		*/
		void set_vehicle_animation(void);
		
		
		/**
		* @brief Calculate the true position of the robot according to given speed, rotation angle and other information.
		 * @param velocity The line speed of the robot.
		 * @param Sangle Robot steering angle value.
		 * @param dt  The robot's sampling cycle.
		*/
		void vehicle_model(double velocity, double Sangle, double dt);
		
		/**
		* @brief Add normally distributed random noise to travel speed and steering angle
		 * @param V The true line speed of the robot.
		 * @param G The robot true steering angle value.
		 * @param flag  Select whether to add noise to speed and angle, if is 1 add
		*/
		void add_control_noise(double V, double G, int flag);
		
		/**
		* @brief Ekf algorithm prediction steps.
		 * @param dt System sampling period
		*/
		void predict(double dt);//EKF predict step
		
		/**
		* @brief Perform state update for a given heading measurement, with fixed measurement noise.shi
		 * @param flag Whether to use fixed noise for measurement update, if it is 1, use it.
		*/
		void observe_heading(int flag);
		
		/**
		* @brief Add normal distribution random noise to the observed measurements.
		 * @param flag Whether to add random noise to the measurement, if it is 1, add it.
		*/
		void add_observation_noise(int flag);
		
		/**
		* @brief In the case of a data association list, data association operations are performed.
		 * @param ftag_visible A list of landmark ids visible in the current state.
		*/
		void data_associate_known(std::vector<int> ftag_visible);
		
		/**
		* @brief  Simple gated nearest-neighbour data-association. No clever feature
		* 		caching tricks to speed up association, so computation is O(N), where
		* 		N is the number of features in the state.
		* 		linear search for nearest-neighbour, no clever tricks (like a quick
		* 		bounding-box threshold to remove distant features; or, better yet,
		* 		a balanced k-d tree lookup). TODO: implement clever tricks.
		 * @param reject_flag maximum distance for association
		 * @param augment_flag minimum distance for creation of new feature
		*/
		void data_associate(double reject_flag, double augment_flag);
		
		/**
		* @brief Given a feature index (ie, the order of the feature in the state vector),
		* 		predict the expected range-bearing observation of this feature and its Jacobian.
		 * @param IDf index of feature order in state
		 * @param out_Z predicted observation, including observation distance and angle
		 * @param out_H  observation Jacobian
		*/
		void observe_model(int IDf, Eigen::MatrixXd& out_Z, Eigen::MatrixXd& out_H);
		
		/**
		* @brief This is a very important step of the Kalman filtering algorithm, the status update step.
		 * @param flag if 1, process scan in batch, if 0, process sequentially
		*/
		void update(int flag);
		
		/**
		* @brief This is an iterative EKF algorithm update step. This implementation is rather inefficient for 

		* 		 SLAM, as it involves the inversion of P (ie, the update is O(n^3) for n landmarks.
		 * @param iter_num Number of iterations
		*/
		void update_iekf(int iter_num);
		
		/**
		* @brief computing the observation model jacobian and predicted covariance.
		 * @param in_R observation uncertainty matrix
		 * @param in_Po P(k|k-1) - predicted covariance in k-1
		 * @param out_H  the observation model jacobian in k
		*/
		void calculate_H_P(Eigen::MatrixXd in_R, Eigen::MatrixXd in_Po, Eigen::MatrixXd &out_H);
		
		/**
		* @brief computing the innovation, given the non-linear observation model and predicted state.
		 * @param in_Z observation matrix
		 * @param in_xo predicted state in k-1
		 * @param in_Poi predicted covariance to inversion in k, Poi= inv(P)
		 * @param in_H the observation model jacobian in k
		 * @param in_Ri  observation uncertainty matrix to inversion in k-1, Ri= inv(R)
		*/
		void calculate_v_x(Eigen::MatrixXd in_Z, Eigen::MatrixXd in_xo, Eigen::MatrixXd in_Poi, Eigen::MatrixXd in_H, Eigen::MatrixXd in_Ri);
		
		/**
		* @brief Calculate the KF (or EKF) update given the prior state [x,P]

		* 		the innovation [v,R] and the (linearised) observation model H.

		* 		The result is calculated using Cholesky factorisation, which

		*		 is more numerically stable than a naive implementation.
		 * @param v The latest observation matrix
		 * @param R Latest Observation Covariance Matrix
		 * @param H  The Jacobian matrix under the latest observation
		*/
		void KF_cholesky_update(Eigen::MatrixXd v, Eigen::MatrixXd R, Eigen::MatrixXd H);
		
		/**
		* @brief Based on the latest observations and estimates, the SLAM's state matrix x and covariance matrix P are updated.
		*/
		void augment(void);
		
		/**
		* @brief Real-time preservation of the robot's real state data and estimated state data.
		*/
		void store_data(void);
		
		/**
		* @brief Transform a list of poses [x;y;phi] so that they are global wrt a base pose
		* @param p The relationship matrix to be converted.
		* @param b The matrix of the transformed benchmarks.
		* @param out_p  The result of the conversion of the target matrix.
		*/
		void transformtoglobal(Eigen::MatrixXd p, Eigen::Vector3d b, Eigen::MatrixXd &out_p);
		
		Eigen::Vector3d xtrue;//the true pose of robot 3*1
		Eigen::VectorXd x;//卡尔曼滤波估计出来的状态，包括机器人的三个状态和路标
		Eigen::MatrixXd P;//卡尔曼估计协方差，就是x变量的协方差  Matrix<float, Dynamic, Dynamic> P;  
		
		int *da_table; //data association table 
		Eigen::Matrix2d Q; //inflate estimated noises (ie, add stabilising noise)
		Eigen::Matrix2d R; // observation noises
		Eigen::Matrix2d QE; //运动模型噪声inflate estimated noises (ie, add stabilising noise)
		Eigen::Matrix2d RE; // 观测模型噪声observation noises
		
		Eigen::Matrix<double, 2, Eigen::Dynamic>  Z; //第一行存储每个观测点距离，第二行存储角度
		
		double wheel_base;//metres, vehicle wheel-base
		Eigen::Matrix<double, 2, 3> veh;//vehicle animation
		
		Sdata OffLine_data;//Offline database.
	private:
		double Vn;//带噪声的速度值
		double Gn;//带噪声的转向角
		
		//Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Zf;
		Eigen::Matrix<double, 2, Eigen::Dynamic> Zf;	//在Z矩阵中提取出具有关联信息的(已经观测过的)landmark路标信息
		Eigen::Matrix<double, 2, Eigen::Dynamic> Zn;	//在Z矩阵中提取出新的未遇见过的landmark路标信息
		std::vector<int> idf;						//当前状态下可见的而且之前见过的landmark路标点的id号，注意从0开始
		
		//ukf估计器相关参数
		double alpha;		//确定sigma点分布在均值多远范围内的比例参数
		double ki = 0;		//确定sigma点分布在均值多远范围内的比例参数
		double beta = 2;	//表示对高斯表示的附加的（较高阶）分布信息的编码
		double scale = 3.0;
		
		//default_random_engine generator;//如果用这个默认的引擎，每次生成的随机序列是相同的。
		std::random_device rd;
		std::mt19937 gen;
		//normal(0,1)中0为均值，1为方差
		std::normal_distribution<double> normal;
	};



}
#endif