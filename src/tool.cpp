#include "slam_simulator/tool.h"

namespace slam_simulator
{
	tool::tool()
	{

	}
	tool::~tool()
	{

	}
	Eigen::VectorXd tool::CalculateRMSE(const std::vector< Eigen::VectorXd >& estimations, const std::vector< Eigen::VectorXd >& ground_truth)
	{
		/**
		TODO:
		* Calculate the RMSE here.均方根误差
		*/
		Eigen::VectorXd rmse(4);
		rmse<<0,0,0,0;

		if(estimations.size() != ground_truth.size()|| estimations.size() == 0)
		{
			std::cout << "Invalid estimation or ground_truth data" << std::endl;
			return rmse;
		}

		for (int i = 0; i < estimations.size(); i++) 
		{
			Eigen::VectorXd residual = estimations[i] - ground_truth[i];
			residual = residual.array()*residual.array();
			rmse += residual;
		}
		//calculate the mean
		rmse = rmse/estimations.size();

		//calculate the squared root
		rmse = rmse.array().sqrt();

		//return the result
		return rmse;
	}
	Eigen::MatrixXd tool::CalculateJacobian(const Eigen::VectorXd& x_state)
	{
		/**
		TODO:
		* Calculate a Jacobian here.
		*/
		Eigen::MatrixXd Hj(3,4);
		//recover state parameters
		float px = x_state(0);
		float py = x_state(1);
		float vx = x_state(2);
		float vy = x_state(3);

		//pre-compute a set of terms to avoid repeated calculation
		float c1 = px*px+py*py;
		float c2 = sqrt(c1);
		float c3 = (c1*c2);

		//check division by zero
		if(fabs(c1) < 0.0001)
		{
			std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
			return Hj;
		}

		//compute the Jacobian matrix
		Hj << (px/c2), (py/c2), 0, 0,
		-(py/c1), (px/c1), 0, 0,
		py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

		return Hj;
	}
	float tool::normalize_angle(float phi)
	{
		//Normalize phi to be between -pi and pi
		while(phi>M_PI) {
			phi = phi - 2*M_PI;
		}

		while(phi<-M_PI) {
			phi = phi + 2*M_PI;
		}
		return phi;
	}
	void tool::normalize_bearing(Eigen::VectorXd& Z)
	{
		//Normalize phi to be between -pi and pi
		for (int i = 1; i < Z.size(); i += 2) 
			Z(i) = normalize_angle(Z(i));
		
	}
	double tool::fmod(double x, double y)
	{
		long n;
		double f;
		if((x<0.000000000000001 && x>-0.000000000000001) || (y<0.000000000000001 && y>-0.000000000000001))
			return x;
		f = x/y;
		n = (long)f;
		if(n>f) 
			n=n-1;
		f =x-n*y;
		
		return f;
	}
	void tool::makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues)
	{
		// Note that sorting of eigenvalues may end up with left-hand coordinate system.
		// So here we correctly sort it so that it does end up being righ-handed and normalised.
		Eigen::Vector3d c0;
		c0.setZero();
		c0.head(2) = eigenvectors.col(0);
		c0.normalize();
		Eigen::Vector3d c1;
		c1.setZero();
		c1.head(2) = eigenvectors.col(1);
		c1.normalize();
		Eigen::Vector3d cc = c0.cross(c1);
		if (cc(2) < 0)
		{
			eigenvectors << c1.head(2), c0.head(2);
			double e = eigenvalues(0);
			eigenvalues(0) = eigenvalues(1);
			eigenvalues(1) = e;
		}else
			eigenvectors << c0.head(2), c1.head(2);
	}
	bool tool::computeEllipseOrientationScale2D(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues, const Eigen::Matrix2d& covariance)
	{
		eigenvectors.setZero(2, 2);
		eigenvalues.setIdentity(2, 1);
		
		// NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(covariance);
		// Compute eigenvectors and eigenvalues
		if (eigensolver.info() == Eigen::Success)
		{
			eigenvalues = eigensolver.eigenvalues();
			eigenvectors = eigensolver.eigenvectors();
		}else{
			eigenvalues = Eigen::Vector2d::Zero();  // Setting the scale to zero will hide it on the screen
			eigenvectors = Eigen::Matrix2d::Identity();
			return false;
		}
		
		// Be sure we have a right-handed orientation system
		makeRightHanded(eigenvectors, eigenvalues);
		return true;
	}




}
