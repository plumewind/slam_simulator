#include "slam_simulator/unscented_kf.h"

namespace slam_simulator
{

	unscented_kf::unscented_kf(unsigned int landmark_size, float _motion_noise )
		:IS_initialized_(false), Scale(3.0)
	{
		int r       		= 3;       
		Mu_x          	= Eigen::VectorXd::Zero(r);
		Sigma_y       	= Eigen::MatrixXd::Identity(3,3)*0.001;

		R_ .resize(3, 3);
		R_<< _motion_noise, 0, 0,
		0, _motion_noise , 0,
		0, 0,   _motion_noise/10;

		Q_ = Eigen::MatrixXd::Zero(2, 2);
		Q_ << 0.1, 0, 0, 0.01;
	}
	unscented_kf::~unscented_kf()
	{
		
		
	}
	void unscented_kf::compute_sigma_points(Eigen::MatrixXd& sigma_points, bool pred)
	{
		int n  = Mu_x.rows();
		if(pred) n = 3;
		int num_sig = 2 * n + 1;
		float lambda = Scale - n;

		//get the square root of matrix sigma
		Eigen::MatrixXd D = Sigma_y.topLeftCorner(n, n).llt().matrixL();

		sigma_points = Eigen::MatrixXd::Zero(n, 2 * n + 1);
		sigma_points.col(0) = Mu_x.head(n);

		for (int i = 0; i < n; i++) 
		{
			sigma_points.col(i + 1)    	=   sigma_points.col(0) + sqrt(n + lambda) *  D.col(i);
			sigma_points.col(i + n + 1) 	=   sigma_points.col(0) - sqrt(n + lambda) *  D.col(i);
		}
	}
	void unscented_kf::recover_mu_sigma(const Eigen::MatrixXd& sig_pts)
	{
		int  n = sig_pts.rows();
		float lambda = Scale - n;

		//weight vector
		Eigen::VectorXd weights = Eigen::VectorXd::Zero(2*n + 1);
		weights(0) = lambda / Scale;
		for (int i = 1; i < 2 * n + 1; i++) 
			weights(i) = 0.5 / Scale;
		
		Eigen::RowVectorXd angle_c = sig_pts.row(2).array().cos();
		Eigen::RowVectorXd angle_s = sig_pts.row(2).array().sin();

		double x_bar = angle_c * weights;
		double y_bar = angle_s * weights;

		//remove mu
		Mu_x.head(n) = (sig_pts * weights);
		double angle = atan2(y_bar, x_bar);
		Mu_x(2) = tool::normalize_angle(angle);

		//remove sigma
		Eigen::MatrixXd dsigma  = Eigen::MatrixXd::Zero(n, n);
		for (int i = 0; i < 2 * n + 1; i++) 
		{
			Eigen::VectorXd diff = sig_pts.col(i) - Mu_x.head(n);
			diff(2) = tool::normalize_angle(diff(2));
			dsigma = dsigma + weights(i) * diff * diff.transpose();
		}
		Sigma_y.topLeftCorner(n, n) = dsigma;
		//cout << "inside recover points" << endl;
	}
	void unscented_kf::Prediction(const Eigen::Vector3f& motion)
	{
		Eigen::MatrixXd Xsig;

		compute_sigma_points(Xsig, true);     // Xsig    3  X    (2*3 +1) 
		//dimensionality
		double r1    	= motion(0);
		double t    	= motion(1);
		double r2    	= motion(2);

		for (int i = 0; i < Xsig.cols(); i++)
		{
			double angle = Xsig(2,i);
			Xsig(0, i) = Xsig(0, i) + t * cos(angle + r1);
			Xsig(1, i) = Xsig(1, i) + t * sin(angle + r1);
			Xsig(2, i) = Xsig(2, i) + r1 + r2;
		}

		recover_mu_sigma(Xsig);
		Sigma_y.topLeftCorner(3,3)  = Sigma_y.topLeftCorner(3,3) + R_;
	}
	void unscented_kf::add_landmark_to_map(const Eigen::Vector3f& z)
	{
		int n = Mu_x.rows();
		LandmarkINmap.push_back((int)z(0));

		//increae size of mu and sigma
		Eigen::VectorXd tempmu = Eigen::VectorXd::Zero(n + 2);
		tempmu.head(n) 	=  Mu_x; 
		tempmu(n)      	= (double)z(1);
		tempmu(n + 1)  	= (double)z(2);
		Mu_x = tempmu;

		Eigen::MatrixXd tempsigma = Eigen::MatrixXd::Zero(n + 2, n + 2);
		tempsigma.topLeftCorner(n, n) = Sigma_y;
		tempsigma.bottomRightCorner(2, 2) = Q_;
		Sigma_y = tempsigma;
		//transform from [range, bearing] to the x/y location of the landmark
		//this operation initializes sthe uncertainty in the position of the landmark
		//sample sigma points
		Eigen::MatrixXd sig_pts;
		compute_sigma_points(sig_pts);
		//normalize
		Eigen::VectorXd Z = (sig_pts.row(2)).transpose();
		tool::normalize_bearing(Z);
		sig_pts.row(2) = Z.transpose();

		//compute newX and newY
		Eigen::RowVectorXd angle_c = (sig_pts.row(2) + sig_pts.row(n + 1)).array().cos();
		Eigen::RowVectorXd angle_s = (sig_pts.row(2) + sig_pts.row(n + 1)).array().sin();

		Eigen::RowVectorXd delta_X = sig_pts.row(n).array() * angle_c.array();
		Eigen::RowVectorXd delta_Y = sig_pts.row(n).array() * angle_s.array();

		sig_pts.row(n)   = sig_pts.row(0) + delta_X ; 
		sig_pts.row(n+1) = sig_pts.row(1) + delta_Y ; 
		//update mu and sigma
		recover_mu_sigma(sig_pts);
	}
	void unscented_kf::update(const Eigen::MatrixXd& sigma_points, const Eigen::Vector3f& Z)
	{
		 int n = sigma_points.rows();
		int num_sig = sigma_points.cols();
		double lambda = Scale - n;
		int index = 0;
		for (int i = 0; i < LandmarkINmap.size(); i++)
		{
			if (LandmarkINmap[i] == (int)Z(0))
			{
				index = i;
				break;
			}
		}

		Eigen::VectorXd DX = sigma_points.row(2*index + 3) - sigma_points.row(0);
		Eigen::VectorXd DY = sigma_points.row(2*index + 4) - sigma_points.row(1);

		Eigen::MatrixXd Z_points = Eigen::MatrixXd::Zero(2, 2 * n + 1);
		for (int i = 0; i < num_sig; i++) 
		{
			double q = pow(DX(i), 2) + pow(DY(i), 2);
			Z_points(0, i) = sqrt(q);
			Z_points(1, i) = tool::normalize_angle(atan2(DY(i), DX(i)) - sigma_points(2, i));
		}

		//weight vector
		Eigen::VectorXd weights = Eigen::VectorXd(num_sig);
		weights(0) = lambda / Scale;
		for (int i = 1; i < num_sig; i++) 
			weights(i) = 0.5 / Scale;
		
		//zm is the recovered expected measurement mean from z_points.
		//It will be a 2x1 vector [expected_range; expected_bearing].
		Eigen::VectorXd z_mean = Eigen::VectorXd::Zero(2);
		z_mean(0) = Z_points.row(0)*weights;
		// cout << "Z: mean(0): " << z_mean(0) << endl;
		Eigen::RowVectorXd angle_c = Z_points.row(1).array().cos();
		Eigen::RowVectorXd angle_s = Z_points.row(1).array().sin();
		double x_bar = angle_c * weights;
		double y_bar = angle_s * weights;
		z_mean(1) = tool::normalize_angle(atan2(y_bar, x_bar));

		// TODO: Compute the innovation covariance matrix S (2x2).
		//Compute sigma_x_z (which is equivalent to sigma times the Jacobian H transposed in EKF).
		//sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
		Eigen::MatrixXd S = Eigen::MatrixXd::Zero(2,2);
		Eigen::MatrixXd sigma_x_z = Eigen::MatrixXd::Zero(n,2);
		for (int i = 0; i < num_sig; i++)
		{
			Eigen::VectorXd tmpz = Z_points.col(i) - z_mean;
			tmpz(1) = tool::normalize_angle(tmpz(1));
			S = S + tmpz*tmpz.transpose()*weights(i);

			Eigen::VectorXd tmpx = sigma_points.col(i) - Mu_x;
			tmpx(2) = tool::normalize_angle(tmpx(2));
			sigma_x_z = sigma_x_z + weights(i)*tmpx*tmpz.transpose();
		}
		S = S + Q_;

		// Compute the Kalman gain
		Eigen::MatrixXd Kt = sigma_x_z*(S.inverse());

		Eigen::VectorXd z_actual = Eigen::VectorXd(2);
		z_actual << Z(1), Z(2);
		Eigen::VectorXd zdiff = z_actual-z_mean;
		zdiff(1) = tool::normalize_angle(zdiff(1));

		Mu_x = Mu_x + Kt*(zdiff);
		Sigma_y = Sigma_y - Kt*S*Kt.transpose();

		// TODO: Normalize the robot heading mu(3)
		Mu_x(2) = tool::normalize_angle(Mu_x(2));
	}
	void unscented_kf::Correction(const std::vector< Eigen::Vector3f >& observation)
	{
		// number of measurements in this step
		int m = observation.size();
		//[range, bearing, range, bearing, .....]
		//Jacobian matrix;
		for (int i = 0; i < m; i++)
		{
			auto&  reading  = observation[i];
			//landmark is not seen before, so to initialize the landmarks
			if (std::find(LandmarkINmap.begin(), LandmarkINmap.end(), (int)reading(0)) == LandmarkINmap.end())
			{
				add_landmark_to_map(reading);
			} else {
				//Compute sigma points from the predicted mean and covariance
				Eigen::MatrixXd sigma_points;
				compute_sigma_points(sigma_points);
				update(sigma_points, reading);
			}
		}
	}
	void unscented_kf::ProcessMeasurement(const Record& record)
	{
		Prediction(record.odom); 
		Correction(record.radar);
	}

	
	
	
	
	



}
