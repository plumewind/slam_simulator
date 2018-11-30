#include "slam_simulator/extend_kf.h"

namespace slam_simulator
{
	extend_kf::extend_kf()
		:gen(rd()), normal(0,1)
	{
		//initialise states
		xtrue.setZero(3, 1);      //用0矩阵初始化,要指定行列数
		x.setZero(3, 1);
		P.setZero(3, 3);
		
		alpha = 0.85;
		ki = 80;
		beta = 2;
		//scale = 10.0;
		
		std::cout<<"The extend_kf class is initialized......"<<std::endl;
	}
	extend_kf::~extend_kf()
	{
		delete[] da_table;
	}
	void extend_kf::initialise_store(void)
	{
		STATE temp;
		OffLine_data.i = 0;
		OffLine_data.estimate_path.push_back(x);
		OffLine_data.real_path.push_back(xtrue);
		temp.x = x;
		temp.P = P.diagonal();//获取对角线上的元素
		//temp.P << P(0, 0), P(1, 1), P(2, 2);
		OffLine_data.state.push_back(temp);
	}
	void extend_kf::set_random_seed(double seed)
	{//随机发生器的状态进行初始化，并且定义该状态初始值
		srand(seed);
	}
	void extend_kf::set_vehicle_animation(void)
	{
		veh<<0, -wheel_base, -wheel_base, 0, -2, 2;
	}
	void extend_kf::vehicle_model(double velocity, double Sangle, double dt)
	{
		double angle = xtrue(2);
		xtrue(0) = xtrue(0) + velocity*dt*cos(Sangle+angle);
		xtrue(1) = xtrue(1) + velocity*dt*sin(Sangle+angle);
		xtrue(2) = tool::normalize_angle(angle + velocity*dt*sin(Sangle)/wheel_base);
	}
	void extend_kf::add_control_noise(double V, double G, int flag)
	{
		if(flag == 1)
		{
			Vn=V+  (normal(gen) )  * sqrt(Q(0, 0));
			Gn=G+  (normal(gen) )  * sqrt(Q(1, 1));
		}
	}
	void extend_kf::predict(double dt)
	{
		double angle_s=sin(Gn+x(2));
		double angle_c=cos(Gn+x(2));
		double vts=Vn*dt*angle_s;
		double vtc=Vn*dt*angle_c;
		
		//predict state
		x(0)=x(0)+vtc;
		x(1)=x(1)+vts; 
		x(2)=tool::normalize_angle(x(2)+Vn*dt*sin(Gn)/wheel_base);
		
		//% jacobians  
		Eigen::Matrix3d Gv;
		Eigen::Matrix<double, 3, 2> Gu;
		Gv.setIdentity(3, 3); 
		Gv(0,2)=-vts;
		Gv(1,2)=vtc;
		Gu<<dt*angle_c, -vts,
			 dt*angle_s, vtc,
			 dt*sin(Gn)/wheel_base, Vn*dt*cos(Gn)/wheel_base;
			 
		//% predict covariance
 		P.block(0, 0, 3, 3) = Gv*( P.block(0, 0, 3, 3) )*(Gv.transpose()) + Gu*QE*(Gu.transpose());
		if( (P.rows()) >3)
		{
			int p_cols=P.cols();
			P.block(0, 3 , 3, (p_cols-3))=Gv*(P.block(0, 3 , 3, (p_cols-3)));
			P.block(3, 0, (p_cols-3), 3)=(P.block(0, 3 , 3, (p_cols-3)).transpose());
		}
	}
	void extend_kf::observe_heading(int flag)
	{
		if(flag == 0) return ;
		
		double sigmaPhi=0.01*M_PI/180.0;//radians, heading uncertainty
		size_t x_len=x.rows();
		Eigen::MatrixXd H;
		H.setZero(1, x_len);
		H(2)=1;
		double v=tool::normalize_angle(xtrue(2)-x(2));
		
		//KF_joseph_update
		double R_d=sigmaPhi * sigmaPhi;
		Eigen::MatrixXd PHt;
		PHt=P*(H.transpose());
		Eigen::MatrixXd S=(H*PHt).array() + R_d;
		Eigen::MatrixXd Si=S.inverse();
		
		//make_symmetric
		Si = (Si + (Si.transpose()))*0.5;
		
		Eigen::MatrixXd PSD_check = ( Si.llt().matrixL() ).transpose() ;//Cholesky分解法 A=LL^T ,matlab用的是上三角矩阵
		Eigen::MatrixXd W= PHt*Si;
		x= x + W*v; 
		
		//% Joseph-form covariance update
		Eigen::MatrixXd P_t;
		P_t.setIdentity(P.rows(), P.cols()); 
		Eigen::MatrixXd C= P_t - W*H;
		P= C*P*(C.transpose()) + W*R_d*(W.transpose());
		
		//eps = std::numeric_limits<double>::epsilon(); // 这个就是
		P= P + P_t*(std::numeric_limits<double>::epsilon()); //% a little numerical safety
		PSD_check=  ( P.llt().matrixL() ).transpose() ;//Cholesky分解法 A=LL^T ,matlab用的是上三角矩阵
	}
	void extend_kf::add_observation_noise(int flag)
	{
		Eigen::MatrixXd z_rand;
		if(flag == 1)
		{
			int len=Z.cols();
			if(len > 0)
			{
				Z.row(0)=Z.row(0) + (z_rand.setRandom(1, len)) * sqrt(R(0, 0));
				Z.row(1)=Z.row(1) + (z_rand.setRandom(1, len)) * sqrt(R(1, 1));
			}
		}
	}
	void extend_kf::data_associate_known(std::vector< int > ftag_visible)
	{
		int marker_id=0;
		int len = ftag_visible.size();
		
		//相关变量进行初始化
		Zn.resize(2, len);
		Zn.setZero(2, len);
		int Zn_col=0;
		
		Zf.resize(2, len);
		Zf.setZero(2, len);
		int Zf_col=0;
		
		std::vector<int> idn;	//临时保存新的路标点
		idn.clear();
		idf.clear();
		for(int i=0 ; i<len ; i++)
		{
			marker_id=ftag_visible[i];	//从观测列表中取出路标id号
			if(da_table[marker_id] == 0)//new feature
			{
				Zn.col(Zn_col)=Z.col(i);
				idn.push_back(marker_id);
				Zn_col++;
			}else{
				Zf.col(Zf_col)=Z.col(i);
				idf.push_back(marker_id);
				Zf_col++;
			}
		}

		//压缩Zf和Zn两个矩阵（去除多余空间），方便后续程序的处理
		Eigen::MatrixXd temp;
		temp = Zn;
		Zn.resize(2, Zn_col);
		for(int i = 0 ; i < Zn_col ; i++)
			Zn.col(i) = temp.col(i);

		temp = Zf;
		Zf.resize(2, Zf_col);
		for(int i = 0 ; i < Zf_col ; i++)
			Zf.col(i) = temp.col(i);
		
		//% add new feature IDs to lookup table
		int Nxv= 3;					//% number of vehicle pose states
		double Nf= (x.rows() - Nxv)/2; 	//number of features already in map

		//table(idn)= Nf + (1:size(zn,2)); % add new feature positions to lookup table
		int new_fea_size = idn.size();
		for(int i = 0 ; i < new_fea_size; i++)
			da_table[idn[i]]= Nf +i+1;
	}
	void extend_kf::data_associate(double reject_flag, double augment_flag)
	{
		int Nxv= 3; 				// number of vehicle pose states
		int Nf= (x.rows() - Nxv)/2; 	//number of features already in map
		int len=Z.cols();			//获取当前状态下的观测信息（观测的路标数量）
		int jbest=-1;
		double nbest = LONG_MAX;
		double outer = LONG_MAX;
		Eigen::MatrixXd zp;
		Eigen::MatrixXd H;
		
		//相关变量进行初始化
		idf.clear();
		std::vector<int> Zn_id, Zf_id;
		Zf_id.clear();		//保存与地图近邻的路标id号（已经见过的路标）
		Zn_id.clear();		//保存与地图远邻的路标id号（没有见过的路标）
		for(int i=0 ; i< len ;i++)		
		{
			jbest = -1;
			nbest = LONG_MAX;
			outer = LONG_MAX;
			
			//在地图中的所有路标里寻找最相似路标
			for(int j = 0 ; j < Nf ; j++)
			{
				double nis=0, nd=0;
				observe_model(j, zp, H);
				Eigen::Vector2d v = Z.col(i) - zp;
				v(1)=tool::normalize_angle(v(1));
				Eigen::Matrix2d S=H*P*( H.transpose() ) + RE;	//计算样本协方差，又称为数据的协方差距离
				nis =  ( v.transpose() ) * ( S.inverse() ) * v;		//计算马氏距离
				nd = nis + log( (S.determinant() ) );			//计算归一化距离(determinant是指行列式值)
				if((nis < reject_flag) && (nd < nbest))			//if within gate, store nearest-neighbour
				{
					nbest= nd;
					jbest = j;
				}else if(nis < outer)					//else store best nis value
					outer= nis;
			}

			//add nearest-neighbour to association list
			if(jbest != (-1))
			{//保存相似路标信息
				Zf_id.push_back(i);
				idf.push_back(jbest);
			}else if(outer > augment_flag)//保存不相似路标信息
				Zn_id.push_back(i);
		}
		int id_size=Zf_id.size();
		Zf.resize(2, id_size);
		Zf.setZero(2, id_size);
		if(id_size != 0)
		{//将具有关联的路标点信息提取到观测矩阵
			for(int i=0;i<id_size;i++)
				Zf.col(i)=Z.col(Zf_id[i]);
		}
		id_size=Zn_id.size();
		Zn.resize(2, id_size);
		Zn.setZero(2, id_size);
		if(id_size != 0)
		{//将新的路标信息提取出来，并保存
			for(int i=0;i<id_size;i++)
				Zn.col(i)=Z.col(Zn_id[i]);
		}
	}
	void extend_kf::observe_model(int IDf, Eigen::MatrixXd& out_Z, Eigen::MatrixXd& out_H)
	{
		int Nxv= 3; 			//number of vehicle pose states
		int fpos= Nxv + IDf*2 ; // position of xf in state
		out_Z.resize(2, 1);
		out_Z.setZero(2, 1);
		out_H.resize(2, x.rows());
		out_H.setZero(2, x.rows());

		//% auxiliary values 辅助值
		double dx= x(fpos)  -x(0); 
		double dy= x(fpos+1)-x(1);
		double d2= dx*dx + dy*dy;
		double d= sqrt(d2);
		double xd= dx/d;
		double yd= dy/d;
		double xd2= dx/d2;
		double yd2= dy/d2;
		
		//predict 观测模型估计矩阵Zit
		out_Z<<d, (atan2(dy, dx) - x(2));
		//calculate H 观测模型协方差矩阵Hit
		out_H.leftCols(3)<<-xd, -yd, 0, yd2, -xd2, -1 ;
		out_H.middleCols(fpos, 2)<<xd, yd,  -yd2, xd2 ;
	}
	void extend_kf::update(int flag)
	{
		int Zf_len=Zf.cols();
		int x_len=x.rows();
		Eigen::MatrixXd H;
		Eigen::VectorXd  v;
		Eigen::MatrixXd  RR;
		Eigen::MatrixXd H_temp, zp;
		
		if(flag == 1)
		{
			H.resize(2*Zf_len, x_len); 
			H.setZero(2*Zf_len, x_len);
			v.resize(2*Zf_len, 1); 
			v.setZero(2*Zf_len, 1);
			RR.resize(2*Zf_len, 2*Zf_len); 
			RR.setZero(2*Zf_len, 2*Zf_len);
			
			for(int i=0; i < Zf_len ; i++)
			{
				int ii= 2*i ;
				observe_model(idf[i], zp, H_temp);
				H.middleRows(ii, 2) = H_temp;
				v(ii) = Zf(0, i) - zp(0);
				v(ii+1) = tool::normalize_angle( Zf(1,i) - zp(1) );
				RR.block(ii, ii, 2, 2)  = RE;
			}
			KF_cholesky_update(v, RR, H);
			
		}else{	//原系统这里用不到
			v.resize(2, 1); 
			for(int i=0 ; i < Zf_len ; i++)
			{
				observe_model(idf[i], zp, H_temp);
				v.setZero(2, 1);
				v<<(Zf(0, i) - zp(0)), ( tool::normalize_angle( Zf(1,i) - zp(1) )) ;
				//FIXME 这里的R不知道是否可行，MATLAB源码用的是RR
				KF_cholesky_update(v, R, H_temp);
			}
		}
		
		augment();
	}
	void extend_kf::update_iekf(int iter_num)
	{
		if (idf.empty())  
			return ;
		
		int Zf_len=Zf.cols();
		Eigen::MatrixXd RR, Zf_Zf;
		RR.resize(2*Zf_len, 2*Zf_len);
		RR.setZero(2*Zf_len, 2*Zf_len);
		Zf_Zf.resize(2*Zf_len, 1);
		Zf_Zf.setZero(2*Zf_len, 1);
		
		for(int i=0 ; i< Zf_len ; i++)
		{
			int ii= 2*i ;
			Zf_Zf(ii) = Zf(0 ,i);
			Zf_Zf(ii+1) = Zf(1 ,i);
			RR.block(ii, ii, 2, 2)=RE;
		}
		
		//[x,P] = KF_IEKF_update(x,P, zz,RR, @hmodel, @hjacobian, N);
		Eigen::MatrixXd xo= x; //% prior values
		Eigen::MatrixXd Po= P;
		Eigen::MatrixXd Poi= P.inverse();
		Eigen::MatrixXd Ri= RR.inverse();
		

		Eigen::MatrixXd out_H;
		for(int i = 0; i < iter_num; i++)//% iterate solution
		{
			calculate_H_P(RR, Po, out_H);
			calculate_v_x(Zf_Zf, xo, Poi, out_H, Ri);
		}
		calculate_H_P(RR, Po, out_H);
	}
	void extend_kf::calculate_H_P(Eigen::MatrixXd in_R, Eigen::MatrixXd in_Po, Eigen::MatrixXd& out_H)
	{
		size_t Z_len = idf.size();
		size_t x_len = x.rows();
		out_H.resize(2*Z_len, x_len);
		out_H.setZero(2*Z_len, x_len);
		
		Eigen::MatrixXd H_temp, zp;
		for(size_t i=0 ; i<Z_len ; i++)
		{
			int ii= 2*i ;
			observe_model(idf[i], zp, H_temp);
			out_H.block(ii, ii, 2, 2) = H_temp;
		}
		
		Eigen::MatrixXd HP= out_H*in_Po;
		Eigen::MatrixXd PHt= in_Po*(out_H.transpose());
		Eigen::MatrixXd S= out_H*PHt + in_R;
		in_Po= in_Po - PHt * (S.inverse()) * HP;

		P= (in_Po + (in_Po.transpose()))/2.0; //for assurance
	}
	void extend_kf::calculate_v_x(Eigen::MatrixXd in_Z, Eigen::MatrixXd in_xo, Eigen::MatrixXd in_Poi, Eigen::MatrixXd in_H, Eigen::MatrixXd in_Ri)
	{
		size_t Z_len = idf.size();
		Eigen::MatrixXd v;
		v.resize(2*Z_len, 1);
		v.setZero(2*Z_len, 1);
		
		Eigen::MatrixXd H_temp, zp;
		for(size_t i = 0 ; i < Z_len ; i++ )
		{
			int ii= 2*i ;
			observe_model(idf[i], zp, H_temp);
			v(ii) = in_Z(ii) - zp(0);
			v(ii+1) = in_Z(ii+1) - zp(1);
			v(ii+1) = tool::normalize_angle( v(ii+1));
		}
		
		Eigen::MatrixXd M1= P *(in_H.transpose())* in_Ri; 
		Eigen::MatrixXd  M2= P * in_Poi * (x-in_xo);
		x= x + M1*v - M2;
	}
	void extend_kf::KF_cholesky_update(Eigen::MatrixXd v, Eigen::MatrixXd R, Eigen::MatrixXd H)
	{
		Eigen::MatrixXd PHt=P*H.transpose();
		Eigen::MatrixXd S=H*PHt+R;
		
		//make symmetric生成对角阵
		S= (S + ( S.transpose() ) )*0.5; 
		//SChol= chol(S);
		Eigen::MatrixXd SChol = ( S.llt().matrixL() ).transpose() ;//Cholesky分解法 A=LL^T ,matlab用的是上三角矩阵
		
		//triangular matrix
		Eigen::MatrixXd SCholInv= (SChol).inverse() ;  
		Eigen::MatrixXd W1= PHt * SCholInv;
		Eigen::MatrixXd W= W1 * ( SCholInv.transpose() );	//这里的W就是卡尔曼滤波的K增益参数
		
		//update
		x = x + W*v; 			//这里的v矩阵就是观测值与预测值的差
		P = P - W1* ( W1.transpose() );	
	}
	void extend_kf::augment(void)
	{
		Eigen::MatrixXd P_expansion;//大矩阵P的扩充矩阵
		Eigen::VectorXd x_expansion;//大矩阵x的扩充矩阵
		int Zn_len=Zn.cols();
		
		for(int i=0 ; i < Zn_len ; i++)
		{
			int x_len=x.rows();
			double r=Zn(0, i);
			double b=Zn(1, i);
			double s= sin(x(2)+b); 
			double c= cos(x(2)+b);
			
			//augment x,将新的landmark路标的x-y坐标加入状态矩阵
			x_expansion.resize(x_len+2, 1);
			x_expansion.setZero(x_len+2, 1);
			x_expansion.topRows(x_len) = x;
			x_expansion(x_len) = x(0) + r*c;
			x_expansion(x_len+1) = x(1) + r*s ;
			x.resize(x_len+2, 1);
			x = x_expansion;
			
			//jacobians
			Eigen::Matrix<double, 2, 3> Gv;
			Eigen::Matrix2d Gz;
			Gv<<1, 0, -r*s, 0, 1, r*c;
			Gz<<c, -r*s, s, r*c;
			
			//augment P
			P_expansion.resize(x_len+2, x_len+2);
			P_expansion.setZero(x_len+2, x_len+2);
			P_expansion.topLeftCorner(x_len, x_len) = P;
			P_expansion.block(x_len, x_len, 2, 2) = Gv* ( P.block(0, 0, 3, 3) ) * ( Gv.transpose() ) + Gz*RE* ( Gz.transpose() ); // feature cov
			P_expansion.block(x_len, 0, 2, 3) = Gv* ( P.block(0, 0, 3, 3) ); 									//vehicle to feature xcorr
			P_expansion.block(0, x_len, 3, 2) = ((P_expansion.block(x_len, 0, 2, 3) ).transpose());
			P.resize(x_len+2, x_len+2);
			P = P_expansion;
			if(x_len > 3)
			{//map to feature xcorr
				int rnm=x_len-3;
				P.block(x_len, 3, 2, rnm) = Gv*(P.block(0, 3, 3, rnm)); 
				P.block(3, x_len, rnm, 2) = ( (P.block(x_len, 3, 2, rnm)).transpose() );
			}
		}
	}
	void extend_kf::store_data(void)
	{
		STATE temp;
		int CHUNK= 5000;
		Eigen::Vector3d x_est;
		x_est.setZero(3, 1);
		
		if((OffLine_data.i) == (OffLine_data.estimate_path.size()) )
		{//增大容器,其实没有必要
// 			for(int i=0; i<CHUNK;i++)
// 			{
// 				OffLine_data.estimate_path.push_back(x_est);
// 				OffLine_data.real_path.push_back(x_est);
// 			}
		}
		OffLine_data.i = OffLine_data.i + 1;
		x_est = x.topRows(3);
		OffLine_data.estimate_path.push_back(x_est);
		OffLine_data.real_path.push_back(xtrue);
		temp.x = x;
		temp.P = P.diagonal();//获取对角线上的元素
		//temp.P << P(0, 0), P(1, 1), P(2, 2);
		OffLine_data.state.push_back(temp);
	}
	void extend_kf::transformtoglobal(Eigen::MatrixXd p, Eigen::Vector3d b, Eigen::MatrixXd& out_p)
	{
		//rotate
		Eigen::Matrix2d rot;
		rot<<cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));
		
		//FIXME 这里的这种规定动态矩阵大小的方法不知道正确与否
		out_p.resize(p.rows(), p.cols());
		out_p.setZero(p.rows(), p.cols());
		out_p.topRows(2)=rot*(p.topRows(2));
		
		//translate
		out_p.row(0)=out_p.row(0).array() + b(0);
		out_p.row(1)=out_p.row(1).array() + b(1);
		
		//if p is a pose and not a point
		//TODO MATLAB程序中并没有进入if语句里
// 		if((p.cols()) == 3)
// 			out_p.row(2) =(float) (tool::normalize_angle( (p.row(2)).array()  + b(2) ));
	}

	
	
	
	
	
	
}
