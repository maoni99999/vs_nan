# include <vs_nan/floatlist.h>
#include <ros/ros.h>
#include <vs_nan/vs_message.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include "Eigen/Eigen"
//#include"Eigen/"
#include <iostream>
//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
//接收末端位姿

#include <moveit/move_group_interface/move_group_interface.h>//16.04和18.04头文件有更改
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>

using namespace cv;
using namespace std;
using namespace Eigen;


Point2f P1_for_error,P2_for_error,P3_for_error,P4_for_error;




class vs_based
{
public:
	//读到的
	
	Point2f lf_img_pt1, lf_img_pt2;
	Point2f rg_img_pt3, rg_img_pt4;
	Eigen::Matrix<float, 1, 3> lf_line_1, lf_line_2, rg_line_3, rg_line_4;//行向量
	//重建的结果
	Point3f lf_P1, lf_P2, rg_P3, rg_P4;
	
	
	//固定的
	//相机外参 要先给
	Eigen::Matrix3f Rl2r;
	Eigen::Vector3f tl2r;
	//相机内参 要先给
	Eigen::Matrix3f K_lf, K_rg;
	float fx_lf, fy_lf, fx_rg, fy_rg;
	//手眼标定 下面的还没算出来
	Eigen::Matrix3f Rb2lf;
	Eigen::Vector3f tb2lf;
	
	Eigen::Matrix3f Rb2rg;
	Eigen::Vector3f tb2rg;
	//tcp
	Eigen::Vector3f rt2lf;
	Eigen::Vector3f rt2rg;

	

	//构造函数
	vs_based();
	
	Point3f rebuild3d(Point2f p, Eigen::Matrix<float, 1, 3> line, Eigen::Matrix3f Ksame, Eigen::Matrix3f Kdiff, Eigen::Matrix3f R, Eigen::Vector3f t);
	Eigen::Matrix<float, 2, 3> comput_J(float fx, float fy, Point3f P);
	Eigen::Matrix<float, 4, 6> comput_J_ba(Eigen::Matrix<float, 2, 3> Ji, Eigen::Matrix<float, 2, 3> Ji_1);
	
	Eigen::Matrix<float, 3, 6> comput_M(Point3f P);
	Eigen::Matrix<float, 6, 6> comput_M_ba(Eigen::Matrix<float, 3, 6> Mi, Eigen::Matrix<float, 3, 6> Mi_1);
	Eigen::Matrix<float, 6, 6> comput_Q(Eigen::Vector3f rt2c);
	Eigen::Matrix<float, 6, 6> comput_W(Eigen::Matrix<float, 3, 3> Rb2c, Eigen::Vector3f tb2c);
	//计算Hc
	Eigen::Matrix<float, 4, 6> compute_Hc(float fcx, float fcy, Point3f Pc, Point3f Pc_1, Eigen::Matrix<float, 3, 1> rt2c, Eigen::Matrix<float, 3, 3> Rb2c, Eigen::Vector3f tb2c);
};



vs_based::vs_based()
{
	this->Rl2r << 
		  0.766044,  0,         -0.642788, 
          0,         1,         0,         
          0.642788,  0 ,         0.766044;//外参
	//单位m 
	this->tl2r <<  0.187939, 0,0.068404;//外参
	//左相机内参
	//640 512 f3629.62
	this->K_lf <<
		3629.6187, 0, 640.5,
		0, 3629.6187, 512.5   , 
		0, 0, 1;
	//右相机内参
	this->K_rg <<
		3629.6187, 0, 640.5,
		0, 3629.6187, 512.5   , 
		0, 0, 1;
	this->fx_lf = this->K_lf(0, 0);
	this->fy_lf = this->K_lf(1, 1);
	this->fx_rg = this->K_rg(0, 0);
	this->fy_rg = this->K_rg(1, 1);
//单位m 
	this->rt2lf<<0.100,0,0.010;
	this->rt2rg<<-0.100,0,0.010;


}

Point3f vs_based::rebuild3d(Point2f p, Eigen::Matrix<float, 1, 3> line, Eigen::Matrix3f Ksame, Eigen::Matrix3f Kdiff, Eigen::Matrix3f R, Eigen::Vector3f t) {
	Point3f result;
	Eigen::Vector3f p_ba, result_v;
	p_ba(0, 0) = p.x;
	p_ba(1, 0) = p.y;
	p_ba(2, 0) = 1.0;
	//cout<<"Z为   "<<-1*((line * Kdiff * t)(0,0) /(line * Kdiff * R * Ksame.inverse() * p_ba)(0,0))<<endl;
	result_v = -1*((line * Kdiff * t)(0,0) /(line * Kdiff * R * Ksame.inverse() * p_ba)(0,0))* Ksame.inverse() * p_ba;;
	result.x = result_v(0, 0);
	result.y = result_v(1, 0);
	result.z = result_v(2, 0);
	return result;
}

//计算J
Eigen::Matrix<float, 4, 6> vs_based::comput_J_ba(Eigen::Matrix<float, 2, 3> Ji, Eigen::Matrix<float, 2, 3>Ji_1)
{
	Eigen::Matrix<float, 4, 6> result;
	result.setZero();
	result.block<2, 3>(0, 0) = Ji;
	result.block<2, 3>(2, 3) = Ji_1;
	//cout<<"J_ba"<<result<<endl;
	return result;
}

Eigen::Matrix<float, 2, 3> vs_based::comput_J(float fx, float fy, Point3f P)
{
	Eigen::Matrix<float, 2, 3> result;
	result.setZero();
	result(0, 0) = fx / P.z;
	result(0, 2) = -fx * P.x / (P.z * P.z);

	result(1, 1) = fy / P.z;
	result(0, 2) = -fy * P.y / (P.z * P.z);
	//cout<<"J"<<result<<endl;
	return result;
}

//计算M
Eigen::Matrix<float, 3, 3> sk(Point3f P)
{
	Eigen::Matrix<float, 3, 3> result;
	result.setZero();
	result(0, 1) = -P.z;
	result(0, 2) = P.y;

	result(1,0)= P.z;
	result(1, 2) = -P.x;

	result(2, 0) = -P.y;
	result(2, 1) =P.x;

	return result;
}

Eigen::Matrix<float, 3, 6> vs_based::comput_M(Point3f P)
{
	Eigen::Matrix<float, 3, 6> result;
	result.setZero();
	Eigen::Matrix<float, 3, 3> fornt =  Eigen::MatrixXd::Identity(3, 3).cast<float>();
	fornt=-1 *fornt;
	result.block<3,3>(0,0)=fornt;
	result.block<3,3>(0, 3) = sk(P);
	cout<<"M"<<result<<endl;
	return result;
}

Eigen::Matrix<float, 6, 6> vs_based::comput_M_ba(Eigen::Matrix<float, 3, 6> Mi, Eigen::Matrix<float, 3, 6> Mi_1)
{
	Eigen::Matrix<float, 6, 6> result;
	result.setZero();
	result.block<3, 6>(0, 0) = Mi;
	result.block<3, 6>(3, 0) = Mi_1;
	return result;
}

Eigen::Matrix<float, 6, 6> vs_based::comput_Q(Eigen::Vector3f rt2c)
{
	Point3f rt2c_v;
	rt2c_v.x = rt2c(0,0);
	rt2c_v.y = rt2c(1,0);
	rt2c_v.z = rt2c(2,0);
	Eigen::Matrix<float, 6, 6> result;
	result.setIdentity();
	result.block<3, 3>(0, 3) = sk(rt2c_v);
	return result;
}


Eigen::Matrix<float, 6, 6> vs_based::comput_W(Eigen::Matrix<float, 3, 3> Rb2c, Eigen::Vector3f tb2c)
{
	Point3f rt2c_v;
	rt2c_v.x = tb2c(0.0);
	rt2c_v.y = tb2c(1.0);
	rt2c_v.z = tb2c(2.0);
	Eigen::Matrix<float, 6, 6> result;
	result.setZero();
	result.block<3, 3>(0, 0) = Rb2c;
	result.block<3, 3>(3, 3) = Rb2c;
	result.block<3, 3>(0, 3) = Rb2c*sk(rt2c_v);
	return result;
}


Eigen::Matrix<float, 4, 6> vs_based::compute_Hc(float fcx, float fcy, Point3f Pc, Point3f Pc_1, Eigen::Matrix<float, 3, 1> rt2c, Eigen::Matrix<float, 3, 3> Rb2c, Eigen::Vector3f tb2c)
{
	Eigen::Matrix<float, 4, 6> J_ba;
	J_ba = this->comput_J_ba(this->comput_J(fcx, fcy, Pc), this->comput_J(fcx, fcy, Pc_1));
	Eigen::Matrix<float, 6, 6> M_ba;
	M_ba = this->comput_M_ba(this->comput_M(Pc), this->comput_M(Pc_1));
	Eigen::Matrix<float, 6, 6> Qc;
	Qc = this->comput_Q(rt2c);
	Eigen::Matrix<float, 6, 6> Wc;
	Wc = this->comput_W(Rb2c, tb2c);
	Eigen::Matrix<float, 4, 6> result;
	result = J_ba * M_ba * Qc * Wc;
	cout<<"jacobian j:"<<endl<<J_ba * M_ba<<endl;
	cout<<"Wc"<<endl<<Wc<<endl;
	return result;
}


Point2f dev_p1,dev_p2,dev_p3,dev_p4;
vs_based source;

Matrix<float,4,4> Tt2lf,Tt2rg,Tb2lf,Tb2rg;

// 建立回调函数，当ros::spin()被触发时，该函数被激活，处理接收到的消息
void vs_callback(const vs_nan::vs_message& vs_msg){

    /*
		缺少
	*/
	//point2f
	dev_p1=Point2f(vs_msg.target_lf_P1_x,vs_msg.target_lf_P1_y);
	dev_p2=Point2f(vs_msg.target_lf_P2_x,vs_msg.target_lf_P2_y);
	dev_p3=Point2f(vs_msg.target_rg_P3_x,vs_msg.target_rg_P3_y);
	dev_p4=Point2f(vs_msg.target_rg_P4_x,vs_msg.target_rg_P4_y);
	//point2f
	source.lf_img_pt1=Point2f(vs_msg.lf_P1_x,vs_msg.lf_P1_y);
	source.lf_img_pt2=Point2f(vs_msg.lf_P2_x,vs_msg.lf_P2_y);
	source.rg_img_pt3=Point2f(vs_msg.rg_P3_x,vs_msg.rg_P3_y);
	source.rg_img_pt4=Point2f(vs_msg.rg_P4_x,vs_msg.rg_P4_y);
	//matrix 1,3
    
	Matrix<float,1,3> line1,line2,line3,line4;
    line1<<vs_msg.lf_line1[0],vs_msg.lf_line1[1],vs_msg.lf_line1[2];
    line2<<vs_msg.lf_line2[0],vs_msg.lf_line2[1],vs_msg.lf_line2[2];
    line4<<vs_msg.rg_line1[0],vs_msg.rg_line1[1],vs_msg.rg_line1[2];
    line3<<vs_msg.rg_line2[0],vs_msg.rg_line2[1],vs_msg.rg_line2[2];

	source.lf_line_1=line1;
	source.lf_line_2=line2;
	source.rg_line_3=line3;
	source.rg_line_4=line4;

	P1_for_error=Point2f(vs_msg.lf_P1_x_for_error,vs_msg.lf_P1_y_for_error);
	P2_for_error=Point2f(vs_msg.lf_P2_x_for_error,vs_msg.lf_P2_y_for_error);
	P3_for_error=Point2f(vs_msg.rg_P3_x_for_error,vs_msg.rg_P3_y_for_error);
	P4_for_error=Point2f(vs_msg.rg_P4_x_for_error,vs_msg.rg_P4_y_for_error);
}

//单位m 
int main(int argc, char** argv)
{
	stringstream error_file;
    string file_road="/home/yiixn/servo_ws/result/";
    string file_name="shoubiaomubiaoweizi325shuju.txt";
    error_file<<file_road<<file_name<<endl;
    ofstream OpenFile(error_file.str());
    if (OpenFile.fail())
    {
        cout<< "Fail to open file"<<endl;
        return -1;
    }








	Tt2lf<<
-0.939693,         0,  -0.34202,       0.100,
        0,        -1,         0,         0,
 -0.34202,         0,  0.939693,        0.010,
        0,         0,         0,         1;
Tt2rg<<
-0.939693,         0,   0.34202,      -0.100,
        0,        -1,         0,         0,
  0.34202,         0,  0.939693,        0.010,
        0,         0,         0,         1;

    ros::init(argc, argv, "v_compute");//节点名字
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("vs_topic",1,vs_callback);

	vs_nan::floatlist velocity;
 	ros::Publisher pub = nh.advertise<vs_nan::floatlist>("ur5_end_velocities",1);


    //接收末端位姿
	ros::AsyncSpinner spinner(1);
    moveit::planning_interface::MoveGroupInterface ur5_group("manipulator");
    ur5_group.setPoseReferenceFrame("/base");

	
    

	tf::TransformListener listener;
	tf::StampedTransform transform;
	ros::Rate rate(1.0);
	// ImageConverter ic;1
    while(ros::ok())
    {
		//cout<<source.lf_img_pt1.x<<source.lf_img_pt1.y<<endl;
		ros::spinOnce();
		//3D重建
		
		source.lf_P1=source.rebuild3d(source.lf_img_pt1,source.rg_line_4,source.K_lf,source.K_rg,source.Rl2r,source.tl2r);
		source.lf_P2=source.rebuild3d(source.lf_img_pt2,source.rg_line_3,source.K_lf,source.K_rg,source.Rl2r,source.tl2r);
		source.rg_P3=source.rebuild3d(source.rg_img_pt3,source.lf_line_2,source.K_rg,source.K_lf,source.Rl2r.inverse(),-source.Rl2r.inverse()*source.tl2r);
		source.rg_P4=source.rebuild3d(source.rg_img_pt4,source.lf_line_1,source.K_rg,source.K_lf,source.Rl2r.inverse(),-source.Rl2r.inverse()*source.tl2r);
		cout<<"The result of rebuild:"<<endl;
		cout<<"lf_P1:"<<source.lf_P1.x<<","<<source.lf_P1.y<<","<<source.lf_P1.z<<endl;
		cout<<"lf_P2:"<<source.lf_P2.x<<","<<source.lf_P2.y<<","<<source.lf_P2.z<<endl;
		cout<<"rg_P3:"<<source.rg_P3.x<<","<<source.rg_P3.y<<","<<source.rg_P3.z<<endl;
		cout<<"rg_P4:"<<source.rg_P4.x<<","<<source.rg_P4.y<<","<<source.rg_P4.z<<endl;

		//cout<<"x"<<source.lf_P1.x<<"y"<<source.lf_P1.y<<"z"<<source.lf_P1.z<<endl;
		//计算H 先算右边的
		Eigen::Matrix<float, 4, 6> Hr,Hl;

        //读取当前相机的位置
		spinner.start();
        //cout<<"Current Pose"<<ur5_group.getCurrentPose()<<endl;
		geometry_msgs::Pose current_pose;
		current_pose=ur5_group.getCurrentPose().pose;
		



		spinner.stop();

		Eigen::Quaterniond quaternion(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);
        Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);//zyx
        //cout<<"eular angle"<< eulerAngle<<endl;
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix=quaternion.matrix();
        Eigen::Vector3f t;
		//单位m 
        t<<current_pose.position.x,current_pose.position.y,current_pose.position.z;
        Matrix<float,4,4> T=Eigen::MatrixXd::Identity(4, 4).cast<float>();
        T.block<3,3>(0,0)=rotation_matrix.cast<float>();
        T.block<3, 1>(0, 3) = t;
        //末端的坐标变换
        Eigen::Vector3d T_eulrt_trans(M_PI/2,0,M_PI/2);
        Eigen::AngleAxisd rollAngle(AngleAxisd(T_eulrt_trans(2),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(AngleAxisd(T_eulrt_trans(1),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(AngleAxisd(T_eulrt_trans(0),Vector3d::UnitZ()));
        Eigen::Matrix3d T_rotation;
        T_rotation=yawAngle*pitchAngle*rollAngle;

         Eigen::Vector3d eulershow=T_rotation.eulerAngles(2,1,0);
         //cout<<"euler"<<eulershow<<endl;
         //cout<<"euler1"<<T_rotation<<endl;
        Matrix<float,4,4> T_transform=Eigen::MatrixXd::Identity(4, 4).cast<float>();
        T_transform.block<3,3>(0,0)=T_rotation.cast<float>();

		// 机械臂读到的 +机械臂的坐标系转换+熟悉的末端到相机
        Tb2lf=T*T_transform*Tt2lf;
        Tb2rg=T*T_transform*Tt2rg;
        //cout<< "left pose"<<Tb2lf<<endl;
        //cout<< "right pose"<<Tb2rg<<endl;		
        source.Rb2rg=Tb2rg.block<3, 3>(0, 0);
	    source.tb2rg=Tb2rg.block<3,1>(0,3);
		//读取当前相机的位置
        source.Rb2lf=Tb2lf.block<3, 3>(0, 0);
	    source.tb2lf=Tb2lf.block<3, 1>(0, 3);;

		
		//Hr=source.compute_Hc(source.fx_rg,source.fy_rg,source.rg_P3,source.rg_P4,source.rt2rg,source.Rb2rg,source.tb2rg);
		Hr=source.compute_Hc(source.fx_rg,source.fy_rg,source.rg_P3,source.rg_P4,-Tt2rg.block<3,3>(0,0).transpose()*source.rt2rg,source.Rb2rg.transpose(),-source.tb2rg);
		cout<<"Hr"<<Hr<<endl;
		Hr=Hr/100;
		cout<<"Hr"<<Hr<<endl;
		Hl=source.compute_Hc(source.fx_lf,source.fy_lf,source.lf_P1,source.lf_P2,-Tt2lf.block<3,3>(0,0).transpose()*source.rt2lf,source.Rb2lf.transpose(),-source.tb2lf);
		Hl=Hl/100;

		Eigen::Matrix<float,4,1>error_rg,error_lf;

		// error_rg(0,0)=P3_for_error.x-dev_p3.x;
		// error_rg(1,0)=P3_for_error.y-dev_p3.y;
		// error_rg(2,0)=P4_for_error.x-dev_p4.x;
		// error_rg(3,0)=P4_for_error.y-dev_p4.y;

		// error_lf(0,0)=P1_for_error.x-dev_p1.x;
		// error_lf(1,0)=P1_for_error.y-dev_p1.y;
		// error_lf(2,0)=P2_for_error.x-dev_p2.x;
		// error_lf(3,0)=P2_for_error.y-dev_p2.y;
		//计算末端速度
		// cout<<"error:"<<endl;
		// cout<<"____right cam_____"<<endl;
		// cout<<error_rg<<endl;
		// cout<<"____left  cam_____"<<endl;
		// cout<<error_lf<<endl;

		Eigen::Matrix<float,6,1> u;
		//方法1 386-408
		Eigen::Matrix<float,8,1>error; 
		error(0,0)=P3_for_error.x-dev_p3.x;
		error(1,0)=P3_for_error.y-dev_p3.y;
		error(2,0)=P4_for_error.x-dev_p4.x;
		error(3,0)=P4_for_error.y-dev_p4.y;
		error(4,0)=P1_for_error.x-dev_p1.x;
		error(5,0)=P1_for_error.y-dev_p1.y;
		error(6,0)=P2_for_error.x-dev_p2.x;
		error(7,0)=P2_for_error.y-dev_p2.y;
		Eigen::Matrix<float, 8, 6> H;
		H.setZero();
		H.block<4, 6>(0, 0) = Hr;
		H.block<4, 6>(4, 0) = Hl;
		cout<<"H为、/100了"<<endl;
		cout<<H<<endl;
		MatrixXf Ht = H.transpose();
		//cout<<"对角线"<<(Ht*H).inverse() <<endl;
		//cout<<"对角线"<<(Ht*H).diagonal() <<endl;
		MatrixXf H_Diag((Ht*H).diagonal().asDiagonal());
		cout<<"HT*T ' DIAG:"<<endl<<H_Diag<<endl;
		H_Diag=0.3*H_Diag+(Ht*H);
		
		cout<<"H_pinv"<<H_Diag<<endl;
		u = H_Diag.inverse()*Ht*error;
		u=-u/100;
		cout<<"gailemei a "<<endl;




		//论文中的方法3////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法//
		// Eigen::Matrix<float,6,6>P;
		// Eigen::Matrix<float,6,6> I =MatrixXd::Identity(6, 6).cast<float>();
		// if(sqrt(double(pow(error_rg(0,0),2)+pow(error_rg(1,0),2)+pow(error_rg(2,0),2)+pow(error_rg(3,0),2)))>=25
		// 		|| sqrt(double(pow(error_lf(0,0),2)+pow(error_lf(1,0),2)+pow(error_lf(2,0),2)+pow(error_lf(3,0),2)))>=25)
		// {
		// 	//Hl=source.compute_Hc(source.fx_lf,source.fy_lf,source.lf_P1,source.lf_P2,-source.rt2lf,source.Rb2lf,source.tb2lf);
		// 	//Hl=Hl/100;
			
		// 	P=I-Hr.transpose()*(Hr*Hr.transpose()).inverse()*Hr;
		// 	u=-0.3*Hr.transpose()*(Hr*Hr.transpose()).inverse()*error_rg
		// 		+P*(Hl*P).transpose()*((Hl*P)*(Hl*P).transpose()).inverse()*(-0.3*error_lf-Hl*Hr.transpose()*(Hr*Hr.transpose()).inverse()
		// 		*(-0.3*error_rg));
		// 	u=u/100;

		// }
		// else
		// {
		// 	 	Matrix<float,6,1> zero =MatrixXd::Zero(6, 1).cast<float>();
		// 	 	u=zero;
		// }

		//论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法////论文中的方法//


		//第二种方法
		//右边的误差和>3          
		// if(sqrt(double(pow(error_rg(0,0),2)+pow(error_rg(1,0),2)+pow(error_rg(2,0),2)+pow(error_rg(3,0),2)))>25)
		// {

		// 	MatrixXf Hr_Diag((Hr*Hr.transpose()).diagonal().asDiagonal());
		// 	 cout<<"Hr_Diag"<<Hr_Diag<<endl;
		// 	Hr_Diag=3*Hr_Diag+Hr*Hr.transpose();
		// 	u = Hr.transpose()*Hr_Diag.inverse()*error_rg;
		// 	//u=0.3*Hr.transpose()*(Hr*Hr.transpose()).inverse()*error_rg;
		// 	u=u/100;
			
		// 	// cout<<"机械臂末端运动速度"<<u.transpose()<<endl;
		// 	// cout<<(Hr*Hr.transpose()).inverse()<<endl;
		// }//右边的误差和<=3 左边的>3
		// else if(sqrt(double(pow(error_rg(0,0),2)+pow(error_rg(1,0),2)+pow(error_rg(2,0),2)+pow(error_rg(3,0),2)))<=25&&
		// sqrt(double(pow(error_lf(0,0),2)+pow(error_lf(1,0),2)+pow(error_lf(2,0),2)+pow(error_lf(3,0),2)))>3
		// )
		// {	
		// 	Eigen::Matrix<float,6,6>P;
		// 	Eigen::Matrix<float,6,6> I =MatrixXd::Identity(6, 6).cast<float>();
		// 	P=I-Hr.transpose()*(Hr*Hr.transpose()).inverse()*Hr;

		// 	// Hl=source.compute_Hc(source.fx_lf,source.fy_lf,source.lf_P1,source.lf_P2,-source.rt2lf,source.Rb2lf,source.tb2lf);
		// 	// Hl=Hl/100;
		// 	u=-0.3*P*(Hl*P).transpose()*((Hl*P)*(Hl*P).transpose()).inverse()*error_lf;
		// 	u=u/100;
		// 	cout<<"机械臂末端运动速度"<<u.transpose()<<endl;
		// }//左右都<3 完成
		// else if(sqrt(double(pow(error_rg(0,0),2)+pow(error_rg(1,0),2)+pow(error_rg(2,0),2)+pow(error_rg(3,0),2)))<=3&&
		// sqrt(double(pow(error_lf(0,0),2)+pow(error_lf(1,0),2)+pow(error_lf(2,0),2)+pow(error_lf(3,0),2)))<=3)
		// {
		// 	Matrix<float,6,1> zero =MatrixXd::Zero(6, 1).cast<float>();
		// 	u=zero;
		// }
		// else{
		// 	Matrix<float,6,1> zero =MatrixXd::Zero(6, 1).cast<float>();
		// 	u=zero;
		// }
			
		//发送 u
		cout<<"速度是"<<endl;
		cout<<u<<endl;
		velocity.data={u(0,0),u(1,0),u(2,0),u(3,0),u(4,0),u(5,0)};
		pub.publish(velocity);
		rate.sleep();

		   	try{
      	listener.lookupTransform("/base_link", "/object_up_link",
                               ros::Time(0), transform);
    	}
    	catch (tf::TransformException &ex) {
      	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
      	continue;
    }
		 OpenFile<<transform.getOrigin().x()<<","<<transform.getOrigin().y()<<","<<transform.getOrigin().z()<<","
            <<transform.getRotation().getW()<<","<<transform.getRotation().getX()<<","<<transform.getRotation().getY()<<","
			<<transform.getRotation().getZ()<<"\n";
		//  OpenFile<<current_pose.position.x<<","<<current_pose.position.y<<","<<current_pose.position.z<<","
        //     <<current_pose.orientation.w<<","<<current_pose.orientation.x<<","<<current_pose.orientation.y<<","<<current_pose.orientation.z<<"\n";
            //OpenFile.close();
    }
	
    return 0;
}
