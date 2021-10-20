#include <ros/ros.h>
#include "Eigen/Eigen"

#include <moveit/move_group_interface/move_group_interface.h>//16.04和18.04头文件有更改
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include<ctime>

#include <math.h>
#include <stdio.h>
using namespace std;
using namespace Eigen;

//测试用
clock_t start,stop;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_pose");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    moveit::planning_interface::MoveGroupInterface ur5_group("manipulator");
    ur5_group.setPoseReferenceFrame("base_link");
    ros::Rate rate(10);
    while(ros::ok())
    {
        //计算器
        Matrix<float,4,4> Tt2lf1,Tt2rg1,Tt2lf2,Tt2rg2 ,Tt2lfn,Tt2rgn,Tlf2rg;
        Tt2lf1<<
        -1, 0,  0,  100,
        0,  -1, 0,  0,
        0,  0,  1,  10,
        0,  0,  0,  1;

        Tt2rg1<<
        -1, 0,  0,  -100,
        0,  -1, 0,  0,
        0,  0,  1,  10,
        0,  0,  0,  1; 


        Tt2lf2<<
        cos(M_PI/9), 0, sin(M_PI/9),0,
        0,            1, 0,            0,
        -sin(M_PI/9),0, cos(M_PI/9), 0,
        0,0,0,1;

        Tt2rg2<<
        cos(-M_PI/9), 0, sin(-M_PI/9),   0,
        0,            1, 0,            0,
        -sin(-M_PI/9),0, cos(-M_PI/9),   0,
        0,0,0,1;

        Tt2lfn=Tt2lf1*Tt2lf2;
        Tt2rgn=Tt2rg1*Tt2rg2;
        Tlf2rg=Tt2lfn.inverse()*Tt2rgn;

        cout<<"Tt2lfn"
        <<Tt2lfn
        <<"Tt2rgn"
        <<Tt2rgn
        <<"Tlf2rg"
        <<Tlf2rg<<endl;



        geometry_msgs::Pose current_pose;
        ros::spinOnce();
        start=clock();
        spinner.start();
        //getCurrentPose读到的坐标系和常规的和gazebo中画出来的也不一样
        current_pose=ur5_group.getCurrentPose().pose;
        cout<<"pose_msg"<<current_pose<<endl;
        spinner.stop();
        
        
        Eigen::Quaterniond quaternion(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);
        Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);//zyx
        cout<<"eular angle"<< eulerAngle<<endl;
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix=quaternion.matrix();
        Eigen::Vector3f t;
        t<<current_pose.position.x*1000,current_pose.position.y*1000,current_pose.position.z*1000;
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
         cout<<"euler"<<eulershow<<endl;
         cout<<"euler1"<<T_rotation<<endl;
        Matrix<float,4,4> T_transform=Eigen::MatrixXd::Identity(4, 4).cast<float>();
        T_transform.block<3,3>(0,0)=T_rotation.cast<float>();
        //相机和末端的关系
        Matrix<float,4,4> Tt2lf,Tt2rg,Tb2lf,Tb2rg;
        // Tt2lf<<-1,0,0,-100,
        //         0,-1,0,0,
        //         0,0,1,10,
        //         0,0,0,1;
        //  Tt2rg<<-1,0,0,100,
        //         0,-1,0,0,
        //         0,0,1,10,
        //         0,0,0,1;   
        Tb2lf=T*T_transform*Tt2lfn;
        Tb2rg=T*T_transform*Tt2rgn;
        cout <<"current pose "<<T*T_transform<<endl;
        cout<< "left pose"<<Tb2lf<<endl;
        cout<< "right pose"<<Tb2rg<<endl;

        //cout<<T<<endl;
        stop=clock();
        double endtime=(double)(stop-start)/CLOCKS_PER_SEC;
	    cout<<"Total time:"<<endtime<<endl;	
        rate.sleep();
    }
    return 0;
}