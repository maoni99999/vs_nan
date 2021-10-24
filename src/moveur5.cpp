#include <moveit/move_group_interface/move_group_interface.h>//16.04和18.04头文件有更改
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>

using namespace std;
#define PI (3.1415926535897932346f)
    

int main(int argc, char** argv)
{
    //double pt1[3]={-0.60,-0.00,0.31};//XYZ
    double pt1[3]={-0.63865,0.0020246,0.3102};//XYZ
    //double pt1[3]={-0.60,0,0.286};//XYZ

    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    
    
    // 创建一个异步的自旋线程（spinning thread）
    ros::AsyncSpinner spinner(1);
    ros::Rate rate(10);
    while(ros::ok())
    {
         spinner.start();

    moveit::planning_interface::MoveGroupInterface ur5_group("manipulator");
    ur5_group.setPoseReferenceFrame("base_link");
    //ur5.setPoseReferenceFrame("base_link");
    double org_x,org_y,org_z;
    org_x = pt1[0];
    org_y = pt1[1];
    org_z = pt1[2];


        
    geometry_msgs::Pose target_pose;
    target_pose.position.x = org_x; //位姿
    target_pose.position.y = org_y;
    target_pose.position.z = org_z;

    //Eigen::Vector3d ea(0-PI/122,PI/2,PI/2+PI/12);
    Eigen::Vector3d ea(0-PI/122,PI/2-PI/20,PI/2+PI/8);
    //Eigen::Vector3d ea(0,PI/2,PI/2);

    Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    target_pose.orientation.w = double(quaternion3.w());   //四元素
    target_pose.orientation.x = double(quaternion3.x());
    target_pose.orientation.y = double(quaternion3.y());
    target_pose.orientation.z = double(quaternion3.z());
    
    ur5_group.setPoseTarget(target_pose);  

       //group.move();
    moveit::planning_interface::MoveGroupInterface::Plan planner;
    ur5_group.plan(planner);
    ur5_group.move();
    rate.sleep();
    spinner.stop();



    }
   

    return 0;
}