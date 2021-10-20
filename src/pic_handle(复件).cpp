# include <ros/ros.h>
# include <vs_nan/vs_message.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "std_msgs/Empty.h"


#include "opencv2/core/utility.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

//记录位置点的信息
#include <sstream>
#include <string>
#include<fstream>  

using namespace std;
using namespace cv;

Mat left_raw_img,right_raw_img;
Mat showlf,showrg;
Point2f target0,target1,target2,target3;

Point2f pic_pt0_forerror,pic_pt1_forerror,pic_pt2_forerror,pic_pt3_forerror;
Point2f pic_pt0_for3d,pic_pt1_for3d,pic_pt2_for3d,pic_pt3_for3d;
//vector<Point2f> pt0_four_feature_point,pt1_four_feature_point,pt2_four_feature_point,pt3_four_feature_point;
float pt0_line[3],pt1_line[3],pt2_line[3],pt3_line[3];
Scalar blue_low=Scalar(100,43,46),red_low=Scalar(0,43,46);
Scalar blue_high=Scalar(124,255,255),red_high=Scalar(10,255,255);


//检测是不是第一次
int flag=1;


void cam_left(const sensor_msgs::ImageConstPtr& msg)
{
    //int count = 0;
    //char t[100];
    //sprintf(t,"%08d",count);
        
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  //bgr8
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    left_raw_img = cv_ptr->image.clone();
    //cv::imshow("left_img", left_raw_img);
    //cv::imwrite(std::string("/home/osgoodwu/save_stereo_workspace/left")+t+std::string(".png"),frame);
   // count++;
    //cv::waitKey(3);
}

void cam_right(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  //bgr8
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    right_raw_img = cv_ptr->image.clone();
    // cv::imshow("right", right_raw_img);
    // cv::waitKey(3);
}


float getDistance(Point2f pointO, Point2f pointA)
{
	float distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);
	return distance;
}


//（子函数）给定两个点    得到数组ABC
void CaculateLine(Point2f &p0,Point2f &p1,float * a){
    a[0]=p1.y-p0.y;
    a[1]=p0.x-p1.x;
    a[2]=p1.x*p0.y-p0.x*p1.y;
}
//（子函数）计算点目标点  GreenPoint Line[3] length长度 flag判断左右 flag=0 左  FLAG=1右
void CaculatePoint(Point2f& GreenPoint,float * Line,float length,int flag,Point2f& TargetPoint){
    float newline[3];
    newline[0]=Line[1];
    newline[1]=-Line[0];
    newline[2]=Line[2];
    if(newline[1]>=0&&flag==0){
        for(int i=0;i<3;i++){
            newline[i]=-newline[i];
        }
    }else if (newline[1]<=0&&flag==1)
    {
        for(int i=0;i<3;i++){
            newline[i]=-newline[i];
        }
    }

    TargetPoint.x=GreenPoint.x+length*newline[1]/sqrtf(powf(newline[0], 2)+powf(newline[1], 2));
    TargetPoint.y=GreenPoint.y+length*newline[0]/sqrtf(powf(newline[0], 2)+powf(newline[1], 2));



}

//第一次的时候计算 目标点      流程
void CaculateTargetPoint(Mat& img,Point2f &up_point,Point2f &down_point,float length,int flag){
     //检测绿色圆心
    Scalar low =Scalar(35,43,46);
    Scalar high =Scalar(77,255,255);
    Mat hsv;
    Mat img_clone=img.clone();
    Mat img_flag;
    cvtColor(img_clone,hsv,CV_BGR2HSV);
    inRange(hsv,low,high,img_flag);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::findContours(img_flag, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    
    //检测椭圆圆心
    vector<Point2f> greenpoint;
    for(int i=0;i< contours.size();i++)
    {
        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);
        //cout << box.center;
        greenpoint.push_back(box.center);
    }
    //排上下
    Point2f circle_point1,circle_point2;
    if(greenpoint[0].y<greenpoint[1].y){
        circle_point1=greenpoint[0];
        circle_point2=greenpoint[1];
    }
    else{
        circle_point1=greenpoint[1];
        circle_point2=greenpoint[0];
    }
    cout<<"circle_point1"<<circle_point1<<"\n"<<"circle_point2"<<circle_point2<<endl;

    float line_1[3];
    CaculateLine(circle_point1, circle_point2 ,line_1);
    

    CaculatePoint(circle_point1,line_1,length,flag,up_point);
    CaculatePoint(circle_point2,line_1,length,flag,down_point);

}

//计算源点 用于重建的坐标点
void CaculateSourcePoint(Mat& img,Scalar low,Scalar high,Point2f &SourcePoint, vector<Point2f> &four_feature_point ){
    Mat hsv;
    Mat img_clone=img.clone();
    Mat img_flag;
    cvtColor(img_clone,hsv,CV_BGR2HSV);
    inRange(hsv,low,high,img_flag);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::findContours(img_flag, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    double maxarea = -1;
	int maxAreaIdx = -1;
    for (int i = 0; i < contours.size(); i++)
	{
		double tmparea = contourArea(contours[i]);
		if (tmparea > maxarea)
		    {
			        maxarea = tmparea;
		            maxAreaIdx = i;//记录最大轮廓的索引号
		    }
	}
    //测试：画出最大轮廓
	if (maxAreaIdx != -1) {
		cv::drawContours(img_clone, contours, maxAreaIdx, (0, 0, 255), 2, 8);
	}
    imshow("轮廓",img_clone);
    //waitKey(0);
    vector<Point> c = contours[maxAreaIdx];

    vector<Point2f> polypoints;
	cv::approxPolyDP(c, polypoints, 4, true);
    
    Mat labels, centers;//labels是编号 center是点
    if (polypoints.size() >= 4) {
        //polypoints 2维 矩阵 ;labels数据点的分类索引; centers分类中心的中心点数据
	    cv::kmeans(Mat(polypoints), 4, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0),
        5, cv::KMEANS_RANDOM_CENTERS, centers);
	}
    //将center转换为vector<Point2f> 四个角点～
	vector<Point2f> pixel_points4;//四个点
	for (int i = 0; i < centers.rows; i++) {
        //cv::drawMarker(frame, Point(int(centers.at<float>(i, 0)), int(centers.at<float>(i, 1))), (255, 255, 255), 1, 20, 5);
		pixel_points4.push_back(Point2f(centers.at<float>(i, 0), centers.at<float>(i, 1)));
        //cout << pixel_points4 << endl << endl;
    }

     //pixel_points4为四个角点，逆时针排列 我们将寻找左上角的点作为起始点
    vector<int> labels_lst={labels.at<int>(0,0)};//第一个点的label
    vector<int>::iterator ret;
    for (int i = 0; i < labels.rows; i++) {
		ret = std::find(labels_lst.begin(), labels_lst.end(), labels.at<int>(0, i));
		if (ret == labels_lst.end()) {
				//not find
				labels_lst.push_back(labels.at<int>(i, 0));
		}
	}
    //顶点顺序重构，多边形定点顺序为逆时针
    //先找最小值，x最小值
    vector<Point2f> pixel_points4_regroup;
    int x_min_idx;
	float x_min = 9999.0;
	for (int i = 0; i < labels_lst.size(); i++) {
	    pixel_points4_regroup.push_back(pixel_points4[labels_lst[i]]);
	    if (pixel_points4_regroup[i].x < x_min) {
		//找pixel_points4_regroup中的最小x的索引
		x_min = pixel_points4_regroup[i].x;
		x_min_idx = i;
		}
	}
      //横坐标小的点要么是上面的要么是下面的
    Point2f pt1= pixel_points4_regroup[x_min_idx];
    //起始点逆时针下一个点
	Point2f pt2 = pixel_points4_regroup[(x_min_idx + 1) % 4];
    //起始点逆时针上一个点
	Point2f pt3 = pixel_points4_regroup[(x_min_idx + 3) % 4];
    //比y大小



    if(pt1.y<pt2.y){
        for (int i = 0; i < 4; i++) {
			four_feature_point.push_back(pixel_points4_regroup[(x_min_idx  + i) % 4]);
		}
    }
    else{
        for (int i = 0; i < 4; i++) {
		four_feature_point.push_back(pixel_points4_regroup[(x_min_idx  + 3+i) % 4]);
		}
    }
    float x=0,y=0;
    for(int i=0;i<four_feature_point.size();i++){
        x+=four_feature_point[i].x;
        y+=four_feature_point[i].y;
    }
    SourcePoint.x=x/4;
    SourcePoint.y=y/4;
    cout<<"测试：targetpoint"<<SourcePoint<<endl;
}
//flag=0123 分别是四个点
void Caculate3Drebuild(vector<Point2f> &four_feature_point,Point2f &source2Dpointfor3d,float * rebuildlinefor3d,int flag){
    if(four_feature_point.size()!=4){
        cout<<"wrong"<<endl;
        
    }
    if(flag==0){
        source2Dpointfor3d=four_feature_point[0];
        CaculateLine(four_feature_point[0],four_feature_point[3],rebuildlinefor3d);

    }else if(flag==1){
        source2Dpointfor3d=four_feature_point[1];
        CaculateLine(four_feature_point[1],four_feature_point[2],rebuildlinefor3d);
    }else if(flag==2){
        source2Dpointfor3d=four_feature_point[2];
        CaculateLine(four_feature_point[1],four_feature_point[2],rebuildlinefor3d);

    }else if(flag==3) {
        source2Dpointfor3d=four_feature_point[3];
        CaculateLine(four_feature_point[0],four_feature_point[3],rebuildlinefor3d);

    }
}






int main(int argc, char **argv)
{
    stringstream error_file;
    string file_road="home/yiixn/servo_ws/result/";
    string file_name="1.txt";
    error_file<<file_road<<file_name<<endl;
    ofstream OpenFile(error_file.str());
    if (OpenFile.fail())
    {
        cout<< "Fail to open file"<<endl;
        //return -1;
    }

    ros::init(argc, argv, "pic_handle");//节点
    ros::NodeHandle nh;
    // ros::NodeHandle nh("~");
    vs_nan::vs_message vs_msg;
    ros::Publisher pub = nh.advertise<vs_nan::vs_message>("vs_topic",1);

    //定义接收的
    image_transport::ImageTransport it_(nh);
    image_transport::Subscriber left_sub;
    image_transport::Subscriber right_sub;
    left_sub = it_.subscribe("camera1/image_raw1", 1, cam_left);
    right_sub = it_.subscribe("camera2/image_raw2", 1, cam_right);



    ros::Rate rate(1);
    while (ros::ok())
    {

        ros::spinOnce();
        if(left_raw_img.empty()||right_raw_img.empty())
        {
            cout<< "without img"<<endl;
            continue;
        }
        else{
            showlf=left_raw_img.clone();
            showrg=right_raw_img.clone();
        }
        rate.sleep();
        
        // waitKey(1000);
        // cv::imshow("left_img", left_raw_img);
        //cv::imwrite(std::string("/home/osgoodwu/save_stereo_workspace/left")+t+std::string(".png"),frame);
        // count++;
        //cv::waitKey(3);
        //是否已经提取过要跟踪的对象
         if(flag==1)//  计算4个点target点 
        {
            //flag =0左边
            CaculateTargetPoint(left_raw_img,target0,target1,121.333,0);
            CaculateTargetPoint(right_raw_img,target3,target2,121.333,1);
            OpenFile<<"target point left 01 right01"<<"\n"
            <<target0.x<<","<<target0.y<<"\n"
            <<target1.x<<","<<target1.y<<"\n"
            <<target2.x<<","<<target2.y<<"\n"
            <<target3.x<<","<<target3.y<<"\n"<<"\n";
            // cv::drawMarker(showrg, Point(int(target3.x), int(target3.y)), (255, 0, 0), 2, 1, 5);//上面的
            // cv::drawMarker(showrg, Point(int(target2.x), int(target2.y)), (0, 255, 0), 2, 1, 5);//下面的 
            // cv::imshow("right_raw_img",showrg);
            // cv::drawMarker(showlf, Point(int(target0.x), int(target0.y)), (255, 0, 0), 2, 1, 5);//上面的
            // cv::drawMarker(showlf, Point(int(target1.x), int(target1.y)), (0, 255, 0), 2, 1, 5);//下面的 
            // cv::imshow("left_raw_img",showlf);
            
            flag =0;
            // //测试
            // cv::circle(left_raw_img, target0, 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径 
            // cv::circle(left_raw_img, target1, 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径 
            // cv::circle(right_raw_img, target2, 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径 
            // cv::circle(right_raw_img, target3, 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径 
            // imshow("left_target",left_raw_img);
            // imshow("right_target",right_raw_img);
            // waitKey(0);
            // cout<<"提取绿色角点成功"<<endl;测试
        }
        vector<Point2f> pt0_four_feature_point,pt1_four_feature_point,pt2_four_feature_point,pt3_four_feature_point;

        CaculateSourcePoint(left_raw_img,blue_low,blue_high,pic_pt0_forerror,pt0_four_feature_point );
        Caculate3Drebuild(pt0_four_feature_point,pic_pt0_for3d,pt0_line,0);
        cv::drawMarker(showlf, Point(int(pic_pt0_for3d.x), int(pic_pt0_for3d.y)), (255, 0, 0), 2, 1, 5);//上面的
        // cv::imshow("left_raw_img",showlf);
        // waitKey(0);

        CaculateSourcePoint(left_raw_img,red_low,red_high,pic_pt1_forerror,pt1_four_feature_point );
        Caculate3Drebuild(pt1_four_feature_point,pic_pt1_for3d,pt1_line,1);
        cv::drawMarker(showlf, Point(int(pic_pt1_for3d.x), int(pic_pt1_for3d.y)), (255, 0, 0), 2, 1, 5);//上面的
        //cv::imshow("left_raw_img",showlf);
        //waitKey(0)

        CaculateSourcePoint(right_raw_img,red_low,red_high,pic_pt2_forerror,pt2_four_feature_point );
        Caculate3Drebuild(pt2_four_feature_point,pic_pt2_for3d,pt2_line,2);
        cv::drawMarker(showrg, Point(int(pic_pt2_for3d.x), int(pic_pt2_for3d.y)), (255, 0, 0), 2, 1, 5);//上面的

        CaculateSourcePoint(right_raw_img,blue_low,blue_high,pic_pt3_forerror,pt3_four_feature_point );
        Caculate3Drebuild(pt3_four_feature_point,pic_pt3_for3d,pt3_line,3);
        cv::drawMarker(showrg, Point(int(pic_pt3_for3d.x), int(pic_pt3_for3d.y)), (255, 0, 0), 2, 1, 5);//上面的



       
    
            // //publish用的
            vs_msg.lf_P1_x = pic_pt0_for3d.x;
            vs_msg.lf_P1_y= pic_pt0_for3d.y;
            
            vs_msg.lf_P2_x = pic_pt1_for3d.x;
            vs_msg.lf_P2_y= pic_pt1_for3d.y;
            
            vs_msg.rg_P3_x =pic_pt2_for3d.x;
            vs_msg.rg_P3_y= pic_pt2_for3d.y;

            vs_msg.rg_P4_x = pic_pt3_for3d.x;
            vs_msg.rg_P4_y= pic_pt3_for3d.y;

            vs_msg.lf_line1[0]=pt0_line[0];
            vs_msg.lf_line1[1]=pt0_line[1];
            vs_msg.lf_line1[2]=pt0_line[2];
            
            vs_msg.lf_line2[0]=pt1_line[0];
            vs_msg.lf_line2[1]=pt1_line[1];
            vs_msg.lf_line2[2]=pt1_line[2];
            
            vs_msg.rg_line1[0]=pt3_line[0];
            vs_msg.rg_line1[1]=pt3_line[1];
            vs_msg.rg_line1[2]=pt3_line[2];

            vs_msg.rg_line2[0]=pt2_line[0];
            vs_msg.rg_line2[1]=pt2_line[1];
            vs_msg.rg_line2[2]=pt2_line[2];
        
            vs_msg.target_lf_P1_x=target0.x;
            vs_msg.target_lf_P1_y=target0.y;
            vs_msg.target_lf_P2_x=target1.x;
            vs_msg.target_lf_P2_y=target1.y;
            vs_msg.target_rg_P3_x=target2.x;
            vs_msg.target_rg_P3_y=target2.y;
            vs_msg.target_rg_P4_x=target3.x;
            vs_msg.target_rg_P4_y=target3.y;


            vs_msg.lf_P1_x_for_error=pic_pt0_forerror.x;
            vs_msg.lf_P1_y_for_error=pic_pt0_forerror.y;
            vs_msg.lf_P2_x_for_error=pic_pt1_forerror.x;
            vs_msg.lf_P2_y_for_error=pic_pt1_forerror.y;
            vs_msg.rg_P3_x_for_error=pic_pt2_forerror.x;
            vs_msg.rg_P3_y_for_error=pic_pt2_forerror.y;
            vs_msg.rg_P4_x_for_error=pic_pt3_forerror.x;
            vs_msg.rg_P4_y_for_error=pic_pt3_forerror.y;
            pub.publish(vs_msg);
        
        
        cout<<"结果发送成功"<<endl;
        cout<<"       plf1:"<<pic_pt0_forerror<<endl;
        cout<<"       plf2:"<<pic_pt1_forerror<<endl;
        cout<<"       prg3:"<<pic_pt2_forerror<<endl;
        cout<<"       prg4:"<<pic_pt3_forerror<<endl;
        cout<<"    lf_line1"<<pt0_line[0]<<","<<pt0_line[1]<<","<<pt0_line[2]<<endl;
        cout<<"    lf_line2"<<pt1_line[0]<<","<<pt1_line[1]<<","<<pt1_line[2]<<endl;
        cout<<"    rg_line1"<<pt3_line[0]<<","<<pt3_line[1]<<","<<pt3_line[2]<<endl;
        cout<<"    rg_line2"<<pt2_line[0]<<","<<pt2_line[1]<<","<<pt2_line[2]<<endl;
        cout<<"target_lf_p1"<<target0<<endl;
        cout<<"target_lf_p2"<<target1<<endl;
        cout<<"target_rg_p3"<<target2<<endl;
        cout<<"target_rg_p4"<<target3<<endl;
        cout<<"___________________________________________________________________________"<<endl;
        cv::drawMarker(showrg, Point(int(pic_pt3_forerror.x), int(pic_pt3_forerror.y)), (255, 0, 0), 2, 1, 5);//上面的
        cv::drawMarker(showrg, Point(int(pic_pt2_forerror.x), int(pic_pt2_forerror.y)), (0, 255, 0), 2, 1, 5);//下面的 

        cv::drawMarker(showrg, Point(int(target3.x), int(target3.y)), (255, 0, 0), 2, 1, 5);//上面的
        cv::drawMarker(showrg, Point(int(target2.x), int(target2.y)), (0, 255, 0), 2, 1, 5);//下面的 
        namedWindow("right",0);
        resizeWindow("right", 640, 512);
        cv::imshow("right",showrg);
        cv::drawMarker(showlf, Point(int(pic_pt0_forerror.x), int(pic_pt0_forerror.y)), (255, 0, 0), 2, 1, 5);//上面的
        cv::drawMarker(showlf, Point(int(pic_pt1_forerror.x), int(pic_pt1_forerror.y)), (0, 255, 0), 2, 1, 5);//下面的 
        
        cv::drawMarker(showlf, Point(int(target0.x), int(target0.y)), (255, 0, 0), 2, 1, 5);//上面的
        cv::drawMarker(showlf, Point(int(target1.x), int(target1.y)), (0, 255, 0), 2, 1, 5);//下面的
        namedWindow("left",0);
        resizeWindow("left", 640, 512);
        cv::imshow("left",showlf);
            OpenFile<<pic_pt0_forerror.x<<","<<pic_pt0_forerror.y<<";"<<pic_pt1_forerror.x<<","<<pic_pt1_forerror.y<<";"
            <<pic_pt2_forerror.x<<","<<pic_pt2_forerror.y<<";"<<pic_pt3_forerror.x<<","<<pic_pt3_forerror.y<<"\n";
            //OpenFile.close();


        waitKey(3);
    }
    return 0;
}