// opencv lib
#include <opencv2/aruco.hpp>

#include <opencv2/imgproc.hpp>

#include <opencv2/imgcodecs.hpp>

// ros lib
#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include "std_msgs/String.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>

#include <fstream>

#include <sstream>

#include <iostream>

#include <armadillo>

#include <cmath>

#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include "geometry_msgs/Twist.h"

#include <visualization_msgs/Marker.h>

#include <sensor_msgs/LaserScan.h>

#include <stdlib.h>

#include<time.h>


class LaserManipulation{
  public:
    //cv::Mat image(500, 1000, CV_8UC1, Scalar(0));
    LaserManipulation(){
      image_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",10,&LaserManipulation::getLaserData,this);
    }
	
  private:

    ros::NodeHandle n;
    ros::Publisher image_pub;
    ros::Subscriber laser_sub;
    double angle_per_ray = 0;
    cv::Mat img = cv::Mat(100,100,CV_64F);
    cv::Mat canny_img;// = cv::Mat(100,100,CV_8UC3);
    std::vector<cv::Vec4i> lines;
    cv::Mat edges;
    double start_angle = 120.0;
    double end_angle = 240.0;
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,cv::Size(2 * 2 + 1,2 * 2 + 1),cv::Point(2, 2));

    void getLaserData(const sensor_msgs::LaserScan::ConstPtr& msg){
    	angle_per_ray = 360.0/720.0;
        for(int i=start_angle/angle_per_ray;i<end_angle/angle_per_ray;i++){
            double sample = msg->ranges[i];
            try{
              if(sample>3){
                continue;
              }
              else{
                ROS_INFO_STREAM("sample:"<<msg->ranges.size()<<"\n");
                ROS_INFO_STREAM("angle per ray: "<<angle_per_ray<<"\n");
                ROS_INFO_STREAM("Iteration No : "<<i<<"\n");
                ROS_INFO_STREAM("angle: "<<i*angle_per_ray<<"\n");
                double angle = i*angle_per_ray*3.14/180;
                int x = sample*cos(angle);
                x = 50-x;
                ROS_INFO_STREAM("x:"<<x<<"\n");
                int y = sample*sin(angle);
                y = 50-y;
                ROS_INFO_STREAM("y:"<<y<<"\n");
                img.at<double>(x,y) = 255;
                ROS_INFO_STREAM("Point added: "<<x<<","<<y<<"\n");
              }
            }
            catch(...){
               ROS_INFO_STREAM("ERROR\n");
            }
        }
      cv::resize(img, img, cv::Size(1920, 1080), cv::INTER_AREA);
      cv::imwrite("/home/neo/ROS_Workspaces/zmr_ws/src/Auto-Dock-Sim/Media/LaserImage.jpg",img);
      detect_edges();
      //cv::imwrite("/home/neo/ROS_Workspaces/zmr_ws/src/Auto-Dock-Sim/Media/LaserImage.jpg",img);
      sleep(1);
    }

    void detect_edges(){
      canny_img = cv::imread("/home/neo/ROS_Workspaces/zmr_ws/src/Auto-Dock-Sim/Media/LaserImage.jpg");
      preProcessing();
      cv::Canny(canny_img, edges, 50, 150, 3);
      cv::HoughLinesP(canny_img, lines, 1, CV_PI/180, 200);
      ROS_INFO_STREAM("lines detected : "<<lines.size());
      for (size_t i=0; i<lines.size(); i++) {
        cv::Vec4i l = lines[i];
        std::cout<<"Line "<<i<<" : "<<l<<"\n";
        line(canny_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 3, cv::LINE_AA);
      }
      cv::imwrite("/home/neo/ROS_Workspaces/zmr_ws/src/Auto-Dock-Sim/Media/Canny.jpg",canny_img);
    }

    void preProcessing(){
      cv::morphologyEx(canny_img,canny_img,cv::MORPH_CLOSE,element,cv::Point(-1,-1),2);
      cv::dilate(canny_img,canny_img,cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)),cv::Point(-1,-1),5);
      cv::cvtColor(canny_img,canny_img,cv::COLOR_BGR2GRAY);
    }
    
};

int main(int argc, char** argv){
  ros::init(argc, argv, "edge_detector");
  LaserManipulation obj;
  ros::spin();
}
