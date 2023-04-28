//Aditya Arun Iyer   Contact - 8073711953  Mail - adityaaruniyer01@gmail.com

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include "std_msgs/String.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <geometry_msgs/PoseStamped.h>

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


class kalmanFilter{

 private:
	ros::Subscriber sub_pose,sub_camera;
	ros::Publisher pub_kf_pose;
	cv::KalmanFilter KF;         // instantiate Kalman Filter
	int nStates;           // the number of states
	int nMeasurements;       // the number of measured states
	int nInputs;             // the number of action control
	double dt;
	cv::Mat measured_eulers = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat measurements = cv::Mat::zeros(1, 6, CV_64F);
	cv::Mat estimated_pose = cv::Mat::zeros(1, 6, CV_64F);
	double roll, pitch, yaw;
    
 public:
   
   kalmanFilter(ros::NodeHandle *nh){
   	
   	create_Kalman_Filter();
   	
   	sub_camera = nh->subscribe("/camera/color/image_raw", 100, & kalmanFilter::imageCallback,this);
   	sub_pose = nh->subscribe("marker/pose", 1000, &kalmanFilter::pose_callback, this);
   	
   	pub_kf_pose = nh->advertise< geometry_msgs::PoseStamped > ("/marker/pose_corrected", 1);
   }
   
   void create_Kalman_Filter(){
   	ROS_INFO_STREAM("Initialized Kalman Filter");
	nStates = 18;            // the number of states
	nMeasurements = 6;       // the number of measured states
	nInputs = 0;             // the number of action control
	dt = 0.0125; 
	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
   
   }
   
   void imageCallback(const sensor_msgs::ImageConstPtr & img) {
   	
   }
   
   void pose_callback(const geometry_msgs::PoseStamped & msg){
   	    measured_eulers = cv::Mat::zeros(3, 1, CV_64F);
   	    measurements = cv::Mat::zeros(1, 6, CV_64F);
   	    estimated_pose = cv::Mat::zeros(1, 6, CV_64F);
   	    ROS_INFO_STREAM("entered pose callback");
	    measurements.at<double>(0) = msg.pose.position.x; // x
	    measurements.at<double>(1) = msg.pose.position.y; // y
	    measurements.at<double>(2) = msg.pose.position.z; // z
	    tf2::Quaternion q(
		msg.pose.orientation.x,
		msg.pose.orientation.y,
		msg.pose.orientation.z,
		msg.pose.orientation.w);
	    tf2::Matrix3x3 m(q);
	    m.getRPY(roll, pitch, yaw);
	    measurements.at<double>(3) = roll* (180.0/3.141592653589793238463);      // roll
	    measurements.at<double>(4) = pitch* (180.0/3.141592653589793238463);     // pitch
	    measurements.at<double>(5) = yaw* (180.0/3.141592653589793238463);       // yaw
	    //ROS_INFO_STREAM(measurements);
	    updateKalmanFilter();
   }
   
   void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt){
   
	  KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
	  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
	  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
	  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
	  
		         /* DYNAMIC MODEL */
	  /*[1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
	    [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
	    [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
	    [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
	    [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
	    [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
	    [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
	    [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
	    [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
	    [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
	    [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
	    [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
	    [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
	    [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
	    [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
	    [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
	    [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
	    [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]*/
	 	  
	  // position
	  KF.transitionMatrix.at<double>(0,3) = dt;
	  KF.transitionMatrix.at<double>(1,4) = dt;
	  KF.transitionMatrix.at<double>(2,5) = dt;
	  KF.transitionMatrix.at<double>(3,6) = dt;
	  KF.transitionMatrix.at<double>(4,7) = dt;
	  KF.transitionMatrix.at<double>(5,8) = dt;
	  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
	  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
	  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
	  
	  // orientation
	  KF.transitionMatrix.at<double>(9,12) = dt;
	  KF.transitionMatrix.at<double>(10,13) = dt;
	  KF.transitionMatrix.at<double>(11,14) = dt;
	  KF.transitionMatrix.at<double>(12,15) = dt;
	  KF.transitionMatrix.at<double>(13,16) = dt;
	  KF.transitionMatrix.at<double>(14,17) = dt;
	  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
	  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
	  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
	  
	       /* MEASUREMENT MODEL 
	    [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	    [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	    [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	    [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
	    [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
	    [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]*/
	    
	  KF.measurementMatrix.at<double>(0,0) = 1;  // x
	  KF.measurementMatrix.at<double>(1,1) = 1;  // y
	  KF.measurementMatrix.at<double>(2,2) = 1;  // z
	  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
	  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
	  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
  }


void updateKalmanFilter() {
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();
    ROS_INFO_STREAM(prediction);
    
    // The "correct" phase that is going to use the predicted value and our measurement
    
    cv::Mat estimated = KF.correct(measurements);
    ROS_INFO_STREAM(estimated);
/*
    // Estimated translation
    estimated_pose.at<double>(0) = estimated.at<double>(0);
    estimated_pose.at<double>(1) = estimated.at<double>(1);
    estimated_pose.at<double>(2) = estimated.at<double>(2);
    
    // Estimated euler angles
    estimated_pose.at<double>(0) = estimated.at<double>(9);
    estimated_pose.at<double>(1) = estimated.at<double>(10);
    estimated_pose.at<double>(2) = estimated.at<double>(11);
    ROS_INFO_STREAM(estimated_pose);*/
  
  }

};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "kalmanFilter");
  ros::NodeHandle nh;
  kalmanFilter obj =  kalmanFilter(&nh);
  ros::spin();
}
