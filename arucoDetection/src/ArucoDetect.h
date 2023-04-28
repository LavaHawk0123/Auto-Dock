//Owner : 
//Aditya Arun Iyer   Contact - 8073711953  Mail - adityaaruniyer01@gmail.com

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

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_datatypes.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_ros/buffer.h>

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

#include <string.h>

#include <thread>

#include "arucoDetection/ArucoMsg.h"

namespace ad {
	

    class ArucoDetect {
    
        private:
			
			// ROS Node Handler
			ros::NodeHandle nh;
				
				// Subscriber object to camera feed
			ros::Subscriber sub_camera;

			ros::Subscriber odom_sub,tf_sub;
			
			// Subscriber object to camera feed(simulated)
			ros::Subscriber sub_camera_sim,tf_sub_sim;
				
			// Image publisher object with markers drawn    
			ros::Publisher pub_marker;
			
			// Marker Pose Publisher object
			ros::Publisher pub_pose;
			
			// Velocity Publisher object
			ros::Publisher pub_velocity;
			
			// Image publisher object with transform-axis drawn  
			ros::Publisher pub_pose_estimated;
			
			//Decoded Marker ID Publisher object 
			ros::Publisher pub_marker_id;
			
			// Publisher for Vizualization Marker in Rviz
			ros::Publisher viz_marker_pub;

			// Publisher to publish information about the marker  
			ros::Publisher pub_arucoMarkerInfo;
			
			// Drawn Image to be publish
			sensor_msgs::ImagePtr msg_pub;
			
			// Image with axis drawn to be published
			sensor_msgs::ImagePtr msg_pub_estimated;
			
			// ROS String message storing ID of marker detected
			std_msgs::String marker_ID;
			
			// ROS PoseStamped message to store the pose of detected Marker
			geometry_msgs::PoseStamped MarkerPose;
			
			/* CV Bridge identifier to convert ROS Image message 
			to an RGB Mat allowing the use of OpenCV functions
			*/
			cv_bridge::CvImagePtr camera_image;
			
			//Velocity Publisher message
			geometry_msgs::Twist Vel;
			
			// Standard String variable storing ID of detected marker
			std::string ID;
			
			
			// Co-ordinates of markers detected in camera frame
			std::vector < cv::Point2f > centers;
			
			// Vector of ID's of markers detected
			std::vector < int > marker_ids_detected;
			
			/*Corner vector of already detected markers corners. 
			type : std::vector<std::vector<cv::Point2f> >
			For each marker, its four corners are provided. For N detected markers,
			the dimensions of this array should be Nx4.The order of the corners should be clockwise.*/

			std::vector < std::vector < cv::Point2f > > corners;
			
			/* An array of output rotation vectors. Each element in rvecs corresponds to the specific marker in marker_ids_detected
			type : std::vector<cv::Vec3d> */
			std::vector < cv::Vec3d > RotationalVectors;
			
			/* An array of output translation vectors. Each element in tvecs corresponds to the specific marker in marker_ids_detected
			type : std::vector<cv::Vec3d> */
			std::vector < cv::Vec3d > TranslationalVectors;
			
			//Image with marker corners highlighted and border drawn
			cv::Mat markers_drawn_img;
			
			// Determines the type of aruco markers used
			cv::Ptr < cv::aruco::Dictionary > dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
			
			/* 3 x 3 matrix using focal length of camera for calliberation 
			[[fx  0  cx ]
			[0   fy  cy]
			[0   0   1 ]]
			*/ 
			cv::Mat cameraMatrix;
			
			/* a vector or 4, 5, or 8 parameters: [k1, k2, p1, p2 [, k3 [, k4, k5, k6]]].
			It helps cancel out distortion due to jitters and bad frames*/
			cv::Mat distortionCoeffs;
			
			/* object of DetectorParameters class passed to cv::aruco::detectMarkers().
			It is used to tune detetction and pose estimation*/
			cv::Ptr < cv::aruco::DetectorParameters > params = cv::aruco::DetectorParameters::create();
			
			//
			cv::Mat final_image;
			
			// Origin, assumed (0,0,0)
			cv::Mat origin = cv::Mat::zeros(3, 1, CV_64F);
					
					// Rotational Matrix (3x3) formed from rotational vectors
			cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64F);
			
			//Counter variable for no. of times a marker is detected
			int times_detected;
			
			//Boolean flag holds true when a marker is detected and false when not
			bool found;

			double markerDistance;

			double aruco_yaw;
			
			// Defenition of shape for visualiztion marker in Rviz
			uint32_t shape = visualization_msgs::Marker::CUBE;

			geometry_msgs::Quaternion quaternion;

			arucoDetection::ArucoMsg arucoInfoMsg;

			cv::Vec3d curr_RotVec;

        	cv::Vec3d curr_TransVec;

			geometry_msgs::TransformStamped odom_to_base_tf,base_to_cam_tf;

			/*
			
			Parameters for camera calliberation. Uncomment if needed only :
			
			int numCornersHor = 7;
			
				int numCornersVer = 6;
			
				int numSquares = numCornersHor * numCornersVer;
			
				Size boardSize = Size(numCornersHor, numCornersVer);
			
			std::vector < std::vector < cv::Point3f >> object_points;
			
			std::vector < std::vector < cv::Point2f >> image_points;
			
			*/

        public:
			//Class Constructor    
			ArucoDetect();
			
			//Image Callback Function    
			void imageCallback(const sensor_msgs::ImageConstPtr & msg);

			void getTransforms(const sensor_msgs::ImageConstPtr & msg);

			void poseCallback(const nav_msgs::Odometry msg);
			
			// Function to detect and draw Aruco Markers
			void detect_aruco();
			
			// Function to set DistCoeffs and CameraMatrix vectors(Caliberate Camera)
			void estimate_pose();
			
			//Estimate pose of Aruco Marker
			void calculate_pose();
			
			// Function to optimise parameters for Aruco Detection and Pose Estimation
			void set_params();

			void publishMarker(cv::Vec3d curr_RotVec,cv::Vec3d curr_TransVec,geometry_msgs::Quaternion quaternion);

			void publishArucoInfo();

			void getGlobalPose();

    };
}
