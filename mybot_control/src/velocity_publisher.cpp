#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "arucoDetection/ArucoMsg.h"
#include <boost/thread/thread.hpp>

class VelocityControl{

  public:
  
    VelocityControl(){
		velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    	yaw_sub = n.subscribe("/arucomarker/info", 100, &  VelocityControl::updateVals, this);
    	ROS_INFO_STREAM("No Sub : \n");
    }
	
  private:

    ros::NodeHandle n;
    ros::Publisher velocity_pub;
	ros::Subscriber yaw_sub;
    double aruco_yaw;
    double distance;
    double linearVel=0,angularVel=0;
	geometry_msgs::Twist vel_msg;
	bool move = 0;

    void updateVals(const arucoDetection::ArucoMsg::ConstPtr& msg){
    	aruco_yaw = msg->yaw;
    	distance = msg->distance;
		ROS_INFO_STREAM("Yaw : " << aruco_yaw);
        ROS_INFO_STREAM("Distance : " << distance);
    	calcVelocity();
    }
    
    void calcVelocity(){
		// Linear Velocity Calculation

			// Parabolic function: aruco_yaw vs linear velocity

			if (distance > 5)
			{
				linearVel = 0.5 - (0.141047 * 2 * std::sqrt(abs(distance)));
			}

			// Parabolic function: linear distance vs linear velocity

			else if (distance <= 5)
			{
				linearVel = 0.2236 * std::sqrt(distance);
			}

			// Angular Velocity Calculaton

			// Constant angular velocity till rover reaches Pi

			if (abs(aruco_yaw) > 180 / 2)
			{
				angularVel = 0.2;
			}

			// Parabolic function: bearing vs angular velocity

			else if (abs(aruco_yaw) <= 90 / 2)
			{
				angularVel = 0.25851 * std::sqrt(abs(aruco_yaw*3.14/180));
			}

			if(aruco_yaw>0){
				angularVel = -angularVel;
			}
		velocityPublisher();
    }
    
    void velocityPublisher(){
		if(distance>0.25){
			move = 1;
			vel_msg.linear.x = linearVel;
			vel_msg.angular.z = angularVel/4;
			ROS_INFO_STREAM("Calculated Velocity : \n");
			ROS_INFO_STREAM("Linear : " << linearVel);
			ROS_INFO_STREAM("Angular : " << angularVel);
			velocity_pub.publish(vel_msg);
			ROS_INFO_STREAM("Published Velocity\n");

		}
		else{
				move = 0;
				vel_msg.linear.x = 0;
				vel_msg.angular.z = 0;
				velocity_pub.publish(vel_msg);
				ROS_INFO_STREAM("Reached Charging Station\n");
		}
    }
	
};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  VelocityControl obj;
  ros::spin();
}
