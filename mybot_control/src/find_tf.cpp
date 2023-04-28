#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "arucoDetection/ArucoMsg.h"
#include <boost/thread/thread.hpp>

class VelocityControl{

  public:
  
    VelocityControl(){
		  velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    	yaw_sub = n.subscribe("/arucomarker/info", 100, &VelocityControl::updateArucoVals, this);
      odom_sub = n.subscribe("/camera/color/image_raw", 100, &VelocityControl::OdomPublisher, this);
    	ROS_INFO_STREAM("No Sub : \n");
    }
	
  private:

    ros::NodeHandle n;
    ros::Publisher velocity_pub;
	  ros::Subscriber yaw_sub,odom_sub;
    double aruco_yaw;
    double distance;
    double linearVel=0,angularVel=0;
	  geometry_msgs::Twist vel_msg;
	  bool move = 0;

    void updateArucoVals(const arucoDetection::ArucoMsg::ConstPtr& msg){
    	aruco_yaw = msg->yaw;
    	distance = msg->distance;
		  //ROS_INFO_STREAM("Yaw : " << aruco_yaw);
      //ROS_INFO_STREAM("Distance : " << distance);
    	calcVelocity();
    }
    void OdomPublisher(const sensor_msgs::ImageConstPtr & msg){
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      geometry_msgs::TransformStamped transformStamped;
      while(transformStamped.transform.translation.x==0 && ros::ok()){
        try{
                transformStamped = tfBuffer.lookupTransform("base_link", "odom",ros::Time(0));
                ROS_INFO_STREAM("Trans Transform : "<<transformStamped.transform.translation<<"\n");
                //ROS_INFO_STREAM("Rot Transform : "<<transformStamped.transform.rotation<<"\n");
        }

        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
        }
      }
    }

    void calcVelocity(){
			if (distance > 5)
			{
				linearVel = 0.5 - (0.141047 * 2 * std::sqrt(abs(distance)));
			}
			else if (distance <= 5)
			{
				linearVel = 0.2236 * std::sqrt(distance);
			}
			if (abs(aruco_yaw) > 180 / 2)
			{
				angularVel = 0.2;
			}
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
			//ROS_INFO_STREAM("Published Velocity\n");

		}
		else{
				move = 0;
				vel_msg.linear.x = 0;
				vel_msg.angular.z = 0;
				velocity_pub.publish(vel_msg);
				//ROS_INFO_STREAM("Reached Charging Station\n");
		}
    }
	
};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  VelocityControl obj;
  ros::waitForShutdown();
  //ros::spin();
}
