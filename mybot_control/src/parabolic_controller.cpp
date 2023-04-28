#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include<math.h>

class Coordinate{
   public:
	double x;
	double y;
};

class Parabola{
   public:
	double a,h,k;
	double derivativeXbyY,derivativeYbyX;
	double angle_x;
	Coordinate focus;
	Parabola(double a,double h,double k){
		this->a = a;
		this->h = h;
		this->k = k;
	}
	
	Parabola(){
		
	}
	double getVal(double x){
		return (this->a)*(pow((x-this->k),2))+this->h;
	}
	void setParams(Coordinate c1, Coordinate c2){
		this->h = c1.x;
		this->k = c1.y;
		this->a = (c2.x - c1.x)/pow((c2.y-c1.y),2);
		focus.x = this->h+(1/(this->a*4));
		focus.y = this->k;
	}
	
	void printFunction(){
		ROS_INFO_STREAM("Function : x="<<this->a<<"(y-("<<this->k<<"))^2+("<<this->h<<")\n");
	}
	
	void printParams(){
		ROS_INFO_STREAM("h : "<<this->h<<"\n");
		ROS_INFO_STREAM("k : "<<this->k<<"\n");
		ROS_INFO_STREAM("a : "<<this->a<<"\n");
		ROS_INFO_STREAM("focus : ("<<focus.x<<","<<focus.y<<")\n");
	}
	
	double getRadius(Coordinate pose,Coordinate c){
		return sqrt(pow(c.x-pose.x,2)+pow(c.y-pose.y,2))/3;
	}

	double setDerivative(Coordinate p){
		this->derivativeXbyY = 2*this->a*(p.y-this->k);
		this->angle_x = atan2(this->derivativeXbyY,1);
		ROS_INFO_STREAM("Derivative: "<<this->derivativeXbyY<<" Angle: "<<this->angle_x<<"\n");
		return this->derivativeXbyY;
	}
};

class ParabolicController{
  public:
    ParabolicController(double x1,double y1,double x2,double y2){
      pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
      sub = n.subscribe("odom", 100, &ParabolicController::poseCallback,this);
      c1.x = x1;
      c1.y = y1;
      c2.x = x2;
      c2.y = y2;
      ROS_INFO_STREAM("Set x1: "<<c1.x<<", y1: "<<c1.y<<"x2: "<<c2.x<<", y2: "<<c2.y<<"\n");
      path.setParams(c1,c2);
      path.printParams();
      path.printFunction();
    }
  private:
  	ros::NodeHandle n;
  	ros::Publisher pub;
  	ros::Subscriber sub;
  	// Vertex of parabola
  	Coordinate c1;
  	// Point it passes through
  	Coordinate c2;
  	Coordinate pose;
  	Parabola path;
  	geometry_msgs::Twist msg,stop;
  	double net_vel;
  	double vel_x = 0.4;
	double vel_y = 0; 
	double theta_diff=0;
	double distance;
	void poseCallback(const nav_msgs::Odometry& msg){
	    ROS_INFO("x: %.2f, y: %.2f", msg.pose.pose.position.x, msg.pose.pose.position.y);
	    pose.x = msg.pose.pose.position.x;
	    pose.y = msg.pose.pose.position.y;
		path.setDerivative(pose);
		theta_diff = atan2(pose.y,pose.x) - path.angle_x;
	    calcVelocity(pose.x,this->path);
	    publishVelocity();
	}
	
	void setVelocities(double v_x,double v_y){
		this->vel_x = v_x;
		this->vel_y = v_y;
	}
	void calcVelocity(double curr_x,Parabola path){
		this->vel_y = theta_diff;
		this->net_vel = sqrt(path.a/(pose.x-path.h));//abs(2*path.a*(pose.y-path.k));//sqrt(pow(this->vel_y,2)+pow(this->vel_x,2));
		
	}
	
	void publishVelocity(){
	    msg.linear.x = this->net_vel;//abs(this->distance/5-theta_diff);
	    msg.angular.z = -this->vel_y*this->distance;
		this->distance = sqrt(pow((pose.x-c1.x),2)+pow((pose.y-c1.y),2));
		ROS_INFO_STREAM("Velocity: Linear x: "<<msg.linear.x<<", Angular z: "<<msg.angular.z<<"\n");
		ROS_INFO("x: %.2f, y: %.2f", pose.x, pose.y);
		ROS_INFO_STREAM("Distance : "<<this->distance<<"\n");
		if(this->distance>0.1){
			ROS_INFO_STREAM("Moving to Goal");
	    	pub.publish(msg);
		}
		else{
			ROS_INFO_STREAM("Goal Reached. Stopping");
			pub.publish(stop);
		}
	}
};
	

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parabolic_controller");
    ParabolicController obj(5,5,0.007501607074063346,0.001932313398747389);
    ros::spin(); 
}
