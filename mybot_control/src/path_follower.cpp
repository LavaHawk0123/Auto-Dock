#include "ros/ros.h"
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "math.h"

class PID{
	public:
		double kp;
		double kd;
		double ki;
	void updateGains(double p,double d,double i){
		this->kp = p;
		this->kd = d;
		this->ki = i;
	}

	double getResult(double delE){
		setError(delE);
		updateErrorParams(delE);
		return this->kp*this->error + this->ki*this->error_sum + this->kd*this->error_diff;
	}
	private:
		double error;
		double error_sum;
		double error_diff;
		double prev_val;

	void setError(double err){
		this->prev_val = this->error;
		this->error = err;
		ROS_INFO_STREAM("Error: integral : "<<this->error_sum<<" derivative : "<<this->error_diff<<"\n");
	}

	void updateErrorParams(double E){
		this->error_sum += E;
		this->error_diff = E-this->prev_val;
	}
};

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
	int quadrant;
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
		focus.y = this->h+this->a;
		focus.x = this->k;
		setQuadrant(c1,c2);
	}
	
	void printFunction(){
		ROS_INFO_STREAM("Function : x="<<this->a<<"(y-("<<this->k<<"))^2+("<<this->h<<")\n");
	}

	void setQuadrant(Coordinate c1, Coordinate c2){
		if(c1.x>c2.x && c1.y>c2.y){
			this->quadrant = 1;
		}
		else if(c1.x<c2.x && c1.y>c2.y){
			this->quadrant = 2;
		}
		else if(c1.x>c2.x && c1.y<c2.y){
			this->quadrant = 4;
		}
		else{
			this->quadrant = 3;
		}
	}
	void printParams(){
		ROS_INFO_STREAM("h : "<<this->h<<"\n");
		ROS_INFO_STREAM("k : "<<this->k<<"\n");
		ROS_INFO_STREAM("a : "<<this->a<<"\n");
		ROS_INFO_STREAM("focus : ("<<focus.x<<","<<focus.y<<")\n");
	}
	
	double getRadius(Coordinate pose){
		return sqrt(pow(focus.x-pose.x,2)+pow(focus.y-pose.y,2));
	}

	double setDerivative(Coordinate p){
		this->derivativeXbyY = 2*this->a*(p.y-this->k);
		this->angle_x = (atan2(1,this->derivativeXbyY)*180/M_PI);
		switch (this->quadrant)
		{
		case 4:
				this->angle_x = this->angle_x-180;
				break;
		case 3:
				this->angle_x = this->angle_x-180;
				break;
		default:
				break;
		}
		ROS_INFO_STREAM("Derivative: "<<this->derivativeXbyY<<" Angle: "<<this->angle_x<<"\n");
		return this->derivativeXbyY;
	}
	
};



class ParabolicController{
  public:
    ParabolicController(double x1,double y1,double x2,double y2){
      pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	  linear.updateGains(0.1,0.00001,0);
	  angular.updateGains(0.1,0.00001,0.0);
      sub_pose = n.subscribe("/odom", 100, &ParabolicController::poseCallback,this);
	  sub_heading = n.subscribe("/amcl_pose", 100, &ParabolicController::imuCallBack,this);
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
  	ros::Subscriber sub_pose,sub_heading;
  	// Vertex of parabola
  	Coordinate c1;
  	// Point it passes through
  	Coordinate c2;
  	Coordinate pose;
  	Parabola path;
  	geometry_msgs::Twist msg,stop;
	PID linear,angular;
  	double net_vel;
  	double vel_x = 0.4;
	double vel_y = 0; 
	double theta_diff=0;
	double distance;
	double heading = 0;
	void poseCallback(const nav_msgs::Odometry msg){
	    pose.x = msg.pose.pose.position.x;
	    pose.y =  msg.pose.pose.position.y;
		path.setDerivative(pose);
		theta_diff = path.angle_x-heading;
		ROS_INFO_STREAM("Theta Diff : "<<theta_diff<<"\n");
	    calcVelocity(pose.x,this->path);
	    publishVelocity();
	}

	void imuCallBack(const geometry_msgs::PoseWithCovarianceStamped msg){

		tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		this->heading = yaw*180/M_PI;
		ROS_INFO_STREAM("Theta : "<<heading<<"\n");
	}
	
	
	void setVelocities(double v_x,double v_y){
		this->vel_x = v_x;
		this->vel_y = v_y;
	}
	void calcVelocity(double curr_x,Parabola path){
		this->vel_y = theta_diff;
		ROS_INFO_STREAM("Vx: "<<this->vel_x<<", Vy: "<<vel_y<<"\n");
		this->net_vel = sqrt(path.a/(pose.x-path.h));
		
	}
	
	void publishVelocity(){
		this->distance = sqrt(pow((pose.x-c1.x),2)+pow((pose.y-c1.y),2));
	    msg.linear.x = std::min(linear.getResult(this->distance),0.3);
	    msg.angular.z = std::min(angular.getResult(theta_diff),0.3);
	    ROS_INFO_STREAM("Velocity: Linear x: "<<msg.linear.x<<", Angular z: "<<msg.angular.z<<"\n");
		ROS_INFO("x: %.2f, y: %.2f", pose.x, pose.y);
		ROS_INFO_STREAM("Distance : "<<this->distance<<"\n");
		if(this->distance>0.7){
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
    ParabolicController obj(6.124868392944336,-5.532645225524902,0,0);
    ros::spin(); 
}
