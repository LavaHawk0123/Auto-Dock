#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class OdometryPublisher{
  private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;
    double dt = (current_time - last_time).toSec();
    double delta_x ;
    double delta_y ;
    double delta_th;

    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;

  public:
    ros::Time current_time, last_time;
    OdometryPublisher(){
      this->odom_pub = this->n.advertise<nav_msgs::Odometry>("odom", 50);
      this->current_time = ros::Time::now();
      this->last_time = ros::Time::now();
      this->dt = (this->current_time - this->last_time).toSec();
      this->delta_x = (this->vx * cos(this->th) - this->vy * sin(this->th)) * this->dt;
      this->delta_y = (this->vx * sin(this->th) + this->vy * cos(this->th)) * this->dt;
      this->delta_th = this->vth * this->dt;
    }

    void updateOdom(){
      this->current_time = ros::Time::now();
      this->dt = (this->current_time - this->last_time).toSec();
      this->delta_x = (this->vx * cos(this->th) - this->vy * sin(this->th)) * this->dt;
      this->delta_y = (this->vx * sin(this->th) + this->vy * cos(this->th)) * this->dt;
      this->delta_th = this->vth * this->dt;
      this->x += this->delta_x;
      this->y += this->delta_y;
      this->th += this->delta_th;
    }

    void sendTransform(){
      this->odom_trans.header.stamp = this->current_time;
      this->odom_trans.header.frame_id = "odom";
      this->odom_trans.child_frame_id = "base_link";

      this->odom_trans.transform.translation.x = this->x;
      this->odom_trans.transform.translation.y = this->y;
      this->odom_trans.transform.translation.z = 0.0;
      this->odom_trans.transform.rotation = this->odom_quat;

      //send the transform
      this->odom_broadcaster.sendTransform(this->odom_trans);
    }

    void sendOdom(){
      this->odom.header.stamp = this->current_time;
      this->odom.header.frame_id = "odom";
      this->odom.child_frame_id = "base_link";

      //set the position
      this->odom.pose.pose.position.x = x;
      this->odom.pose.pose.position.y = y;
      this->odom.pose.pose.position.z = 0.0;
      this->odom.pose.pose.orientation = this->odom_quat;

      //set the velocity
      this->odom.twist.twist.linear.x = vx;
      this->odom.twist.twist.linear.y = vy;
      this->odom.twist.twist.angular.z = vth;

      //publish the message
      this->odom_pub.publish(this->odom);
    }

	
};
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  OdometryPublisher obj;
  while(ros::ok()){
    obj.updateOdom();
    obj.sendOdom();
    obj.sendTransform();
    obj.last_time = obj.current_time;
  }
}
