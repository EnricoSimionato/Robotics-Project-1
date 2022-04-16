#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#define M_PI 3.14159265358979323846

class OdometryCalculator {
public:
  OdometryCalculator() { 
    this->velocitiesSubcriber = n.subscribe("cmd_vel", 1000, &OdometryCalculator::computeOdometryCallback, this); // mettere lo / in wheel_states?
    this->odometryPublisher = n.advertise<nav_msgs::Odometry>("odom", 1000);
  }

  void main_loop() {
    ros::spin();
  }

  void computeOdometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if(!this->firstUse) {
      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;
      this->firstUse = true;
      for(int i = 0; i < 3; i++)
        this->pose[i] = 0.0;
    } else {
      this->pose[0] = this->pose[0] + (msg->twist).linear.x * (msg->header.stamp.sec - this->times[0] + ((float) ((msg->header).stamp.nsec - this->times[1]) / 1000000000.0));
      this->pose[1] = this->pose[1] + (msg->twist).linear.y * (msg->header.stamp.sec - this->times[0] + ((float) ((msg->header).stamp.nsec - this->times[1]) / 1000000000.0));
      this->pose[2] = this->pose[2] + (msg->twist).angular.z * (msg->header.stamp.sec - this->times[0] + ((float) ((msg->header).stamp.nsec - this->times[1]) / 1000000000.0));

      ROS_INFO("position in x: %f", this->pose[0]);
      ROS_INFO("position in y: %f", this->pose[1]);
      ROS_INFO("orientation: %f", this->pose[2]);

      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;
      
      nav_msgs::Odometry odometryMessage;

      // x,y position
      odometryMessage.pose.pose.position.x = this->pose[0];
      odometryMessage.pose.pose.position.y = this->pose[1];
      odometryMessage.pose.pose.position.z = 0.0;
      
      // orientation
      tf2::Quaternion q;
      q.setRPY(0, 0, this->pose[2]); //this should be in radians
      odometryMessage.pose.pose.orientation.x = q.x();
      odometryMessage.pose.pose.orientation.y = q.y();
      odometryMessage.pose.pose.orientation.z = q.z();
      odometryMessage.pose.pose.orientation.w = q.w();

      // v,w velocities
      odometryMessage.twist.twist.linear.x = (msg->twist).linear.x;
      odometryMessage.twist.twist.linear.y = (msg->twist).linear.y;
      odometryMessage.twist.twist.linear.z = (msg->twist).linear.z;
      odometryMessage.twist.twist.angular.x = (msg->twist).angular.x;
      odometryMessage.twist.twist.angular.y = (msg->twist).angular.y;
      odometryMessage.twist.twist.angular.z = (msg->twist).angular.z;

      // time
      odometryMessage.header.stamp.sec = this->times[0];
      odometryMessage.header.stamp.nsec = this->times[1];

      odometryPublisher.publish(odometryMessage);
    }
  }

private:
  ros::NodeHandle n; 
  ros::Subscriber velocitiesSubcriber;
  ros::Publisher odometryPublisher;
  bool firstUse = false;
  int times[2];
  float pose[3];
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometryCalculator");
  
  OdometryCalculator my_odometryCalculator;

  my_odometryCalculator.main_loop();

  return 0;
}
