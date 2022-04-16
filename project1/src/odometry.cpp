#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
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
        this->pose[i] = 0;
    } else {
      this->pose[0] = this->pose[0] + (msg->linear).x * (msg->header.stamp.sec - this->times[0] + (msg->header.stamp.nsec - this->times[1])/10^9);
      this->pose[1] = this->pose[1] + (msg->linear).y * (msg->header.stamp.sec - this->times[0] + (msg->header.stamp.nsec - this->times[1])/10^9);
      this->pose[2] = this->pose[2] + (msg->angular).z * (msg->header.stamp.sec - this->times[0] + (msg->header.stamp.nsec - this->times[1])/10^9);

      ROS_INFO("position in x: %f", this->pose[0]);
      ROS_INFO("position in y: %f", this->pose[1]);
      ROS_INFO("orientation: %f", this->pose[2]);

      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;
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
