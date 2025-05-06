#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/wheels_rpm.h"

class ControlCalculator {
public:
  ControlCalculator() { 
    this->sub = n.subscribe("cmd_vel", 1000, &ControlCalculator::computeControlCallback, this);
    this->controlPublisher = n.advertise<project1::wheels_rpm>("wheels_rpm", 1000);
  }

  void main_loop() {
    ros::spin();
  }

  void computeControlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
      project1::wheels_rpm controlMessage;
      
      // Calculate the rpm of each wheel
      controlMessage.rpm_fl = 60 * this->T * ((msg->twist).linear.x - (msg->twist).linear.y - (this->sum_lX_lY) * (msg->twist).angular.z) / this->r;
      controlMessage.rpm_fr = 60 * this->T * ((msg->twist).linear.x + (msg->twist).linear.y + (this->sum_lX_lY) * (msg->twist).angular.z) / this->r; 
      controlMessage.rpm_rr = 60 * this->T * ((msg->twist).linear.x - (msg->twist).linear.y + (this->sum_lX_lY) * (msg->twist).angular.z) / this->r; 
      controlMessage.rpm_rl = 60 * this->T * ((msg->twist).linear.x + (msg->twist).linear.y - (this->sum_lX_lY) * (msg->twist).angular.z) / this->r; 

      controlMessage.header.stamp.sec = (msg->header).stamp.sec;
      controlMessage.header.stamp.nsec = (msg->header).stamp.nsec; 

      controlPublisher.publish(controlMessage);
  }

private:
  ros::NodeHandle n; 
  ros::Subscriber sub;
  ros::Publisher controlPublisher;
  int T = 5; 
  float r = 0.075;
  float sum_lX_lY = 0.36;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "controlCalculator");
  
  ControlCalculator my_controlCalculator;

  my_controlCalculator.main_loop();

  return 0;
}
