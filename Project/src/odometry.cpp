#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>
#include "project1/Reset.h"
#define M_PI 3.14159265358979323846

class OdometryCalculator {
public:
  OdometryCalculator() { 
    this->service = n.advertiseService<project1::Reset::Request, project1::Reset::Response>("reset", boost::bind(&OdometryCalculator::reset_callback, this, pose, _1, _2));
    this->velocitiesSubcriber = n.subscribe("cmd_vel", 1000, &OdometryCalculator::computeOdometryCallback, this);
    this->odometryPublisher = n.advertise<nav_msgs::Odometry>("odom", 1000);   
  }

  void main_loop() {
    ros::spin();
  }

  bool reset_callback(float *pose, project1::Reset::Request  &req, project1::Reset::Response &res) {
    res.old_x = this->pose[0];
    res.old_y = this->pose[1];
    res.old_theta = this->pose[2];
    this->pose[0] = req.new_x;
    this->pose[1] = req.new_y;
    this->pose[2] = req.new_theta;
    ROS_INFO("Request to reset x to %f - Responding with old x: %f", 
        (float)req.new_x, (float)res.old_x);
    ROS_INFO("Request to reset y to %f - Responding with old y: %f", 
        (float)req.new_y, (float)res.old_y);
    ROS_INFO("Request to reset theta to %f - Responding with old theta: %f", 
        (float)req.new_theta, (float)res.old_theta);
    return true;
  }

  void computeOdometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (this->firstUse)
    {
      this->firstUse = false;
      this->n.getParam("/initial_x", this->pose[0]);
      this->n.getParam("/initial_y", this->pose[1]);
      this->n.getParam("/initial_t", this->pose[2]);
      this->n.getParam("/initial_secs", this->times[0]);
      this->n.getParam("/initial_nsecs", this->times[1]);
      this->initialPose[0] = this->pose[0];
      this->initialPose[1] = this->pose[1];
      this->initialPose[2] = this->pose[2];
    }
    else
    {
      float ts = (msg->header).stamp.sec - this->times[0] + ((float) (msg->header).stamp.nsec) / 1000000000.0 - ((float) this->times[1]) / 1000000000.0;

      this->pose[2] = this->pose[2] + (msg->twist).angular.z * ts;

      float v = sqrt((msg->twist).linear.x * (msg->twist).linear.x + (msg->twist).linear.y * (msg->twist).linear.y);
      
      float teta;
      if ((msg->twist).linear.x == 0.0) 
        (msg->twist).linear.y >= 0.0 ? teta = M_PI / 2.0 : teta = - M_PI / 2.0;
      else if ((msg->twist).linear.x > 0.0)
        teta = atan((msg->twist).linear.y / (msg->twist).linear.x);
      else teta = atan((msg->twist).linear.y / (msg->twist).linear.x) + M_PI;

      if(this->integrationMode == 0) 
      {
        this->pose[0] = this->pose[0] + v * cos(this->pose[2] + teta) * ts;
        this->pose[1] = this->pose[1] + v * sin(this->pose[2] + teta) * ts;
      } else if (this->integrationMode == 1) 
      {
        this->pose[0] = this->pose[0] + v * cos(this->pose[2] + teta + (msg->twist).angular.z * ts / 2) * ts;
        this->pose[1] = this->pose[1] + v * sin(this->pose[2] + teta + (msg->twist).angular.z * ts / 2) * ts;
      } else {
        ROS_INFO("Error");
      }

        this->times[0] = (msg->header).stamp.sec;
        this->times[1] = (msg->header).stamp.nsec;
        
        nav_msgs::Odometry odometryMessage;

        // x,y position
        odometryMessage.pose.pose.position.x = this->pose[0];
        odometryMessage.pose.pose.position.y = this->pose[1];
        odometryMessage.pose.pose.position.z = 0.0;
        
        // orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, this->pose[2]); 
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
        odometryMessage.header.frame_id = "odom"; 
        odometryMessage.child_frame_id = "base_link"; 
        odometryPublisher.publish(odometryMessage);   

        transformStamped.header.frame_id = "odom"; 
        transformStamped.child_frame_id = "base_link"; 
   
        transformStamped.transform.translation.x = this->pose[0] - this->initialPose[0];
        transformStamped.transform.translation.y = this->pose[1] - this->initialPose[1];
        transformStamped.transform.translation.z = 0.0;
       
        tf2::Quaternion q1; 
        q1.setRPY(0, 0, this->pose[2] - this->initialPose[2]);
        transformStamped.transform.rotation.x = q1.x();
        transformStamped.transform.rotation.y = q1.y();
        transformStamped.transform.rotation.z = q1.z();
        transformStamped.transform.rotation.w = q1.w();

        br.sendTransform(transformStamped);   
    }
     
  }

  void setIntegrationMode(int integrationMethod) {
    this->integrationMode = integrationMethod;
   }

private:
  ros::NodeHandle n; 
  ros::Subscriber velocitiesSubcriber;
  ros::Subscriber positionSubscriber;
  ros::Publisher odometryPublisher;
  ros::ServiceServer service;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  int times[2];
  float pose[3];
  float initialPose[3];
  int integrationMode;
  bool firstUse = true;
};

void integrationMethodCallback(OdometryCalculator *my_odometryCalculator, project1::parametersConfig &config, uint32_t level) {
  if(config.method == 0 || config.method == 1)
  {
    if (config.method == 0)
      ROS_INFO("The integration method is Euler");
    else ROS_INFO("The integration method is Runge-Kutta");
    my_odometryCalculator->setIntegrationMode(config.method);
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "odometryCalculator");
  
  OdometryCalculator my_odometryCalculator;

  dynamic_reconfigure::Server<project1::parametersConfig> dynServer;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
  f = boost::bind(&integrationMethodCallback, &my_odometryCalculator, _1, _2);
  dynServer.setCallback(f);

  my_odometryCalculator.main_loop();

  return 0;
}
