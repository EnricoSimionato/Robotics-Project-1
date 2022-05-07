#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersCalibrationConfig.h>
#define M_PI 3.14159265358979323846

class CalibrationCalculator {
public:
  CalibrationCalculator() { 
    this->velocitiesSubscriber = n.subscribe("wheel_states", 1000, &CalibrationCalculator::computeOdometryCallback, this); 
    this->calibrationPublisher = n.advertise<std_msgs::Float32>("calibration", 1000);
    this->poseSubscriber = n.subscribe("robot/pose", 1000, &CalibrationCalculator::computeMetricCallback, this);
  }

  void main_loop() {
    ros::spin();
  }

  void computeOdometryCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if(!this->firstUse)
    {
      // The first time initialize ticks and times
      for(int i = 0; i < sizeof(this->ticks)/sizeof(float); i++)
        this->ticks[i] = msg->position[i];

      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;

      this->firstUse = true;
    }
    else
    {
      float wRuota[4];
      float ts = msg->header.stamp.sec - this->times[0] + (float) ((msg->header).stamp.nsec - this->times[1]) / 1000000000.0;

      // Calculate the angular velocity of each wheel [rad/s]
      for(int i = 0; i < sizeof(wRuota)/sizeof(float); i++)
      {
        wRuota[i] = (2.0 * M_PI * (msg->position[i] - this->ticks[i])) / 
          (this->N * this->T * ts);
      }

      // Calculate the linear velocity v [m/s]
      this->v[0] = (this->r * ((float) (wRuota[0] + wRuota[1] + wRuota[2] + wRuota[3]))) / 4.0;
      this->v[1] = (this->r * ((float) (- wRuota[0] + wRuota[1] + wRuota[2] - wRuota[3]))) / 4.0;
      this->v[2] = 0.0;
      
      // Calculate the angular velocity w [rad/s]
      this->w[0] = 0.0;
      this->w[1] = 0.0;
      this->w[2] = (this->r * ((float) (- wRuota[0] + wRuota[1] - wRuota[2] + wRuota[3]))) / (4.0 * (this->sum_lX_lY));
      
      // Update ticks and time
      for(int i = 0; i < sizeof(this->ticks)/sizeof(float); i++)
        this->ticks[i] = msg->position[i];
      
      // Calculate the odometry
      this->pose[2] = this->pose[2] + this->w[2] * ts;
      this->vel = sqrt(this->v[0] * this->v[0] + this->v[1] * this->v[1]);
      
      if (this->v[0] == 0.0) 
        this->v[1] >= 0.0 ? this->teta = M_PI / 2.0 : this->teta = - M_PI / 2.0;
      else if (this->v[0] > 0.0)
        this->teta = atan(this->v[1] / this->v[0]);
      else this->teta = atan(this->v[1]/ this->v[0]) + M_PI;

      this->pose[0] = this->pose[0] + this->vel * cos(this->pose[2] + this->teta) * ts;
      this->pose[1] = this->pose[1] + this->vel * sin(this->pose[2] + this->teta) * ts;

      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;
    }
  }

  void computeMetricCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if(!this->firstUsePose) {
        this->times[0] = (msg->header).stamp.sec;
        this->times[1] = (msg->header).stamp.nsec;

        this->pose[0] = msg->pose.position.x;
        this->pose[1] = msg->pose.position.y;

        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        this->pose[2] = yaw;

        this->firstUsePose = true;
    }
    else if (this->bag == 0)
    {
      float ts = msg->header.stamp.sec - this->times[0] + ((float) (msg->header).stamp.nsec) / 1000000000.0 - ((float) this->times[1]) / 1000000000.0;
      if (ts < 0.02 && ts > -0.02)
      {
        float dist = sqrt((msg->pose.position.x - this->pose[0]) * (msg->pose.position.x - this->pose[0]) + (msg->pose.position.y - this->pose[1]) * (msg->pose.position.y - this->pose[1])); 
      
        if (dist < 1000.0)
          this->distance += dist;
      }
    }
    else if (this->bag == 1)
    {
      float ts = msg->header.stamp.sec - this->times[0] + ((float) (msg->header).stamp.nsec) / 1000000000.0 - ((float) this->times[1]) / 1000000000.0;
      if (ts < 0.02 && ts > -0.02)
      {
       // float dist = sqrt((msg->pose.position.x - this->pose[0]) * (msg->pose.position.x - this->pose[0]) + (msg->pose.position.y - this->pose[1]) * (msg->pose.position.y - this->pose[1])); 
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        float dist;
        if (this->pose[2] > M_PI){
          dist = (yaw - (this->pose[2] - 2.0 * M_PI)) * (yaw - (this->pose[2] - 2.0 * M_PI));
        }
        else {
          dist = (yaw - this->pose[2]) * (yaw - this->pose[2]);
        }

        if (dist < 1000.0)
          this->distance += dist;
      }
    }
  }

  void setParameters(int N, float r, float sum_lX_lY, int bag) {
    this->N = N;
    this->r = r;
    this->sum_lX_lY = sum_lX_lY;
    this->bag = bag;
    this->firstUse = false;
    this->firstUsePose = false;

    std_msgs::Float32 distance;
    distance.data = this->distance;
    calibrationPublisher.publish(distance);

    this->distance = 0.0;
    this->times[0] = 0;
    this->times[1] = 0; 
  }

private:
  ros::NodeHandle n; 
  ros::Subscriber velocitiesSubscriber;
  ros::Subscriber poseSubscriber;
  ros::Subscriber initialPoseSubscriber;
  ros::Publisher calibrationPublisher;
  bool firstUse = false;
  bool firstUsePose = false;
  float vel;
  float teta;
  float ticks[4];
  int times[2] = {0};
  float v[3];
  float w[3];
  float pose[3];
  int bag = 0;
  int N = 42; 
  int T = 5; // gear ratio
  float r = 0.07;
  float sum_lX_lY = 0.369;
  float distance = 0.0;
};

void parametersCallback(CalibrationCalculator *my_calibrationCalculator, project1::parametersCalibrationConfig &config, uint32_t level) {
  ROS_INFO("The parameters are %d %f %f %d", config.N, config.r, config.sum_lX_lY, config.bag);
    my_calibrationCalculator->setParameters(config.N, config.r, config.sum_lX_lY, config.bag);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibrationCalculator");
  
  CalibrationCalculator my_calibrationCalculator;

  dynamic_reconfigure::Server<project1::parametersCalibrationConfig> dynServer;
  dynamic_reconfigure::Server<project1::parametersCalibrationConfig>::CallbackType f;
  f = boost::bind(&parametersCallback, &my_calibrationCalculator, _1, _2);
  dynServer.setCallback(f);

  my_calibrationCalculator.main_loop();

  return 0;
}
