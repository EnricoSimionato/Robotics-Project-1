#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
//#include "Vec3.h"
#define M_PI 3.14159265358979323846

class VelocityCalculator {
public:
  VelocityCalculator() { 
    this->sub = n.subscribe("wheel_states", 1000, &VelocityCalculator::computeVelocityCallback, this); // mettere lo / in wheel_states?
    this->velocitiesPublisher = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }

  void main_loop() {
    ros::spin();
  }

  void computeVelocityCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    if(!this->firstUse)
    {
      // The first time initialize ticks and times
      for(int i = 0; i < sizeof(this->ticks)/sizeof(float); i++)
        this->ticks[i] = msg->position[i];

      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;

      this->firstUse = true;

      geometry_msgs::TwistStamped velocitiesMessage;
      velocitiesMessage.header.stamp.sec = this->times[0];
      velocitiesMessage.header.stamp.nsec = this->times[1];


      velocitiesPublisher.publish(velocitiesMessage);
    }
    else
    {
      float wRuota[4];

      // Calculate the angular velocity of each wheel [rad/s]
      for(int i = 0; i < sizeof(wRuota)/sizeof(float); i++)
      {
        wRuota[i] = (2.0 * M_PI * (msg->position[i] - this->ticks[i])) / 
            (this->N * this->T * (((msg->header).stamp.sec - this->times[0]) + ((float) ((msg->header).stamp.nsec - this->times[1]) / 1000000000.0)));
      }

      // Calculate the linear velocity v [m/s]
      this->v[0] = (this->r * ((float) (wRuota[0] + wRuota[1] + wRuota[2] + wRuota[3]))) / 4.0;
      this->v[1] = (this->r * ((float) (- wRuota[0] + wRuota[1] + wRuota[2] - wRuota[3]))) / 4.0;
      this->v[2] = 0.0;
      
      // Calculate the angular velocity w [rad/s]
      this->w[0] = 0.0;
      this->w[1] = 0.0;
      this->w[2] = (this->r * ((float) (- wRuota[0] + wRuota[1] - wRuota[2] + wRuota[3]))) / (4.0 * (this->lX + this->lY));
      
      ROS_INFO("w ruota 1: %f", wRuota[0]);
      ROS_INFO("w ruota 2: %f", wRuota[1]);
      ROS_INFO("w ruota 3: %f", wRuota[2]);
      ROS_INFO("w ruota 4: %f", wRuota[3]);
      ROS_INFO("w ruota 1 reale: %f", msg->velocity[0] / (60 * this->T)); // in [rad/s]
      ROS_INFO("w ruota 2 reale: %f", msg->velocity[1] / (60 * this->T));
      ROS_INFO("w ruota 3 reale: %f", msg->velocity[2] / (60 * this->T));
      ROS_INFO("w ruota 4 reale: %f", msg->velocity[3] / (60 * this->T));
      //ROS_INFO("v in x: %f", this->v.x);
      //ROS_INFO("v in y: %f", this->v.y);
      //ROS_INFO("w in z: %f", this->w.z);
      ROS_INFO("v in x: %f", this->v[0]);
      ROS_INFO("v in y: %f", this->v[1]);
      ROS_INFO("w in z: %f", this->w[2]);
      
      // Update ticks and time
      for(int i = 0; i < sizeof(this->ticks)/sizeof(float); i++)
        this->ticks[i] = msg->position[i];

      this->times[0] = (msg->header).stamp.sec;
      this->times[1] = (msg->header).stamp.nsec;

      geometry_msgs::TwistStamped velocitiesMessage;
      //velocitiesMessage.twist.linear = this->v;
      //velocitiesMessage.twist.angular = this->w
      velocitiesMessage.twist.linear.x = this->v[0];
      velocitiesMessage.twist.linear.y = this->v[1];
      velocitiesMessage.twist.linear.z = this->v[2];

      velocitiesMessage.twist.angular.x = this->w[0];
      velocitiesMessage.twist.angular.y = this->w[1];
      velocitiesMessage.twist.angular.z = this->w[2];

      velocitiesMessage.header.stamp.sec = this->times[0];
      velocitiesMessage.header.stamp.nsec = this->times[1];


      velocitiesPublisher.publish(velocitiesMessage);
    }
  }

private:
  ros::NodeHandle n; 
  ros::Subscriber sub;
  ros::Publisher velocitiesPublisher;
  bool firstUse = false;
  float ticks[4];
  int times[2];
  float v[3];
  float w[3];
  //Vec3 v(0, 0, 0);
  //Vec3 w(0, 0, 0);
  int N = 42; // encoder resolution
  int T = 5; // gear ratio
  float r = 0.07;
  float lX = 0.2;
  float lY = 0.169;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocityCalculator");
  
  VelocityCalculator my_velocityCalculator;

  my_velocityCalculator.main_loop();

  return 0;
}
