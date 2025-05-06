#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#define M_PI 3.14159265358979323846

class VelocityCalculator {
public:
  VelocityCalculator() { 
    this->sub = n.subscribe("wheel_states", 1000, &VelocityCalculator::computeVelocityCallback, this); 
    this->velocitiesPublisher = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }

  void main_loop() {
    ros::spin();
  }

  void computeVelocityCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    if(!this->firstUse)
    {
      // The first call initializes ticks and times
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
      if(this->counter == 3) {
        float wRuota[4];

        // Calculate the angular velocity of each wheel [rad/s]
        for(int i = 0; i < sizeof(wRuota)/sizeof(float); i++)
        {
          wRuota[i] = (2.0 * M_PI * (msg->position[i] - this->ticks[i])) / 
              (this->N * this->T * (((msg->header).stamp.sec - this->times[0]) + ((float) (msg->header).stamp.nsec) / 1000000000.0 - ((float) this->times[1]) / 1000000000.0));
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

        this->times[0] = (msg->header).stamp.sec;
        this->times[1] = (msg->header).stamp.nsec;

        geometry_msgs::TwistStamped velocitiesMessage;
        velocitiesMessage.twist.linear.x = this->v[0];
        velocitiesMessage.twist.linear.y = this->v[1];
        velocitiesMessage.twist.linear.z = this->v[2];

        velocitiesMessage.twist.angular.x = this->w[0];
        velocitiesMessage.twist.angular.y = this->w[1];
        velocitiesMessage.twist.angular.z = this->w[2];

        velocitiesMessage.header.stamp.sec = this->times[0];
        velocitiesMessage.header.stamp.nsec = this->times[1];


        velocitiesPublisher.publish(velocitiesMessage);
        this->counter = 0;
      } else {
        this->counter++;
      }
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
  int N = 41; 
  int T = 5; 
  float r = 0.075;
  float sum_lX_lY = 0.36;
  int counter = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocityCalculator");
  
  VelocityCalculator my_velocityCalculator;

  my_velocityCalculator.main_loop();

  return 0;
}
