#include "ros/ros.h"
#include "project1/Reset.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_client");
  if (argc != 2)
  {
    ROS_INFO("usage: reset_client new_x new_y new_theta");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<project1::Reset>("reset");
  project1::Reset srv;
  srv.request.new_x = atoll(argv[1]);
  srv.request.new_y = atoll(argv[2]);
  srv.request.new_theta = atoll(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Old x: %f", (float)srv.response.old_x);
    ROS_INFO("Old y: %f", (float)srv.response.old_y);
    ROS_INFO("Old theta: %f", (float)srv.response.old_theta);
  }
  else
  {
    ROS_ERROR("Failed to call service reset");
    return 1;
  }

  return 0;
}
