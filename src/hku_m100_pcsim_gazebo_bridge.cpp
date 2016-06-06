#include "ros/ros.h"
#include <ros/console.h>

#include "std_msgs/String.h"

#include <dji_sdk/Acceleration.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/LocalPosition.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <string>


geometry_msgs::Pose target_pose;
geometry_msgs::Twist target_twist;
gazebo_msgs::ModelState target_model_state;

std::string model_name = "hku_m100";
std::string reference_frame = "world";

ros::Subscriber attitude_quaternion_subscriber;
ros::Subscriber velocity_subscriber;
ros::Subscriber local_position_subscriber;

ros::ServiceClient model_state_client;

gazebo_msgs::SetModelState set_model_state;

bool velocity_updated = false;
bool position_updated = false;


void attitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr& attitude_quaternion_msg)
{
  target_pose.orientation.w = attitude_quaternion_msg->q0;
  target_pose.orientation.x = attitude_quaternion_msg->q1;
  target_pose.orientation.y = attitude_quaternion_msg->q2;
  target_pose.orientation.z = attitude_quaternion_msg->q3;

  target_twist.angular.x = attitude_quaternion_msg->wx;
  target_twist.angular.y = attitude_quaternion_msg->wy;
  target_twist.angular.z = attitude_quaternion_msg->wz;

  // std::cout << "attitude callback get called" << std::endl;

  if(velocity_updated == true && position_updated == true)
  {
    if(model_state_client)
    {
      target_model_state.model_name = model_name;
      target_model_state.reference_frame = reference_frame;
      target_model_state.pose = target_pose;
      target_model_state.twist = target_twist;
      set_model_state.request.model_state = target_model_state;
      model_state_client.call(set_model_state);
      // std::cout << "service called" << std::endl;
    }
    else
    {
      //TODO: change to ROS_ERROR
      // std::cout << "connection with service lost!!" << std::endl;
    }
  }
}

void velocityCallback(const dji_sdk::Velocity::ConstPtr& velocity_msg)
{
  target_twist.linear.x  = velocity_msg->vx;
  target_twist.linear.y  = velocity_msg->vy;
  target_twist.linear.z  = velocity_msg->vz;

  velocity_updated = true;
  // std::cout << "velocity callback get called" << std::endl;

}

void localPositionCallback(const dji_sdk::LocalPosition::ConstPtr& position_msg)
{
  target_pose.position.x = position_msg->x;
  target_pose.position.y = position_msg->y;
  target_pose.position.z = position_msg->z;

  position_updated = true;
  // std::cout << "local position callback get called" << std::endl;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hku_m100_pcsim_gazebo_bridge");

  ros::NodeHandle n;

 
  attitude_quaternion_subscriber = n.subscribe("/dji_sdk/attitude_quaternion", 1000, attitudeQuaternionCallback);
  velocity_subscriber = n.subscribe("/dji_sdk/velocity", 1000, velocityCallback);
  local_position_subscriber = n.subscribe("/dji_sdk/local_position", 1000, localPositionCallback);

  model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);

  ROS_INFO("Bridge between PC sim and gazebo connected");

  ros::spin();

  return 0;
}