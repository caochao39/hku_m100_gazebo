#include "ros/ros.h"
#include <ros/console.h>

#include "std_msgs/String.h"

#include <dji_sdk/Acceleration.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Gimbal.h>


#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <tf/LinearMath/Quaternion.h>

#include <cmath>

#include <string>


geometry_msgs::Pose target_pose;
geometry_msgs::Twist target_twist;
geometry_msgs::Pose target_gimbal_pose;
geometry_msgs::Twist target_gimbal_twist;

gazebo_msgs::ModelState target_model_state;
gazebo_msgs::LinkState target_gimbal_state;

std::string model_name = "hku_m100";
std::string reference_frame = "world";

std::string gimbal_link_name = "camera_link";
std::string gimbal_reference_frame = "base_link";

ros::Subscriber attitude_quaternion_subscriber;
ros::Subscriber velocity_subscriber;
ros::Subscriber local_position_subscriber;
ros::Subscriber gimbal_orientation_subscriber;

ros::ServiceClient model_state_client;
ros::ServiceClient gimbal_state_client;

gazebo_msgs::SetModelState set_model_state;
gazebo_msgs::SetLinkState set_link_state;

bool velocity_updated = false;
bool position_updated = false;

double gimbal_pitch;
double gimbal_yaw;
double gimbal_roll;

tf::Quaternion gimbal_q;


void attitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr& attitude_quaternion_msg)
{
  target_pose.orientation.w = -attitude_quaternion_msg->q0;
  target_pose.orientation.x = -attitude_quaternion_msg->q1;
  target_pose.orientation.y = attitude_quaternion_msg->q2;
  target_pose.orientation.z = attitude_quaternion_msg->q3;

  target_twist.angular.x = attitude_quaternion_msg->wx;
  target_twist.angular.y = attitude_quaternion_msg->wy;
  target_twist.angular.z = attitude_quaternion_msg->wz;

  // std::cout << "attitude callback get called" << std::endl;

  
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
  target_pose.position.y = -position_msg->y;
  target_pose.position.z = position_msg->z;

  position_updated = true;
  // std::cout << "local position callback get called" << std::endl;

}


void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_orientation_msg)
{
  gimbal_pitch = gimbal_orientation_msg->pitch;
  gimbal_yaw = gimbal_orientation_msg->yaw;
  gimbal_roll = gimbal_orientation_msg->roll;

  //roll pitch yaw
  gimbal_q.setEuler(-gimbal_pitch / 180 * M_PI, -gimbal_roll / 180 * M_PI, gimbal_yaw / 180 * M_PI);

  target_gimbal_pose.orientation.w = gimbal_q.w();
  target_gimbal_pose.orientation.x = gimbal_q.x();
  target_gimbal_pose.orientation.y = gimbal_q.y();
  target_gimbal_pose.orientation.z = gimbal_q.z();

  target_gimbal_pose.position.x = 0.1;
  target_gimbal_pose.position.y = 0.0;
  target_gimbal_pose.position.z = -0.04;


  target_gimbal_twist.angular.x = 0;
  target_gimbal_twist.angular.y = 0;
  target_gimbal_twist.angular.z = 0;
  target_gimbal_twist.linear.x = 0;
  target_gimbal_twist.linear.y = 0;
  target_gimbal_twist.linear.z = 0;


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hku_m100_pcsim_gazebo_bridge");

  ros::NodeHandle n;

 
  attitude_quaternion_subscriber = n.subscribe("/dji_sdk/attitude_quaternion", 1000, attitudeQuaternionCallback);
  velocity_subscriber = n.subscribe("/dji_sdk/velocity", 1000, velocityCallback);
  local_position_subscriber = n.subscribe("/dji_sdk/local_position", 1000, localPositionCallback);
  gimbal_orientation_subscriber = n.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);

  model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);
  gimbal_state_client = n.serviceClient<gazebo_msgs::SetLinkState>("gazebo/set_link_state", true);

  ROS_INFO("Bridge between PC sim and gazebo connected");

  ros::Rate spin_rate(200);

  while(ros::ok())
  {
    ros::spinOnce();

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
      ROS_INFO("update model state failed.");
    }
    
    // if(gimbal_state_client)
    // {
    //   target_gimbal_state.link_name = gimbal_link_name;
    //   target_gimbal_state.reference_frame = gimbal_reference_frame;
    //   target_gimbal_state.pose = target_gimbal_pose;
    //   target_gimbal_state.twist = target_gimbal_twist;
    //   set_link_state.request.link_state = target_gimbal_state;
    //   gimbal_state_client.call(set_link_state);
    // }
    // else
    // {
    //   ROS_INFO("update gimbal state failed.");
    // }

    spin_rate.sleep();
  }

  return 0;
}