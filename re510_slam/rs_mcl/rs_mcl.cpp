/* * @Author: zhaoyu  * @Date: 2024-04-05 19:34:04  * @Last Modified by:   zhaoyu  * @Last Modified time: 2024-04-05 19:34:04  */
#include <ros/ros.h>

#include "src/mcl.h"

std::vector<Eigen::Matrix4f> vec_poses_;
std::vector<double> vec_poses_time_;
std::vector<Eigen::Matrix4Xf> vec_lasers_;
std::vector<double> vec_lasers_time_;

mcl mclsitmulator_;

void callbackPose(const nav_msgs::Odometry::ConstPtr &msg)
{

  Eigen::Matrix4f pose_;

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);

  pose_ << m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
      m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
      m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
      0, 0, 0, 1;

  vec_poses_.push_back(pose_);
  vec_poses_time_.push_back(msg->header.stamp.toSec());

  ROS_INFO("pose: %f", msg->header.stamp.toSec());

  checkData();
}
void callackLaser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  int scanQuantity_ = (msg->angle_max - msg->angle_min) / msg->angle_increment + 1;

  Eigen::Matrix4Xf laser_ = Eigen::Matrix4Xf::Ones(4, 1);

  int scanEffective_ = 0;
  for (int i = 0; i < scanQuantity_; i++)
  {
    float dist_ = msg->ranges[i];

    if (dist_ > 1 && dist_ < 10)
    {
      laser_(0, scanEffective_ - 1) = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
      laser_(1, scanEffective_ - 1) = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
      laser_(2, scanEffective_ - 1) = 0;
      laser_(3, scanEffective_ - 1) = 1;
      scanEffective_++;
    }
  }

  vec_lasers_.push_back(laser_);
  vec_lasers_time_.push_back(msg->header.stamp.toSec());

  ROS_INFO("laser: %f", msg->header.stamp.toSec());

  checkData();
}

void checkData()
{

  if (vec_poses_.size() == 0 || vec_lasers_.size() == 0)
  {
    ROS_WARN("Data is not ready");
  }

  while (!vec_poses_.empty() && !vec_lasers_.empty())
  {
    if (fabs(vec_poses_time_[0] - vec_lasers_time_[0]) > 0.1)
    {
      if (vec_poses_time_[0] < vec_lasers_time_[0])
      {
        vec_poses_.erase(vec_poses_.begin());
        vec_poses_time_.erase(vec_poses_time_.begin());
      }
      else
      {
        vec_lasers_.erase(vec_lasers_.begin());
        vec_lasers_time_.erase(vec_lasers_time_.begin());
      }
    }
    else
    {
      mclsitmulator_.update(vec_poses_[0], vec_lasers_[0]);

      vec_poses_.erase(vec_poses_.begin());
      vec_poses_time_.erase(vec_poses_time_.begin());
      vec_lasers_.erase(vec_lasers_.begin());
      vec_lasers_time_.erase(vec_lasers_time_.begin());
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_mcl");

  ros::NodeHandle nh;
  nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, callackLaser);
  nh.subscribe<nav_msgs::Odometry>("/odom", 100, callbackPose);

  ros::spin();
}