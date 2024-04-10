/* * @Author: zhaoyu  * @Date: 2024-04-05 16:25:46  * @Last Modified by:   zhaoyu  * @Last Modified time: 2024-04-05 16:25:46  */

#ifndef MCL_H
#define MCL_H
#endif

#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <random>
#include "tool.h"
#include <cmath>

class mcl
{
  struct Particle
  {
    Eigen::Matrix4f pose_;
    float weight_;
    Eigen::Matrix4Xf laser_;
  };

private:
  int m_sync_cnt_;

  std::random_device rd_;
  std::mt19937 gen_;

  float imageResolution_;
  float mapCenterX_;
  float mapCenterY_;

  float odomCovariance_[6];

  int numParticles_;

  std::vector<Particle> particles_;

  Particle maxProbParicle_; // the most likely particle

  cv::Mat gridMap_;
  cv::Mat gridMapCV_;

  Eigen::Matrix4f tf_laser2robot_;
  Eigen::Matrix4f odomBefore_;

  float minOdomDistance_;
  float minOdomAngle_;

  int repropagateCountNeeded_;

  bool isOdomInitialized_;

  int predictionCounter_;

public:
  mcl();
  virtual ~mcl() = default;
  void initializeParticles();

  void prediction(Eigen::Matrix4f diffPose_); // predict with odometry
  void correction(Eigen::Matrix4Xf laser_);   // correcting the weights;

  void setOdom(nav_msgs::Odometry odom);
  void setLaser(sensor_msgs::LaserScan laser);

  void resampling(); // resample with systematic resampling method

  void update(Eigen::Matrix4f pose_, Eigen::Matrix4Xf laser_);
  void showInMap();
};
