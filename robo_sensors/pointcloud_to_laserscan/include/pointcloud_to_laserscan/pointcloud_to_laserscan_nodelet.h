/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H

#include <boost/thread/mutex.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>

namespace pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;



/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
class PointCloudToLaserScanNodelet : public nodelet::Nodelet
{
public:
  PointCloudToLaserScanNodelet();

private:
  virtual void onInit();

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                 tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  void cloud_combine(const sensor_msgs::PointCloud2ConstPtr& left_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& right_cloud_msg);
  void connectCb();

  void disconnectCb();

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pub_;
  ros::Publisher cloud_merged_pub_;
  boost::mutex connect_mutex_;

  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_left_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_right_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> *timeSynchronizer_;

  boost::shared_ptr<MessageFilter> lidar_left_message_filter_;
  boost::shared_ptr<MessageFilter> lidar_right_message_filter_;

  //app time
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> appTimeSyncPolicy_;
  typedef message_filters::Synchronizer<appTimeSyncPolicy_> Sync;
  boost::shared_ptr<Sync> sync_;

  // ROS Parameters
  unsigned int input_queue_size_;
  std::string target_frame_;
  std::string lidar_left_topic_;
  std::string lidar_right_topic_;
  double tolerance_;
  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
  bool use_inf_;
  double inf_epsilon_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H
